/* sbitx_ctrl.c
 *
 * Minimal TCP control shim for SoapySBITX (and other clients).
 *
 * TCP commands (newline terminated):
 *   f            -> get frequency (Hz)
 *   F <hz>       -> set frequency (Hz)
 *   t            -> get PTT state (0/1)   [software PTT]
 *   T 0|1        -> set PTT state (0=RX,1=TX)  [software PTT]
 *   p            -> get power/SWR snapshot (only valid in TX)
 *                  returns: "P <fwd_W10> <ref_W10> <swr_10>\n"
 *                  where values are tenths (e.g. 37 => 3.7)
 *
 * Notes:
 * - This does NOT implement CAT; it is a tiny private control plane.
 * - If you also use hardware PTT (key_down), you can OR it in here,
 *   but for Soapy drivers it's usually cleaner to keep "software PTT"
 *   as the one truth and let the hardware PTT be handled elsewhere.
 */

#include <arpa/inet.h>
#include <errno.h>
#include <pthread.h>
#include <signal.h>
#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#include "sbitx_core.h"

/* ---------------- ALSA coarse band trims (Master/Capture) ----------------
 * Optional: load /etc/sbitx/alsamix_bands.ini (or ./alsamix_bands.ini).
 * Applies coarse ALSA mixer settings on band change and PTT transitions.
 * Only touches:
 *   - 'Master'  (0..127)  : coarse TX IF drive (and/or RX speaker baseline)
 *   - 'Capture' (0..31)   : coarse RX IF / mic (line) level
 */

typedef struct {
    long long min_hz;
    long long max_hz;
    int rx_master;
    int rx_capture;
    int tx_master;
    int tx_capture;
    char name[32];
} band_cfg_t;

#define MAX_BANDS 32

typedef struct {
    char card[32];          /* amixer -c <card> */
    int has_input_mux;
    char input_mux[32];

    int rx_master_def;
    int rx_capture_def;
    int tx_master_def;
    int tx_capture_def;

    int nbands;
    band_cfg_t bands[MAX_BANDS];
} alsa_cfg_t;

static alsa_cfg_t g_alsa_cfg;
static int g_alsa_loaded = 0;

/* cache last applied values to avoid spamming amixer */
static int g_last_master = -9999;
static int g_last_capture = -9999;
static int g_last_ptt = -1;
static int g_last_band = -2;

static long long current_freq = 0;
static int ptt_state = 0;

static void strtrim(char *s)
{
    if (!s) return;
    char *p = s;
    while (*p==' '||*p=='\t'||*p=='\r'||*p=='\n') p++;
    if (p != s) memmove(s, p, strlen(p)+1);
    size_t n = strlen(s);
    while (n>0 && (s[n-1]==' '||s[n-1]=='\t'||s[n-1]=='\r'||s[n-1]=='\n')) {
        s[n-1]=0; n--;
    }
}

static int clampi(int v, int lo, int hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static void amixer_set(const char *ctl, int val)
{
    char cmd[256];
    snprintf(cmd, sizeof(cmd), "amixer -q -c %s sset '%s' %d",
             g_alsa_cfg.card[0] ? g_alsa_cfg.card : "audioinjectorpi",
             ctl, val);
    (void)system(cmd);
}

static int find_band(long long hz)
{
    for (int i = 0; i < g_alsa_cfg.nbands; i++) {
        if (hz >= g_alsa_cfg.bands[i].min_hz && hz <= g_alsa_cfg.bands[i].max_hz)
            return i;
    }
    return -1;
}

static void apply_profile(int ptt_on, long long hz)
{
    if (!g_alsa_loaded) return;

    int b = find_band(hz);

    int want_master = ptt_on ? g_alsa_cfg.tx_master_def : g_alsa_cfg.rx_master_def;
    int want_capture = ptt_on ? g_alsa_cfg.tx_capture_def : g_alsa_cfg.rx_capture_def;

    if (b >= 0) {
        const band_cfg_t *bc = &g_alsa_cfg.bands[b];
        want_master  = ptt_on ? bc->tx_master  : bc->rx_master;
        want_capture = ptt_on ? bc->tx_capture : bc->rx_capture;
    }

    want_master = clampi(want_master, 0, 127);
    want_capture = clampi(want_capture, 0, 31);

    if (g_last_master == want_master && g_last_capture == want_capture &&
        g_last_ptt == ptt_on && g_last_band == b)
        return;

    g_last_master = want_master;
    g_last_capture = want_capture;
    g_last_ptt = ptt_on;
    g_last_band = b;

    if (g_alsa_cfg.has_input_mux && g_alsa_cfg.input_mux[0]) {
        char cmd[256];
        snprintf(cmd, sizeof(cmd), "amixer -q -c %s cset name='Input Mux' '%s' >/dev/null 2>&1",
                 g_alsa_cfg.card[0] ? g_alsa_cfg.card : "audioinjectorpi",
                 g_alsa_cfg.input_mux);
        (void)system(cmd);
        g_alsa_cfg.has_input_mux = 0;
    }

    amixer_set("Capture", want_capture);
    amixer_set("Master", want_master);
}

static int parse_kv(char *line, char *key, size_t ksz, char *val, size_t vsz)
{
    char *eq = strchr(line, '=');
    if (!eq) return 0;
    *eq = 0;
    strncpy(key, line, ksz-1); key[ksz-1]=0;
    strncpy(val, eq+1, vsz-1); val[vsz-1]=0;
    strtrim(key);
    strtrim(val);
    return key[0] != 0;
}

static int alsa_cfg_load_file(const char *path)
{
    FILE *fp = fopen(path, "r");
    if (!fp) return 0;

    memset(&g_alsa_cfg, 0, sizeof(g_alsa_cfg));
    strncpy(g_alsa_cfg.card, "audioinjectorpi", sizeof(g_alsa_cfg.card)-1);

    g_alsa_cfg.rx_master_def = 70;
    g_alsa_cfg.rx_capture_def = 21;
    g_alsa_cfg.tx_master_def = 90;
    g_alsa_cfg.tx_capture_def = 26;

    char section[64] = "global";
    band_cfg_t cur; memset(&cur, 0, sizeof(cur));
    int in_band = 0;

    char buf[256];
    while (fgets(buf, sizeof(buf), fp)) {
        char *c = strchr(buf, '#'); if (c) *c = 0;
        c = strchr(buf, ';'); if (c) *c = 0;
        strtrim(buf);
        if (buf[0] == 0) continue;

        if (buf[0] == '[') {
            if (in_band && g_alsa_cfg.nbands < MAX_BANDS) {
                g_alsa_cfg.bands[g_alsa_cfg.nbands++] = cur;
            }
            in_band = 0;
            memset(&cur, 0, sizeof(cur));

            char *r = strchr(buf, ']');
            if (!r) continue;
            *r = 0;
            strncpy(section, buf+1, sizeof(section)-1);
            section[sizeof(section)-1]=0;
            strtrim(section);

            if (strncmp(section, "band", 4) == 0) {
                in_band = 1;
                strncpy(cur.name, section, sizeof(cur.name)-1);
                cur.rx_master = g_alsa_cfg.rx_master_def;
                cur.rx_capture = g_alsa_cfg.rx_capture_def;
                cur.tx_master = g_alsa_cfg.tx_master_def;
                cur.tx_capture = g_alsa_cfg.tx_capture_def;
            }
            continue;
        }

        char key[64], val[128], tmp[256];
        strncpy(tmp, buf, sizeof(tmp)-1); tmp[sizeof(tmp)-1]=0;
        if (!parse_kv(tmp, key, sizeof(key), val, sizeof(val))) continue;

        if (!in_band) {
            if (!strcmp(key, "card")) strncpy(g_alsa_cfg.card, val, sizeof(g_alsa_cfg.card)-1);
            else if (!strcmp(key, "input_mux")) { g_alsa_cfg.has_input_mux=1; strncpy(g_alsa_cfg.input_mux, val, sizeof(g_alsa_cfg.input_mux)-1); }
            else if (!strcmp(key, "rx_master")) g_alsa_cfg.rx_master_def = atoi(val);
            else if (!strcmp(key, "rx_capture")) g_alsa_cfg.rx_capture_def = atoi(val);
            else if (!strcmp(key, "tx_master")) g_alsa_cfg.tx_master_def = atoi(val);
            else if (!strcmp(key, "tx_capture")) g_alsa_cfg.tx_capture_def = atoi(val);
        } else {
            if (!strcmp(key, "min_hz")) cur.min_hz = atoll(val);
            else if (!strcmp(key, "max_hz")) cur.max_hz = atoll(val);
            else if (!strcmp(key, "rx_master")) cur.rx_master = atoi(val);
            else if (!strcmp(key, "rx_capture")) cur.rx_capture = atoi(val);
            else if (!strcmp(key, "tx_master")) cur.tx_master = atoi(val);
            else if (!strcmp(key, "tx_capture")) cur.tx_capture = atoi(val);
        }
    }

    if (in_band && g_alsa_cfg.nbands < MAX_BANDS) g_alsa_cfg.bands[g_alsa_cfg.nbands++] = cur;

    fclose(fp);

    for (int i=0;i<g_alsa_cfg.nbands;i++){
        if (g_alsa_cfg.bands[i].min_hz <= 0 || g_alsa_cfg.bands[i].max_hz <= 0 ||
            g_alsa_cfg.bands[i].max_hz < g_alsa_cfg.bands[i].min_hz) {
            for (int j=i+1;j<g_alsa_cfg.nbands;j++) g_alsa_cfg.bands[j-1]=g_alsa_cfg.bands[j];
            g_alsa_cfg.nbands--; i--;
        }
    }

    g_alsa_loaded = 1;
    return 1;
}

static void alsa_cfg_try_load(void)
{
    if (g_alsa_loaded) return;
    if (alsa_cfg_load_file("/etc/sbitx/alsamix_bands.ini")) return;
    (void)alsa_cfg_load_file("alsamix_bands.ini");
}

#define LISTEN_IP   "127.0.0.1"
#define LISTEN_PORT 9999

static volatile int g_shutdown = 0;

static radio g_radio;
static pthread_mutex_t g_lock = PTHREAD_MUTEX_INITIALIZER;

/* software PTT (from TCP) */
static int g_soft_ptt = 0;

/* ---------- helpers ---------- */

static void on_sigint(int sig)
{
    (void)sig;
    g_shutdown = 1;
}

static void set_freq_locked(uint32_t hz)
{
    set_frequency(&g_radio, hz);
    /* optional LPF control if present in your sbitx_core */
    lpf_off(&g_radio);
    lpf_set(&g_radio);
}

static uint32_t get_freq_locked(void)
{
    return g_radio.frequency;
}

static void set_ptt_locked(int on)
{
    g_soft_ptt = on ? 1 : 0;
    tr_switch(&g_radio, g_soft_ptt ? IN_TX : IN_RX);
}

static int get_ptt_locked(void)
{
    return g_soft_ptt;
}

/* Return 1 if values valid, 0 otherwise */
static int get_power_locked(int *fwd_w10, int *ref_w10, int *swr_10)
{
    if (g_radio.txrx_state != IN_TX)
        return 0;

    if (!update_power_measurements(&g_radio))
        return 0;

    *fwd_w10 = get_fwd_power(&g_radio);
    *ref_w10 = get_ref_power(&g_radio);
    *swr_10  = get_swr(&g_radio);
    return 1;
}

static void replyf(FILE *fp, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vfprintf(fp, fmt, ap);
    va_end(ap);
    fflush(fp);
}

/* ---------- TCP client thread ---------- */

static void *client_thread(void *arg)
{
    int fd = (int)(intptr_t)arg;

    FILE *fp = fdopen(fd, "r+");
    if (!fp) {
        close(fd);
        return NULL;
    }

    char line[256];

    while (!g_shutdown && fgets(line, sizeof(line), fp)) {
        /* trim CR/LF */
        // debug: print every received command line
        if (getenv("SBITX_CTRL_DEBUG")) {
        fprintf(stderr, "[sbitx_ctrl] fd=%d cmd='%s'\n", fd, line);
        fflush(stderr);
    }
        size_t n = strlen(line);
        while (n && (line[n-1] == '\n' || line[n-1] == '\r')) {
            line[--n] = 0;
        }

        if (n == 0)
            continue;

        /* f => read frequency */
        if (line[0] == 'f' && line[1] == 0) {
            pthread_mutex_lock(&g_lock);
            uint32_t hz = get_freq_locked();
            pthread_mutex_unlock(&g_lock);
            current_freq = (long long)hz;
            replyf(fp, "%u\n", hz);
            continue;
        }

        /* F <hz> => set frequency */
        if (line[0] == 'F') {
            const char *p = line + 1;
            while (*p == ' ' || *p == '\t') p++;
            if (*p == 0) {
                replyf(fp, "ERR missing\n");
                continue;
            }
            uint32_t hz = (uint32_t)strtoul(p, NULL, 10);
            if (hz < 100000 || hz > 70000000) {
                replyf(fp, "ERR range\n");
                continue;
            }
            pthread_mutex_lock(&g_lock);
            set_freq_locked(hz);
            uint32_t now = get_freq_locked();
            pthread_mutex_unlock(&g_lock);
            replyf(fp, "OK %u\n", now);
            continue;
        }

        /* t => get software PTT */
        if (line[0] == 't' && line[1] == 0) {
            pthread_mutex_lock(&g_lock);
            int on = get_ptt_locked();
            pthread_mutex_unlock(&g_lock);
            replyf(fp, "%d\n", on);
            continue;
        }

        /* T 0|1 => set software PTT */
        if (line[0] == 'T') {
            const char *p = line + 1;
            while (*p == ' ' || *p == '\t') p++;
            if (*p == '1') {
                pthread_mutex_lock(&g_lock);
                set_ptt_locked(1);
                pthread_mutex_unlock(&g_lock);
                ptt_state = 1;
                alsa_cfg_try_load();
                apply_profile(ptt_state, current_freq);
                replyf(fp, "OK\n");
                continue;
            } else if (*p == '0') {
                pthread_mutex_lock(&g_lock);
                set_ptt_locked(0);
                pthread_mutex_unlock(&g_lock);
                ptt_state = 0;
                alsa_cfg_try_load();
                apply_profile(ptt_state, current_freq);
                replyf(fp, "OK\n");
                continue;
            } else {
                replyf(fp, "ERR\n");
                continue;
            }
        }

        /* p => power/swr snapshot */
        if (line[0] == 'p' && line[1] == 0) {
            int fwd=0, ref=0, swr=0;
            pthread_mutex_lock(&g_lock);
            int ok = get_power_locked(&fwd, &ref, &swr);
            pthread_mutex_unlock(&g_lock);
            if (!ok) {
                replyf(fp, "ERR\n");
            } else {
                replyf(fp, "P %d %d %d\n", fwd, ref, swr);
            }
            continue;
        }

        replyf(fp, "?\n");
    }

    fclose(fp); /* closes fd */
    return NULL;
}

/* ---------- TCP server ---------- */

static void *server_thread(void *arg)
{
    (void)arg;

    int listen_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (listen_fd < 0) {
        perror("socket");
        return NULL;
    }

    int one = 1;
    setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(LISTEN_PORT);
    inet_pton(AF_INET, LISTEN_IP, &addr.sin_addr);

    if (bind(listen_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("bind");
        close(listen_fd);
        return NULL;
    }

    if (listen(listen_fd, 5) < 0) {
        perror("listen");
        close(listen_fd);
        return NULL;
    }

    fprintf(stderr, "sbitx_ctrl listening on %s:%d\n", LISTEN_IP, LISTEN_PORT);

    while (!g_shutdown) {
        int cfd = accept(listen_fd, NULL, NULL);
        if (cfd < 0) {
            if (errno == EINTR) continue;
            perror("accept");
            continue;
        }
        pthread_t th;
        pthread_create(&th, NULL, client_thread, (void *)(intptr_t)cfd);
        pthread_detach(th);
    }

    close(listen_fd);
    return NULL;
}

/* ---------- main ---------- */

int main(int argc, char **argv)
{
    (void)argc; (void)argv;

    signal(SIGINT, on_sigint);

    memset(&g_radio, 0, sizeof(g_radio));
    /* match your platform */
    strcpy(g_radio.i2c_device, "/dev/i2c-22");
    g_radio.bfo_frequency = 40037000;
    g_radio.bridge_compensation = 100;

    hw_init(&g_radio);

    /* sane defaults */
    pthread_mutex_lock(&g_lock);
    set_freq_locked(7100000);
    set_ptt_locked(0);
    pthread_mutex_unlock(&g_lock);

    pthread_t srv;
    pthread_create(&srv, NULL, server_thread, NULL);

    /* just idle until ctrl-c */
    while (!g_shutdown) {
        usleep(200000);
    }

    pthread_mutex_lock(&g_lock);
    set_ptt_locked(0);
    pthread_mutex_unlock(&g_lock);

    hw_shutdown(&g_radio);
    fprintf(stderr, "sbitx_ctrl exiting.\n");
    return 0;
}
