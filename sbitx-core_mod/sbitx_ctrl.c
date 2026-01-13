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
                replyf(fp, "OK\n");
                continue;
            } else if (*p == '0') {
                pthread_mutex_lock(&g_lock);
                set_ptt_locked(0);
                pthread_mutex_unlock(&g_lock);
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
