/* sBitx core
 * Copyright (C) 2023 Rhizomatica
 * Author: Rafael Diniz <rafael@riseup.net>
 */

// Added minimal audio bring-up:
//   - setup_audio_codec()
//   - core_audio_init() / core_audio_shutdown()
// These wrap the basic ALSA mixer setup and start the duplex sound thread,
// without pulling in DSP, modem, or UI code.

#include "sbitx_core.h"
#include "sbitx_i2c.h"
#include "sbitx_gpio.h"
#include "sbitx_si5351.h"
#include "sound.h"          // for sound_mixer, sound_thread_start
#include <string.h>
#include <wiringPi.h>
#include <unistd.h>

// Audio bring-up (no DSP/UI) 
// added to eliminate need to have started sBitx to perform initialization
// sbitx_ctrl.c should call core_audio_init() once when starting
// calling core_audio_shutdown seems polite but we don't do it

static char audio_card[32] = "hw:0";
static const char *audio_thread_device = "plughw:CARD=audioinjectorpi,DEV=0";

// Mixer setup lifted (minimally) from sbitx.c
static void setup_audio_codec(void)
{
    sound_mixer(audio_card, "Input Mux", 0);
    sound_mixer(audio_card, "Line", 1);
    sound_mixer(audio_card, "Mic", 0);
    sound_mixer(audio_card, "Mic Boost", 0);
    sound_mixer(audio_card, "Playback Deemphasis", 0);

    sound_mixer(audio_card, "Master", 10);
    sound_mixer(audio_card, "Output Mixer HiFi", 1);
    sound_mixer(audio_card, "Output Mixer Mic Sidetone", 0);
}

// Call this if you want audio; keep hw_init() usable without audio
int core_audio_init(void)
{
    setup_audio_codec();
    return sound_thread_start(audio_thread_device);
}

// Provide a shutdown hook if your sound layer has one; otherwise no-op
void core_audio_shutdown(void)
{
    // If sound_thread_stop() exists, call it here
}

// Hardware-layer functions (unchanged)

void hw_init(radio *radio_h)
{
    // I2C SETUP
    i2c_open(radio_h);

    // GPIO SETUP
    gpio_init(radio_h);

    // Si5351 SETUP
    setup_oscillators(radio_h);
}

void hw_shutdown(radio *radio_h)
{
    i2c_close(radio_h);

    // WiringPi has no function to close/shutdown resources
    // should we stop the Si5351 clocks?
}

// reads the power measurements from I2C bus
bool update_power_measurements(radio *radio_h)
{
    uint8_t response[4];
    uint16_t vfwd, vref;

    int count = i2c_read_pwr_levels(radio_h, response);

    if (count != 4)
        return false;

    memcpy(&vfwd, response, 2);
    memcpy(&vref, response + 2, 2);

    radio_h->fwd_power = vfwd;
    radio_h->ref_power = vref;

    return true;
}

// returns power * 10
uint32_t get_fwd_power(radio *radio_h)
{
    // 40 should be we are using 40W as end of scale
    uint32_t fwdvoltage = (radio_h->fwd_power * 40) / 100; // 100 = bridge_compensation
    uint32_t fwdpower = (fwdvoltage * fwdvoltage) / 400;

    return fwdpower;
}

uint32_t get_ref_power(radio *radio_h)
{
    uint32_t refvoltage = (radio_h->ref_power * 40) / radio_h->bridge_compensation;
    uint32_t refpower = (refvoltage * refvoltage) / 400;

    return refpower;
}

uint32_t get_swr(radio *radio_h)
{
    uint32_t vfwd = radio_h->fwd_power;
    uint32_t vref = radio_h->ref_power;
    uint32_t vswr;

    if (vref == vfwd)
        vfwd++;

    if (vref > vfwd)
        vswr = 100;
    else
        vswr = (10 * (vfwd + vref)) / (vfwd - vref);

    return vswr;
}

void set_frequency(radio *radio_h, uint32_t frequency)
{
    radio_h->frequency = frequency;
    // We are setting the real frequency of the radio (in USB), without
    // the 24 kHz offset present in Ashhar implementation (just "- 24000" to replicate behavior)
    si5351bx_setfreq(2, radio_h->frequency + radio_h->bfo_frequency);
}

// disconnect all LPFs
void lpf_off(radio *radio_h)
{
    digitalWrite(LPF_A, LOW);
    digitalWrite(LPF_B, LOW);
    digitalWrite(LPF_C, LOW);
    digitalWrite(LPF_D, LOW);
}

// set appropriate LPF according to the frequency
void lpf_set(radio *radio_h)
{
    int lpf = 0;

    if (radio_h->frequency < 5500000)
        lpf = LPF_D;
    else if (radio_h->frequency < 8000000)
        lpf = LPF_C;
    else if (radio_h->frequency < 18500000)
        lpf = LPF_B;
    else if (radio_h->frequency < 30000000)
        lpf = LPF_A;

    digitalWrite(lpf, HIGH);
}

void tr_switch(radio *radio_h, bool txrx_state)
{
    if (txrx_state == radio_h->txrx_state)
        return;

    if (txrx_state == IN_TX)
    {
        radio_h->txrx_state = IN_TX;

        lpf_off(radio_h); delay(2);
        digitalWrite(TX_LINE, HIGH); delay(2);
        lpf_set(radio_h);
    }
    else
    {
        radio_h->txrx_state = IN_RX;

        lpf_off(radio_h); delay(2);
        digitalWrite(TX_LINE, LOW); delay(2);
        // Optionally select receive LPF here if different policy is desired
    }
}