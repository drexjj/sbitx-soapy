/* sBitx core
 * Copyright (C) 2023 Rhizomatica
 * Author: Rafael Diniz <rafael@riseup.net>
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 *
 */

#include <stdbool.h>
#include <wiringPi.h>

#include "sbitx_gpio.h"

// global radio handle pointer used for the callback functions
radio *radio_gpio_h;

// for now this initializes the GPIO and also initializes the structures for
// encoder/knobs for easy reading by application
void gpio_init(radio *radio_h)
{
    // we need the radio handler available for the callbacks.
    radio_gpio_h = radio_h;

    // GPIO SETUP
    wiringPiSetup();

    // TODO: change the number to the #defines for easier reading
    char pins[13] = {0, 2, 3, 6, 7, 10, 11, 12, 13, 14, 21, 25, 27};
    for (int i = 0; i < 13; i++)
    {
        pinMode(pins[i], INPUT);
        pullUpDnControl(pins[i], PUD_UP);
    }

    //setup the LPFs and TX lines to initial state
    pinMode(TX_LINE, OUTPUT);
    pinMode(TX_POWER, OUTPUT);
    pinMode(LPF_A, OUTPUT);
    pinMode(LPF_B, OUTPUT);
    pinMode(LPF_C, OUTPUT);
    pinMode(LPF_D, OUTPUT);
    digitalWrite(LPF_A, LOW);
    digitalWrite(LPF_B, LOW);
    digitalWrite(LPF_C, LOW);
    digitalWrite(LPF_D, LOW);
    digitalWrite(TX_LINE, LOW);
    digitalWrite(TX_POWER, LOW);

   

}

