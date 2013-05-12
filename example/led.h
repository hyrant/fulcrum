/*
 * Copyright (C) 2013 Derek Hageman <Derek.C.Hageman@gmail.com> 
 * 
 * This file is part of Fulcrum.
 *
 * Fulcrum is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.

 * Fulcrum is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 * 
 * You should have received a copy of the GNU Affero General Public License
 * along with Fulcrum.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef LED_H
#define LED_H

#include <stdint.h>

/**
 * The mode of operation for the LED blinking.
 */
typedef enum {
    /** Just turn on and off after the specified delays. */
    LED_BLINK,
    /** Fade on and off (sawtooth) over the delays. */
    LED_FADE,
} LED_MODE;

/**
 * Set the LED operation mode.
 * 
 * @param mode      the mode
 * @param up        the time (in milliseconds) that the LED is on or fading on
 * @param down      the time that the LED is off for fading off
 */
void LED_setMode(LED_MODE mode, uint16_t up, uint16_t down);

/**
 * Get the current LED operation mode.
 * 
 * @param mode      the mode
 * @param up        the time (in milliseconds) that the LED is on or fading on
 * @param down      the time that the LED is off for fading off
 */
void LED_getMode(LED_MODE *mode, uint16_t *up, uint16_t *down);

/**
 * Initialize the LED subsystem.
 */
void LED_init(void);

#endif
