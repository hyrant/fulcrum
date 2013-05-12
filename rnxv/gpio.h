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

#ifndef GPIO_H
#define GPIO_H

#include <stdint.h>
#include <stdbool.h>

/**
 * Set the GPIO mode.
 * 
 * @param mask      the mask of pins to change
 * @param output    true to set as an output
 */
void GPIO_setMode(uint16_t mask, bool output);

/**
 * Set the actual output values.
 * 
 * @param mask      the mask to change
 * @param bits      a bit set to raise
 */
void GPIO_setOutput(uint16_t mask, uint16_t bits);

/**
 * Get the name of a specific pin index.
 * 
 * @param index     the index
 */
const char *GPIO_getName(int index);

/**
 * Initialize GPIO.
 */
void GPIO_init(void);

#endif
