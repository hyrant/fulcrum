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

#ifndef ADC_H
#define ADC_H

#include <stdint.h>

/**
 * Get the values from the ADC.
 * 
 * @param temperature   the output temperature sensor value
 * @param reference     the output reference voltage
 */
void ADC_get(uint16_t *temperature, uint16_t *reference);

/**
 * Initialize the ADC subsystem.
 */
void ADC_init(void);

#endif
