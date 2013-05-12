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

#ifndef _EEPROM_H
#define _EEPROM_H

#include <stdint.h>

/**
 * Read data from the EEPROM.  
 * 
 * @param address   the start address to begin reading at
 * @param data      the output data buffer
 * @param length    the number of bytes to read
 */
void eeprom_read(uint16_t address, void *data, uint16_t length);

/**
 * Write data to the EEPROM.
 * 
 * @param address   the address to start writing at
 * @param data      the data to write
 * @param length    the number of bytes to write
 */
void eeprom_write(uint16_t address, const void *data, uint16_t length);

/**
 * Zero data in the EEPROM.
 * 
 * @param address   the address to start writing at
 * @param length    the number of bytes to write
 */
void eeprom_zero(uint16_t address, uint16_t length);

#endif
