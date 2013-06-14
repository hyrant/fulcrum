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

#ifndef _TIDATA_H
#define _TIDATA_H

#include <stdint.h>

extern const uint16_t Data_version;

extern const uint8_t Data_wlanDriver[];
extern const uint16_t Data_wlanDriver_Length;
extern const uint8_t Data_firmware[];
extern const uint16_t Data_firmware_Length;

extern const uint8_t Data_wlanDriverP1P5[];
extern const uint16_t Data_wlanDriverP1P5_Length;
extern const uint8_t Data_firmwareP1P5[];
extern const uint16_t Data_firmwareP1P5_Length;

extern const uint8_t Data_cRMdefaultParams[];
extern const uint16_t Data_FATEntriesAddress[];
extern const uint16_t Data_FATEntriesLength[];

#endif
