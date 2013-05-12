/*
 * Copyright (C) 2013 Derek Hageman <Derek.C.Hageman@gmail.com> 
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/* Derived from the libopencm3 linking scripts 
 * (C) Uwe Hermann <uwe@hermann-uwe.de> 
 */
 

/* Define memory regions. */
MEMORY
{
	rom (rx) : ORIGIN = 0x08006400, LENGTH = 103K
	ram (rwx) : ORIGIN = 0x20000400, LENGTH = 7K - 64 /* Main thread return */
}

/* Include the common ld script. */
INCLUDE libopencm3_stm32f1.ld

INCLUDE ../api.ld
