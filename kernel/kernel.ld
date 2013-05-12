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
    /* Write protect has a 4 page granularity */
	rom (rx) : ORIGIN = 0x08000000, LENGTH = 24k
	ram (rwx) : ORIGIN = 0x20000000, LENGTH = 1K
}

EXTERN (vector_table)
ENTRY(run)

/* Define sections. */
SECTIONS
{
	.text : {
		*(.vectors)
		*(.text*)
		. = ALIGN(4);
		*(.rodata*)
		. = ALIGN(4);
	} >rom

	. = ALIGN(4);
	_etext = .;

	.data : {
		_data = .;
		*(.data*)	/* Read-write initialized data */
		. = ALIGN(4);
		_edata = .;
	} >ram AT >rom
	_data_loadaddr = LOADADDR(.data);
    
    INCLUDE exports.ld

	.bss : {
		*(.bss*)	/* Read-write zero initialized data */
		*(COMMON)
		. = ALIGN(4);
		_ebss = .;
	} >ram

	. = ALIGN(4);
	end = .;
}

PROVIDE(_ram = ORIGIN(ram));
PROVIDE(_stack = ORIGIN(ram) + LENGTH(ram));
PROVIDE(_kernelbegin = ORIGIN(rom));
PROVIDE(_config = ORIGIN(rom) + LENGTH(rom));
PROVIDE(_userbegin = ORIGIN(rom) + LENGTH(rom) + 1k);
