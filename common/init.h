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

#ifndef _INIT_H
#define _INIT_H

/**
 * Initialize all peripherals.
 * 
 * @return  true if the system should do a settings reset
 */
bool Kernel_initializePeripherals(void);

/**
 * Initialize the UART for setup mode.
 */
void Kernel_initializeUART(void);

/**
 * Disable the UART when exiting setup mode.
 */
void Kernel_disableUART(void);

/**
 * Disable all watchdogs.
 */
void Kernel_disableWatchdog(void);

/**
 * Enable all watchdogs.
 */
void Kernel_enableWatchdog(void);

/**
 * Refresh all watchdogs.
 */
void Kernel_refreshWatchdog(void);

/**
 * Disable all externally usable peripherals.  This is used after a soft
 * fault or user code halt so ensure we don't reference parts of the system
 * we should not.
 */
void Kernel_disableUserPeripherals(void);

#endif
