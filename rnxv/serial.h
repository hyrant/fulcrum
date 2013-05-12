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

#ifndef SERIAL_H
#define SERIAL_H

#include <stdbool.h>

#include <libopencm3/stm32/usart.h>

/**
 * Initialize the serial system.
 */
void Serial_init(void);

/**
 * The main execution function for the serial system.
 */
void Serial_run(void);

/**
 * Get the port the TCP server is listening on.
 * 
 * @return  the TCP port
 */
int Serial_getTCPPort(void);

/**
 * Set the TCP port the server is listening on.
 * 
 * @param port  the new port
 */
void Serial_setTCPPort(int port);

/**
 * Serial port parity selection.
 */
typedef enum {
    Parity_None = USART_PARITY_NONE,
    Parity_Even = USART_PARITY_EVEN,
    Parity_Odd = USART_PARITY_ODD,
} SerialParity;

/**
 * Get the serial port settings.
 * 
 * @param baud          the baud rate
 * @param parity        the parity setting
 * @param dataBits      the number of data bits
 * @param stopBits      the number of stop bits
 * @param flowControl   the flow control setting
 */
void Serial_getSettings(int *baud, SerialParity *parity, int *dataBits,
                        int *stopBits, bool *flowControl);
                        
/**
 * Set the baud rate.
 * 
 * @param baud          the new baud rate
 */
void Serial_setBaud(int baud);

/**
 * Set the parity.
 * 
 * @param parity        the parity
 */
void Serial_setParity(SerialParity parity);

/**
 * Set the number of data bits.
 * 
 * @param dataBits      the data bits (7 or 8)
 */
void Serial_setDataBits(int dataBits);

/**
 * Set the number of stop bits.
 * 
 * @param stopBits      the stop bits (1 or 2)
 */
void Serial_setStopBits(int stopBits);

/**
 * Enable or disable RTS and CTS flow control.
 * 
 * @param flowCOntrol   enable RTS and CTS
 */
void Serial_setFlowControl(bool flowControl);

/**
 * Save the current settings to the flash.
 */
void Serial_saveSettings(void);

#endif
