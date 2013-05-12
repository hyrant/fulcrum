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

#ifndef _TRANSPORT_H
#define _TRANSPORT_H

#include <stdint.h>
#include <stdbool.h>

#include "hci.h"

#ifdef ENABLE_CC3000_PATCHING
/**
 * The startup patching mode.
 */
typedef enum {
    /** Use the patches in the CC3000 EEPROM. */
    Transport_DefaultPatches,
    /** Request patches from the host. */
    Transport_HostPatches,
    /** Start up with no patches. */
    Transport_NoPatches,
} TransportPatchMode;
#endif


/**
 * A buffer in a transmit linked list.
 */
typedef struct TransmitBuffer_s {
    /** The next buffer or NULL if this is the final one. */
    struct TransmitBuffer_s *next;
    
    /** The data to send. */
    const void *buffer;
    /** The length of the data to send. */
    uint16_t length;
} TransmitBuffer;

/**
 * A buffer in a receive linked list.
 */
typedef struct ReceiveBuffer_s {
    /** The next buffer or NULL if this if the final one. */
    struct ReceiveBuffer_s *next;
    
    /** The buffer to place data into. */
    void *buffer;
    /** Initially this is the maximum size of the buffer.  After reading the
     * actual amount of data read is placed here. */
    uint16_t length;
} ReceiveBuffer;

/**
 * The handler for the DMA read channel.
 */
void Transport_irqRxDMA(void);
/** 
 * The handler for the external interrupt (falling edge) on the CC3000 
 * IRQ line.
 */
void Transport_irqEXTI(void);

/**
 * Start the initialization sequence.
 * 
 * @param mode  the patch mode
 */
#ifdef ENABLE_CC3000_PATCHING
void Transport_initPatched(TransportPatchMode mode);
#else
void Transport_initPatched(void);
#endif

/**
 * Start the initialization sequence in the default mode.  This is
 * normally a weak alias to Transport_initPatched(Transport_DefaultPatches).
 */
void Transport_init(void);

/**
 * Perform periodic processing for the transport system.  This should be called
 * at regular intervals and will generally return immediately.
 */
void Transport_process(void);

/**
 * This is called from Transport_process() while interrupts are disabled
 * if the transport layer is about to re-initialize.  By default
 * this is a weak alias that does nothing.
 */
void Transport_reinitializing(void);

/**
 * This is called from Transport_process() when any TCP socket has had the
 * remote end closed.  By default this is a weak alias that does nothing.
 */
void Transport_closingTCP(void);

/**
 * Test if the transport layer is ready (initialization completed).
 * 
 * @return true if the transport layer is ready
 */
bool Transport_ready(void);

#ifdef ENABLE_CC3000_PATCHING
/** The type of patch requested */
typedef enum {
    Transport_PatchDriver = HCI_EVENT_PATCHES_DRV_REQ,
    Transport_PatchFirmware = HCI_EVENT_PATCHES_FW_REQ,
    Transport_PatchBootloader = HCI_EVENT_PATCHES_BOOTLOAD_REQ,
} TransportPatchRequest;
/**
 * Called during a host patch request sequence to get the patch data for
 * the given stage.  This is normally a weak alias to a stub handler that
 * returns nothing.
 * 
 * @param patch         the patch stage
 * @param patchData     the output patch data pointer or NULL for none
 * @param patchLength   the output patch length
 */
void Transport_patch(TransportPatchRequest patch, 
                     const void **patchData, uint16_t *patchLength);
#endif
                     
/**
 * Called from interrupt context when the simple configuration procedure
 * has completed.  This is normally a weak alias to a stub handler that does
 * nothing.
 */
void Transport_simpleConfigDone(void);

/**
 * Called from interrupt context when the WLAN connected state changes.
 * This is normally a weak alias to a stub handler that does nothing.
 * 
 * @param connected     true if the device is currently connected
 */
void Transport_connected(bool connected);

/**
 * Called from interrupt context when any data is received from the network.
 * This is normally a weak alias to a stub handler that does nothing.
 */
void Transport_dataReceived(void);

/**
 * Called from interrupt context when a ping report is received.
 * This is normally a weak alias to a stub handler that does nothing.
 * 
 * @param packetsSent       the number of packets sent
 * @param packetsReceived   the number of packets received
 * @param minRTT            the minimum round trip (in msec?)
 * @param maxRTT            the maximum round trip (in msec?)
 * @param avgRTT            the average round trip (in msec?)
 */
void Transport_pingReport(uint32_t packetsSent, uint32_t packetsReceived,
                          uint32_t minRTT, uint32_t maxRTT, uint32_t avgRTT);
                          
/**
 * Called from interrupt context when a DHCP status message is received.
 * This is normally a weak alias to a stub handler that does nothing.
 * 
 * @param ip        the current IP address in network byte order
 * @param subnet    the current subnet mask in network byte order
 * @param gateway   the current gateway IP address in network byte order
 * @param dhcp      the current DHCP server address in network byte order
 * @param dns       the current DNS server address in network byte order
 */
void Transport_dhcp(uint32_t ip, uint32_t subnet, uint32_t gateway,
                    uint32_t dhcp, uint32_t dns);

/**
 * Issue a simple command to the CC3000.
 * 
 * @param opcode        the opcode of the command
 * @param tx            the transmit parameters of the command or NULL for none
 * @param rx            the buffers to place any data returned by the command or NULL to discard
 * @return              the command execution status
 */
int Transport_command(uint16_t opcode, TransmitBuffer *tx, ReceiveBuffer *rx);

/**
 * Issue a data write and wait for an event response to that data write.
 * 
 * @param opcode        the data opcode
 * @param arguments     the arguments to the write stream or NULL for none
 * @param data          the data to send or NULL for none
 * @param responseOpcode    the opcode of the response to wait for
 * @param response      the buffer to place the data returned into or NULL to discard
 * @return              the response status
 */
int Transport_write(uint8_t opcode, 
                     TransmitBuffer *arguments, TransmitBuffer *data,
                     uint16_t responseOpcode, ReceiveBuffer *response);

/**
 * Send something to the data channel of the CC3000.
 * 
 * @param opcode        the send opcode
 * @param arguments     the arguments to the send stream or NULL for none
 * @param data          the data to send or NULL for none
 */
void Transport_send(uint8_t opcode, 
                    TransmitBuffer *arguments, TransmitBuffer *data);

/**
 * Issue a command then wait for a data packet that contains the response
 * to that command.
 * 
 * @param opcode        the command opcode
 * @param command       the command additional argument data to send or NULL for none
 * @param arguments     the output arguments from the data read operation or NULL to discard
 * @param data          the output data from the data read operation or NULL to discard
 * @return              the command status code
 */
int Transport_read(uint16_t opcode, TransmitBuffer *command, 
                   ReceiveBuffer *arguments, ReceiveBuffer *data);
                   
/**
 * Issue a (network) receive command then wait for the data response from that.
 * This invokes special handling for the number of bytes returned by the
 * command.
 * 
 * @param opcode        the command opcode
 * @param command       the command additional argument data to send or NULL for none
 * @param arguments     the output arguments from the data read operation or NULL to discard
 * @param data          the output data from the data read operation or NULL to discard
 * @return              the number of bytes the command answered with
 */
int Transport_recv(uint16_t opcode, TransmitBuffer *command, 
                   ReceiveBuffer *arguments, ReceiveBuffer *data);
                   
/**
 * Flush the transmission layer.  This attempts to ensure that the device
 * has transmitted all data at the time of the call before returning.  The 
 * flush is complete when either all buffers have been released, a full cycle 
 * of buffers has been transmitted (meaning no possible buffers from before
 * the call are still in use) or when the timeout has elapsed.
 * 
 * @param timeout       the maximum time in ticks to wait
 */
void Transport_flush(uint32_t timeout);

/**
 * Wait for the transport layer to become idle.  This disables interrupts
 * and leaves them disabled at the point that it returns.
 */
void Transport_waitIdle(void);
       
/**
 * Set to the current systick whenever a keel alive event is received.
 */
extern volatile uint32_t lastTransportKeepAlive;

#endif
