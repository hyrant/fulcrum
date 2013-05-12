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

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/f1/dma.h>
#include <libopencm3/stm32/f1/gpio.h>

#include "kernel.h"
#include "synchronization.h"
#include "transport.h"
#include "hci.h"

volatile uint32_t lastTransportKeepAlive;

#pragma pack(push,1)
typedef struct {
    uint8_t handle;
    uint8_t blocking;
    uint16_t released;
} ReleaseBuffersElement;
static union {
    struct {
        uint8_t spiRead;
        uint8_t busy[2];
        uint8_t lenHI;
        uint8_t lenLO;
        uint8_t type;
        union {
            struct {
                uint8_t unknown;
                uint8_t argLength;
                uint16_t totalLength;
            } data;
            struct {
                uint16_t opcode;
                uint8_t length;
                int8_t status;
            } event;
        } body;
    } readHeader;
    struct {
        uint8_t spiWrite;
        uint8_t spiLenHI;
        uint8_t spiLenLO;
        uint8_t busy[2];
        uint8_t type;
        union {
            struct {
                uint16_t opcode;
                uint8_t length;
            } command;
            struct {
                uint8_t opcode;
                uint8_t argLength;
                uint16_t totalLength;
            } data;
            #ifdef ENABLE_CC3000_PATCHING
            struct {
                uint8_t patchType;
                uint16_t patchLength;
                uint16_t packetLength;
            } patch;
            #endif
        } body;
    } writeHeader;
    
    struct {
        uint8_t numberOfBuffers;
        uint16_t bufferSize;
    } readBuffers;
    #ifdef ENABLE_CC3000_PATCHING
    struct {
        uint8_t type;
    } patchRequest;
    #endif
    struct {
        uint32_t packetsSent;
        uint32_t packetsReceived;
        uint32_t minRTT;
        uint32_t maxRTT;
        uint32_t avgRTT;
    } pingReport;
    struct {
        uint32_t ip;
        uint32_t subnet;
        uint32_t gateway;
        uint32_t dhcp;
        uint32_t dns;
    } dhcp;
    struct {
        uint32_t fd;
        int32_t status;
        uint32_t flags;
    } dataResponse;
    struct {        
        uint16_t numberOfBuffers;
        ReleaseBuffersElement elements[3];
    } releaseBuffersFirst;
    struct {        
        ReleaseBuffersElement elements[4];
    } releaseBuffers;

    /* Minimum size for unsolicited responses */
    uint8_t buffer[20];
} scratchData;
#pragma pack(pop)
static uint8_t discardByte[1];

/* Note that Rx DMA is used as the final transaction even when transmitting,
 * since the transfer won't complete until the SPI is no longer busy.  Tx
 * DMA completes as soon as the last byte is the the send buffer for the SPI
 * which is before we can actually de-assert CS.  We could use Tx DMA for the
 * middle writes, but that introduces the complexity of figuring out if
 * we need to read one extra byte (or busy wait until the current one is
 * sent).  So we just accept the small gaps between interrupt service and
 * DMA reload (not like we really care about speed anyway). */

typedef enum {
    /* The interface has faulted somehow.  This state is entered when something
     * unexpected has happened.  The response is to re-initialize the interface
     * the next time control returns to the processing loop (which is
     * outside of any interrupt handlers).
     * Processing loop -> Restart interface initialization */
    Faulted = 0,
    
    /* Initialization is spinning up.  All access is blocked until the
     * thread doing the spin up completes this. */
    Initialize_Spinup,
    
    /* Initialization starting.  The initial start command has been issued
     * and we're waiting for the response to it.
     * IRQ -> Issue header read
     * Rx complete with start response -> Complete start
     * Rx complete with patch request -> Complete patch read
     * Rx complete -> Discard data */
    Initialize_Start,
    
    /* Start response received.  A response to the initial start command was
     * received, so we're finishing discarding the trailing data.
     * Rx complete -> Wait for ready */
    Initialize_StartCompleting,
    
    /* Initialization received an out of sequence response; discard.  The
     * initialization didn't get the start response as the first response,
     * so discard it.
     * Rx complete -> Wait for another response */
    Initialize_StartDiscard,
    
    #ifdef ENABLE_CC3000_PATCHING
    /* A patch request was received, currently completing the read.  This
     * is entered when a patch request was received and we need to read the
     * patch request data (the patch type)
     * Rx complete -> Patch request wait */
    Initialize_PatchRequestCompleting,
    
    /* A patch request is pending, we're currently in a wait state.  This is
     * entered after the device has de-asserted its IRQ and we're currently
     * waiting a little bit before asserting CS again.
     * Processing -> Assert CS, if IRQ already asserted then start send, 
     *      otherwise wait
     * IRQ -> Start send */
    Initialize_PatchRequestWait,
    
    /* Sending the header for a patch.  This is entered if a patch is requested
     * in initialization.  The header is currently being transmitted.
     * Rx complete -> Send patch data */
    Initialize_StartSendingPatchHeader,
    
    /* Sending the data for a patch.  This follows the patch header send.
     * Rx complete -> Return to start wait state */
    Initialize_StartSendingPatchData,
    #endif
    
    
    /* Device initialized so we're now waiting for it be be ready.  This
     * state waits until we return to processing then begins transmission
     * of the read buffers command.
     * Processing -> Assert CS, if the IRQ is asserted then start send,
     *      otherwise wait for it
     * IRQ -> Start send */
    Initialize_ReadBuffersSendWait,
    
    /* Start response received, currently sending read buffers command.  The
     * start response has been received, and the DMA for the read buffers
     * command is in flight.
     * Rx complete -> Disable transceiver and wait
     * IRQ -> Begin header read */
    Initialize_ReadBuffersSend,
    
    /* Read buffers command pending.  The read buffers command has been issued
     * and we're waiting for the response to it.
     * Rx complete & read buffers response -> Read buffers data
     * Rx complete & !read buffers response -> Discard data */
    Initialize_ReadBuffersHeader,
    
    /* Read buffers response received, DMA to read the data currently in flight.
     * The read buffers command response header was received, so we're currently
     * reading the data for the command.
     * Rx complete -> Process read buffers and go to Idle */
    Initialize_ReadBuffersData,
    
    /* Received an out of sequence response, so discard it.  The response to
     * the read buffers command does not match, so discard this response.
     * Rx complete -> Wait for response */
    Initialize_ReadBuffersDiscard,
    
    
    
    /* The system is idle.  This is the state when there is no active DMA
     * transaction and no other work being done.
     * IRQ -> Unsolicited read header
     * Outside APIs can claim to initial write states */
    Idle,
    
    
    /* Currently reading the header of an unsolicited event.  A read request
     * was received while the system was in idle, so the header is currently
     * being read.
     * Rx complete and no more data -> Process and back to idle
     * Rx complete with more data -> Read more data */
    Unsolicited_ReadHeader,
    /* Currently reading more data for an unsolicited event.  This is entered
     * when the unsolicited event has had the header read but there is still
     * more data to be read.
     * Rx complete with more data -> Continue reading
     * Rx complete with no more data -> Process and back to idle */
    Unsolicited_Read,
    /* An unsolicited header indicated that we just need to discard data.
     * Rx complete -> Idle */
    Unsolicited_Discard,
    
    
    /* The header of a command is being written.  This is entered by the
     * external API by exclusively taking control of CS while the IRQ is
     * not asserted.  Once that is done the scratch buffer is filled with the
     * header and prepared to be ready to write once the IRQ is asserted.
     * IRQ -> Begin write of scratch buffer
     * Rx complete -> Write body */
    Command_WriteHeader,
    
    /* The body of a command is being written.  The data part of the command
     * is currently being transferred.  This is entered after the header
     * is written.  Once written we enter the wait response state.
     * Rx complete with more data -> Send next block
     * Rx complete and no more data -> Wait for response */
    Command_WriteBody,
    
    /* Currently waiting for a response to the command.  We're currently waiting
     * for a response to the last issued command or are in the process of
     * reading a header to one.
     * IRQ -> Read header
     * Rx complete with response header -> Read body
     * Rx complete with non-response header -> Handle unsolicited or discard */
    Command_WaitResponse,
    
    /* Currently reading the command response.  We're currently handling DMA
     * completions to read the data for the command response.
     * Rx complete -> Complete command then return to Idle */
    Command_ReadResponseBody,
    
    /* Currently discarding spurious data received instead of a command 
     * response. 
     * Rx complete -> Wait for read IRQ */
    Command_DiscardSpuriousResponse,
    
    /* Currently handling an unsolicited response that came in instead of
     * the command response we were expecting.
     * Rx complete with more data -> Continue reading
     * Rx complete with no more data -> Process and back to command wait */
    Command_UnsolicitedResponse,
    
    
    /* The header of a data send request is being written.  This is entered by 
     * the external API by exclusively taking control of CS while the IRQ is
     * not asserted.  Once that is done the scratch buffer is filled with the
     * header and prepared to be ready to write once the IRQ is asserted.  This
     * will consume a buffer that will be later released by an unsolicited
     * event.
     * IRQ -> Begin write of scratch buffer
     * Rx complete -> Write body */
    WriteData_WriteHeader,
    
    /* The body of a send data request is being written.  The buffers of
     * the request are being transversed in DMA requestes.
     * Rx complete and more data -> Send next buffer
     * Rx complete and data end, no response -> Mark complete and idle
     * Rx complete and data end, with response -> Wait response */
    WriteData_WriteBody,
    
    /* The data has been written and we're waiting for a response.  When we
     * get an IRQ we start reading a header then decide what to do with
     * the incoming packet.
     * IRQ -> Read header
     * Rx complete with response header -> Read body
     * Rx complete with non-response header -> Handle unsolicited or discard */
    WriteData_WaitResponse,
    
    /* Currently handling an unsolicited response that came in instead of
     * the command response we were expecting.
     * Rx complete with more data -> Continue reading
     * Rx complete with no more data -> Process and back to response wait */
    WriteData_UnsolicitedResponse,
    
    /* Currently reading the response body to the data send.
     * Rx complete -> Complete command then return to Idle */
    WriteData_ReadResponseBody,
    
    /* Currently discarding spurious data received instead of a command 
     * response. 
     * Rx complete -> Wait for read IRQ */
    WriteData_DiscardSpuriousResponse,
    
    
    /* The header of the initiator command for data receive is being written.
     * This is entered by the external API by exclusively taking control of CS 
     * while the IRQ is not asserted.  Once that is done the scratch buffer is 
     * filled with the header and prepared to be ready to write once the IRQ is
     * asserted.
     * IRQ -> Begin write of scratch buffer
     * Rx complete -> Write body */
    ReadData_WriteCommandHeader,
    
    /* The body of a command is being written.  The data part of the command
     * is currently being transferred.  This is entered after the header
     * is written.  Once written we enter the wait response state.
     * Rx complete with more data -> Send next block
     * Rx complete and no more data -> Wait for response */
    ReadData_WriteCommandBody,
    
    /* Currently waiting for a response to the data read command.  This is
     * entered by IRQ assertion after the command write.  A read complete means
     * the header has been finished.
     * reading a header to one.
     * IRQ -> Read header
     * Rx complete with correct response -> Read body
     * Rx complete with non-response header -> Handle unsolicited or discard */
    ReadData_WaitCommandResponse,
    
    /* Currently reading the command response.  We're currently handling DMA
     * completions to read the data for the command response.
     * Rx complete -> Complete verify command and wait for data */
    ReadData_ReadCommandResponseBody,
    
    /* Currently discarding spurious data received instead of a command 
     * response. 
     * Rx complete -> Wait for read IRQ */
    ReadData_DiscardSpuriousCommandResponse,
    
    /* Currently handling an unsolicited response that came in instead of
     * the command response we were expecting.
     * Rx complete with more data -> Continue reading
     * Rx complete with no more data -> Process and back to command wait */
    ReadData_UnsolicitedCommandResponse,
    
    /* Currently waiting for a data response.  This is entered after the
     * command has been responded to and indicated that there was data
     * to receive.
     * IRQ -> Read header
     * Rx complete with correct response -> Read body
     * Rx complete with non-response header -> Handle unsolicited or discard */
    ReadData_WaitDataResponse,
    
    /* Currently reading the body of the data.  A data response has been 
     * received and is currently being used to fill the response buffers.
     * Rx complete with more to read -> Launch next DMA
     * Rx complete and no more -> Mark complete and idle */
    ReadData_ReadDataBody,
    
    /* Currently discarding spurious data received instead of the expected
     * data packet.
     * Rx complete -> Wait for read IRQ */
    ReadData_DiscardSpuriousDataResponse,
    
    /* Currently handling an unsolicited response that came in instead of
     * the data response we were expecting.
     * Rx complete with more data -> Continue reading
     * Rx complete with no more data -> Process and back to command wait */
    ReadData_UnsolicitedDataResponse,

} SystemState;
static SystemState state;

static struct {
    uint16_t opcode;
    union {
        struct {
            uint16_t remainingHandles;
        } bufferRelease;
    } data;
} unsolicited;

typedef struct {    
    uint16_t opcode;
    int8_t status;
    
    TransmitBuffer *tx;
    uint8_t txPadding;
    
    ReceiveBuffer *rx;
    uint16_t bufferRemaining;
    
    volatile bool completed;
} CommandProcessing;

typedef struct {
    TransmitBuffer *arguments;
    TransmitBuffer *data;
    uint8_t txPadding;
    
    uint16_t responseOpcode;
    ReceiveBuffer *response;
    uint16_t responseRemaining;
    
    int8_t status;
    volatile bool completed;
} WriteProcessing;

typedef struct {
    uint16_t opcode;
    int32_t result;
    
    TransmitBuffer *command;
    uint8_t txPadding;    
    bool recvProcessing;
    
    ReceiveBuffer *arguments;
    uint16_t argumentsRemaining;
    ReceiveBuffer *data;
    uint16_t dataRemaining;
    
    volatile bool completed;
} ReadProcessing;

static union {
    CommandProcessing *command;
    WriteProcessing *write;
    ReadProcessing *read;
    #ifdef ENABLE_CC3000_PATCHING
    struct {
        TransportPatchRequest type;
    } patch;
    #endif
} processing;
static uint16_t transferLengthRemaining;

static Semaphore freeBuffers;
static uint8_t totalBuffers;
static volatile uint16_t bufferCounter;

enum {
    Callback_TCP_Closed = 0x01,
};
static uint8_t callbackNotification;

static uint32_t stateBeginTime;
static void stateTransition( SystemState newstate )
{
    state = newstate;
    stateBeginTime = systick;
}


static void initializeDelay(void)
{
    for (int i=0; i<0x400; i++)
        asm volatile ( "NOP" );
}

static void deassertCS(void)
{
    GPIO_BSRR(GPIOB) = GPIO9;
}
static void assertCS(void)
{
    GPIO_BSRR(GPIOB) = GPIO9 << 16;
}

static bool isIRQAsserted(void)
{
    return (GPIO_IDR(GPIOA) & GPIO15) == 0;
}
static void clearIRQExti(void)
{
    /* Since we're on the 10-15 IRQ, we clear all of those to make
     * sure we can re-enter the interrupt handler if one isn't
     * cleared by the user code.  Without doing this we can get stuck
     * with the handler never being called again if it's failed to
     * be cleared. */
    EXTI_PR = EXTI15|EXTI14|EXTI13|EXTI12|EXTI11|EXTI10;
}
static void disableIRQExti(void)
{
    EXTI_FTSR &= ~EXTI15;
}
static void enableIRQExti(void)
{
    EXTI_FTSR |= EXTI15;
    clearIRQExti();
}
static void resetPulse(void)
{
    GPIO_BSRR(GPIOA) = GPIO14 << 16;
    for (int i=0; i<0x2000; i++)
        asm volatile ( "NOP" );
    GPIO_BSRR(GPIOA) = GPIO14;
    initializeDelay();
}

static void beginReadHeader(void);

static void waitSPIInactive(void)
{
    while (SPI_SR(SPI1) & SPI_SR_BSY) { }
}

static void transferComplete(void)
{
    waitSPIInactive();
    clearIRQExti();
    deassertCS();
}

static void blockingWrite( uint8_t data )
{
	SPI_DR(SPI1) = data;
    while (!(SPI_SR(SPI1) & SPI_SR_TXE)) { }
}

#define DMA_WRITE_CCR_BASE      (DMA_CCR_PL_LOW | DMA_CCR_MSIZE_8BIT | DMA_CCR_PSIZE_8BIT | DMA_CCR_DIR)
#define DMA_READ_CCR_BASE       (DMA_CCR_PL_MEDIUM | DMA_CCR_MSIZE_8BIT | DMA_CCR_PSIZE_8BIT)

static void stopActiveTransaction(void)
{
    waitSPIInactive();
    DMA_CCR(DMA1, DMA_CHANNEL2) = 0;
    DMA_CCR(DMA1, DMA_CHANNEL3) = 0;
    DMA1_IFCR = DMA_ISR_GIF2;
}

#ifdef ENABLE_CC3000_PATCHING
void Transport_initPatched(TransportPatchMode mode)
#else
void Transport_initPatched(void)
#endif
{
    disableInterrupts();    
    if (state == Initialize_Spinup) {
        enableInterrupts();
        return;
    }
    
    state = Initialize_Spinup;
    lastTransportKeepAlive = systick;
    semaphore_init(&freeBuffers, 0);
    callbackNotification = 0;
    
    deassertCS();
    
    stopActiveTransaction();
    disableIRQExti();
    
    enableInterrupts();
    
    resetPulse();
    
    /* Wait for it to lower IRQ to indicate ready (53 ms supposedly) */
    while (!isIRQAsserted()) {
        Thread_yield();
    }

    /* Minimum 7ms delay before the initial command */
    {
        uint32_t start = systick;
        while (Kernel_elapsed(start) <= ((7*TICK_RATE)/1000 + 1)) {
            Thread_yield();
        }
    }
    
    disableInterrupts();
        
    assertCS();
    initializeDelay();
    
    /* Send the initial startup command with the special handling the first
     * first command requires. */
     
    blockingWrite(SPI_WRITE);
    /* Length */
    blockingWrite(0);
    blockingWrite(5);
    /* Busy */
    blockingWrite(0);
    /* Startup wait delay as required */
    waitSPIInactive();
    initializeDelay();
    /* Busy */
    blockingWrite(0);
    
    /* Command payload */
    blockingWrite(HCI_TYPE_CMND);
    blockingWrite(HCI_CMND_SIMPLE_LINK_START & 0xFF);
    blockingWrite(HCI_CMND_SIMPLE_LINK_START >> 8);
    blockingWrite(1);
    #ifdef ENABLE_CC3000_PATCHING
    switch (mode) {
    case Transport_DefaultPatches:
        blockingWrite(SL_PATCHES_REQUEST_DEFAULT);
        break;
    case Transport_HostPatches:
        blockingWrite(SL_PATCHES_REQUEST_FORCE_HOST);
        break;
    case Transport_NoPatches:
        blockingWrite(SL_PATCHES_REQUEST_FORCE_NONE);
        break;
    }
    #else
    blockingWrite(SL_PATCHES_REQUEST_DEFAULT);
    #endif
    
    /* Enable only after we've finished transmitting and right before
     * we raise CS again. */
    enableIRQExti();    
    transferComplete();
    
    stateTransition(Initialize_Start);
    
    enableInterrupts();
}
void Transport_init(void) __attribute__((weak));
void Transport_init(void)
{
    #ifdef ENABLE_CC3000_PATCHING
    Transport_initPatched(Transport_DefaultPatches);
    #else
    Transport_initPatched();
    #endif
}

static void protocolFault(void)
{
    switch (state) {
    case Command_WriteHeader:
    case Command_WriteBody:
    case Command_WaitResponse:
    case Command_ReadResponseBody:
    case Command_DiscardSpuriousResponse:
    case Command_UnsolicitedResponse:
        processing.command->status = -1;
        for (ReceiveBuffer *rx = processing.command->rx; 
                rx != NULL; rx = rx->next) {
            rx->length = 0;
        }
        processing.command->completed = true;
        break;
    
    case WriteData_WriteHeader:
    case WriteData_WriteBody:
    case WriteData_WaitResponse:
    case WriteData_ReadResponseBody:
    case WriteData_UnsolicitedResponse:
    case WriteData_DiscardSpuriousResponse:
        processing.write->status = -1;
        for (ReceiveBuffer *rx = processing.write->response; 
                rx != NULL; rx = rx->next) {
            rx->length = 0;
        }
        processing.write->completed = true;
        break;
        
    case ReadData_WriteCommandHeader:
    case ReadData_WriteCommandBody:
    case ReadData_WaitCommandResponse:
    case ReadData_ReadCommandResponseBody:
    case ReadData_DiscardSpuriousCommandResponse:
    case ReadData_UnsolicitedCommandResponse:
    case ReadData_WaitDataResponse:
    case ReadData_ReadDataBody:
    case ReadData_DiscardSpuriousDataResponse:
    case ReadData_UnsolicitedDataResponse:
        processing.read->result = -1;
        for (ReceiveBuffer *rx = processing.read->arguments; 
                rx != NULL; rx = rx->next) {
            rx->length = 0;
        }
        for (ReceiveBuffer *rx = processing.read->data; 
                rx != NULL; rx = rx->next) {
            rx->length = 0;
        }
        processing.read->completed = true;
        break;
        
    default:
        break;
    }
    
    stopActiveTransaction();
    transferComplete();
    
    state = Faulted;
}

static void launchReadTransaction(void *buffer, uint16_t n)
{
    DMA_CMAR(DMA1, DMA_CHANNEL2) = (uint32_t)buffer;
    DMA_CNDTR(DMA1, DMA_CHANNEL2) = n;
    DMA_CMAR(DMA1, DMA_CHANNEL3) = (uint32_t)buffer;
    DMA_CNDTR(DMA1, DMA_CHANNEL3) = n;
    
    { uint8_t discard = SPI_DR(SPI1); (void)discard; }
    
    DMA_CCR(DMA1, DMA_CHANNEL2) = DMA_READ_CCR_BASE | DMA_CCR_MINC | 
        DMA_CCR_EN | DMA_CCR_TCIE;
    DMA_CCR(DMA1, DMA_CHANNEL3) = DMA_WRITE_CCR_BASE | DMA_CCR_MINC | 
        DMA_CCR_EN;
}

static void launchWriteTransaction(const void *buffer, uint16_t n)
{
    DMA_CMAR(DMA1, DMA_CHANNEL3) = (uint32_t)buffer;
    DMA_CNDTR(DMA1, DMA_CHANNEL3) = n;
    DMA_CMAR(DMA1, DMA_CHANNEL2) = (uint32_t)(&discardByte[0]);
    DMA_CNDTR(DMA1, DMA_CHANNEL2) = n;
    
    { uint8_t discard = SPI_DR(SPI1); (void)discard; }
        
    DMA_CCR(DMA1, DMA_CHANNEL2) = DMA_READ_CCR_BASE | 
        DMA_CCR_EN | DMA_CCR_TCIE;
    DMA_CCR(DMA1, DMA_CHANNEL3) = DMA_WRITE_CCR_BASE | DMA_CCR_MINC | 
        DMA_CCR_EN;
}

static void launchDiscardTransaction(uint16_t n)
{
    DMA_CMAR(DMA1, DMA_CHANNEL3) = (uint32_t)(&discardByte[0]);
    DMA_CNDTR(DMA1, DMA_CHANNEL3) = n;
    DMA_CMAR(DMA1, DMA_CHANNEL2) = (uint32_t)(&discardByte[0]);
    DMA_CNDTR(DMA1, DMA_CHANNEL2) = n;

    { uint8_t discard = SPI_DR(SPI1); (void)discard; }

    DMA_CCR(DMA1, DMA_CHANNEL2) = DMA_READ_CCR_BASE | 
        DMA_CCR_EN | DMA_CCR_TCIE;
    DMA_CCR(DMA1, DMA_CHANNEL3) = DMA_WRITE_CCR_BASE | 
        DMA_CCR_EN;
}

#ifdef ENABLE_CC3000_PATCHING
void Transport_patch(TransportPatchRequest patch, const void **patchData, 
                     uint16_t *patchLength) __attribute__((weak));
void Transport_patch(TransportPatchRequest patch, const void **patchData, 
                     uint16_t *patchLength)
{
    (void)patch;
    *patchData = NULL;
    *patchLength = 0;
}
#endif

void Transport_simpleConfigDone(void) __attribute__((weak));
void Transport_simpleConfigDone(void)
{
}

void Transport_connected(bool connected) __attribute__((weak));
void Transport_connected(bool connected)
{
    (void)connected;
}

void Transport_pingReport(uint32_t packetsSent, uint32_t packetsReceived,
                          uint32_t minRTT, uint32_t maxRTT, uint32_t avgRTT) 
                          __attribute__((weak));
void Transport_pingReport(uint32_t packetsSent, uint32_t packetsReceived,
                          uint32_t minRTT, uint32_t maxRTT, uint32_t avgRTT)
{
    (void)packetsSent;
    (void)packetsReceived;
    (void)minRTT;
    (void)maxRTT;
    (void)avgRTT;
}

void Transport_dhcp(uint32_t ip, uint32_t subnet, uint32_t gateway,
                    uint32_t dhcp, uint32_t dns) __attribute__((weak));
void Transport_dhcp(uint32_t ip, uint32_t subnet, uint32_t gateway,
                    uint32_t dhcp, uint32_t dns)
{
    (void)ip;
    (void)subnet;
    (void)gateway;
    (void)dhcp;
    (void)dns;
}

static void finishUnsolicitedReceive(void)
{
    transferComplete();
    
    switch (state) {
    case Command_UnsolicitedResponse:
        stateTransition(Command_WaitResponse);
        break;
    case WriteData_UnsolicitedResponse:
        stateTransition(WriteData_WaitResponse);
        break;
    case ReadData_UnsolicitedCommandResponse:
        stateTransition(ReadData_WaitCommandResponse);
        break;
    case ReadData_UnsolicitedDataResponse:
        stateTransition(ReadData_WaitDataResponse);
        break;
    default:
        stateTransition(Idle);
        break;
    }
    
    switch (unsolicited.opcode) {
    case HCI_EVNT_WLAN_UNSOL_INIT:
    case HCI_EVNT_DATA_UNSOL_FREE_BUFF:
        return;
        
    case HCI_EVNT_BSD_TCP_CLOSE_WAIT:
        /* Rather than do the sane/logical/standard thing and have recv()
         * return -1 when the remote socket is closed, the CC3000 just barfs
         * up this event.  To add insult to injury, the event has no information
         * about WHICH socket was closed, so we have to notify all of them.
         * Hurray. */
        callbackNotification |= Callback_TCP_Closed;
        return;
    
    case HCI_EVNT_WLAN_KEEPALIVE:
        lastTransportKeepAlive = systick;
        return;
        
    case HCI_EVNT_WLAN_UNSOL_CONNECT:
        Transport_connected(true);
        return;
        
    case HCI_EVNT_WLAN_UNSOL_DISCONNECT:
        Transport_connected(false);
        return;
        
    case HCI_EVNT_WLAN_ASYNC_SIMPLE_CONFIG_DONE:
        Transport_simpleConfigDone();
        return;
        
    case HCI_EVNT_WLAN_ASYNC_PING_REPORT:
        Transport_pingReport(scratchData.pingReport.packetsSent, 
                scratchData.pingReport.packetsReceived,
                scratchData.pingReport.minRTT, scratchData.pingReport.maxRTT,
                scratchData.pingReport.avgRTT);
        return;
        
    case HCI_EVNT_WLAN_UNSOL_DHCP:
        Transport_dhcp(scratchData.dhcp.ip, scratchData.dhcp.subnet, 
                scratchData.dhcp.gateway, scratchData.dhcp.dhcp, 
                scratchData.dhcp.dns);
        return;
    
    default:
        break;
    }
}

static void beginUnsolicitedReceive(uint16_t opcode, uint8_t dataLength) 
{    
    switch (state) {
    case Unsolicited_ReadHeader:
        stateTransition(Unsolicited_Read);
        break;
    case Command_WaitResponse:
        stateTransition(Command_UnsolicitedResponse);
        break;
    case WriteData_WaitResponse:
        stateTransition(WriteData_UnsolicitedResponse);
        break;
    case ReadData_WaitCommandResponse:
        stateTransition(ReadData_UnsolicitedCommandResponse);
        break;
    case ReadData_WaitDataResponse:
        stateTransition(ReadData_UnsolicitedDataResponse);
        break;
        
    default:
        protocolFault();
        return;
    }
    
    unsolicited.opcode = opcode;
    
    if (transferLengthRemaining <= 0) {
        finishUnsolicitedReceive();
        return;
    }
    
    switch (opcode) {
    case HCI_EVNT_DATA_UNSOL_FREE_BUFF:
    {
        dataLength -= 2;
        dataLength /= sizeof(ReleaseBuffersElement);
        if (dataLength > sizeof(scratchData.releaseBuffersFirst.elements) /
                sizeof(ReleaseBuffersElement)) {
            dataLength = sizeof(scratchData.releaseBuffersFirst.elements) /
                sizeof(ReleaseBuffersElement);
        }
        dataLength *= sizeof(ReleaseBuffersElement);
        dataLength += 2;
        
        unsolicited.data.bufferRelease.remainingHandles = 0xFFFF;
        transferLengthRemaining -= dataLength;
        launchReadTransaction(&scratchData.releaseBuffersFirst, dataLength);
        return;
    }
        
    default:
        break;
    }
    
    if (transferLengthRemaining >= sizeof(scratchData)) {
        transferLengthRemaining -= sizeof(scratchData);
        launchReadTransaction(&scratchData, sizeof(scratchData));
        return;
    }
    
    launchReadTransaction(&scratchData, transferLengthRemaining);
    transferLengthRemaining = 0;
}

static void finishCommand(void)
{
    transferComplete();
    
    processing.command->completed = true;    
    stateTransition(Idle);
}

static void finishWrite(void)
{
    transferComplete();
    
    if (processing.write->responseOpcode == 0) {    
        processing.write->completed = true;    
        stateTransition(Idle);
        return;
    }
    
    stateTransition(WriteData_WaitResponse);
}

static void finishRecv(void)
{
    transferComplete();
    
    processing.read->completed = true;    
    stateTransition(Idle);
}

/* Receive into the next buffer.  This will advance the target as needed
 * and set both its and the origin length.  This returns false when there
 * is no more data to be handled in this state. */
static bool receiveBuffer(ReceiveBuffer **target, uint16_t *length)
{
    if (transferLengthRemaining == 0 || *length == 0) {
        for (; *target != NULL; *target = (*target)->next)
            (*target)->length = 0;
        return false;
    }
    if ((*target) == NULL || (*target)->length == 0) {
        if (transferLengthRemaining > *length) {
            launchDiscardTransaction(*length);
            transferLengthRemaining -= *length;
            *length = 0;
            return true;
        }
        launchDiscardTransaction(transferLengthRemaining);
        transferLengthRemaining = 0;
        *length = 0;
        return true;
    }
    
    if (transferLengthRemaining < *length)
        *length = transferLengthRemaining;
    
    if (*length > (*target)->length) {
        launchReadTransaction((*target)->buffer, (*target)->length);
        transferLengthRemaining -= (*target)->length;
        *length -= (*target)->length;
        (*target) = (*target)->next;
        return true;
    }
    
    (*target)->length = *length;
    launchReadTransaction((*target)->buffer, *length);
    transferLengthRemaining -= *length;
    *length = 0;
    for (*target = (*target)->next; *target != NULL; *target = (*target)->next)
        (*target)->length = 0;
    return true;
}

/* Send the next buffer.  This will advance the source as needed.  This returns
 * false when there is no more data to be sent */
static bool sendBuffer(TransmitBuffer **source)
{
    if (*source == NULL || (*source)->length == 0)
        return false;
    launchWriteTransaction((*source)->buffer, (*source)->length);
    (*source) = (*source)->next;
    return true;
}

static bool writeCommandAdvance(void)
{
    if (sendBuffer(&processing.command->tx))
        return true;
    if (processing.command->txPadding != 0) {
        launchDiscardTransaction(processing.command->txPadding);
        processing.command->txPadding = 0;
        return true;
    }
    return false;
}

static bool writeDataAdvance(void)
{
    if (processing.write->arguments != NULL &&
            processing.write->arguments->length != 0) {
        launchWriteTransaction(processing.write->arguments->buffer,
                processing.write->arguments->length);
        processing.write->arguments = processing.write->arguments->next;
        return true;
    }
    if (processing.write->data != NULL &&
            processing.write->data->length != 0) {
        launchWriteTransaction(processing.write->data->buffer,
                processing.write->data->length);
        processing.write->data = processing.write->data->next;
        return true;
    }
    if (processing.write->txPadding != 0) {
        launchDiscardTransaction(processing.write->txPadding);
        processing.write->txPadding = 0;
        return true;
    }
    
    return false;
}

static void commandReadAdvance(void)
{
    if (receiveBuffer(&processing.command->rx,
            &processing.command->bufferRemaining)) {
        return;
    }
    if (transferLengthRemaining > 0) {
        launchDiscardTransaction(transferLengthRemaining);
        transferLengthRemaining = 0;
        return;
    }
    
    finishCommand();
}

static void writeResponseReadAdvance(void)
{
    if (receiveBuffer(&processing.write->response,
            &processing.write->responseRemaining)) {
        return;
    }
    if (transferLengthRemaining > 0) {
        launchDiscardTransaction(transferLengthRemaining);
        transferLengthRemaining = 0;
        return;
    }
    
    transferComplete();
    
    processing.write->completed = true;    
    stateTransition(Idle);
}

void Transport_dataReceived() __attribute__((weak));
void Transport_dataReceived(void)
{
}

static void dataResponseAdvance(void)
{
    if (processing.read->recvProcessing && 
            processing.read->dataRemaining != 0) {
        if (processing.read->dataRemaining < sizeof(scratchData.dataResponse)) {
            protocolFault();
            return;
        }
        launchReadTransaction(&scratchData.dataResponse, 
            sizeof(scratchData.dataResponse));
        processing.read->dataRemaining = 0;
        transferLengthRemaining -= sizeof(scratchData.dataResponse);
        return;
    }
    if (transferLengthRemaining > 0) {
        launchDiscardTransaction(transferLengthRemaining);
        transferLengthRemaining = 0;
        return;
    }
    
    transferComplete();
    
    if (processing.read->recvProcessing) {
        int32_t status = scratchData.dataResponse.status;
        processing.read->result = status;
        if (status <= 0) {
            processing.read->completed = true;
            stateTransition(Idle);
            return;
        }
    } else {
        processing.read->result = 1;
    }
    
    stateTransition(ReadData_WaitDataResponse);
}

static void dataRecvAdvance(void)
{
    if (receiveBuffer(&processing.read->arguments,
            &processing.read->argumentsRemaining)) {
        return;
    }
    if (receiveBuffer(&processing.read->data,
            &processing.read->dataRemaining)) {
        return;
    }
    if (transferLengthRemaining > 0) {
        launchDiscardTransaction(transferLengthRemaining);
        transferLengthRemaining = 0;
        return;
    }
    
    finishRecv();
}

static void incomingHeaderReadCompleted(void)
{
    transferLengthRemaining = ((uint16_t)scratchData.readHeader.lenHI << 8) |
             (uint16_t)scratchData.readHeader.lenLO;
    if (transferLengthRemaining < 5) {
        protocolFault();
        return;
    }
    transferLengthRemaining -= 5;
    
    switch (scratchData.readHeader.type) {
    case HCI_TYPE_DATA: {
        uint8_t argLength = scratchData.readHeader.body.data.argLength;
        uint16_t dataLength = scratchData.readHeader.body.data.totalLength;

        if (dataLength < argLength) {
            protocolFault();
            return;
        }
        dataLength -= argLength;
             
        switch (state) {
        case Unsolicited_ReadHeader:
            if (transferLengthRemaining == 0) {
                stateTransition(Idle);
                transferComplete();
                return;
            }
            stateTransition(Unsolicited_Discard);
            launchDiscardTransaction(transferLengthRemaining);
            return;
        
        case Command_WaitResponse:
            if (transferLengthRemaining == 0) {
                transferComplete();
                return;
            }
            stateTransition(Command_DiscardSpuriousResponse);
            launchDiscardTransaction(transferLengthRemaining);
            return;
            
        case WriteData_WaitResponse:
            if (transferLengthRemaining == 0) {
                transferComplete();
                return;
            }
            stateTransition(WriteData_DiscardSpuriousResponse);
            launchDiscardTransaction(transferLengthRemaining);
            return;
        
        /* Is this always only after the command response? */ 
        case ReadData_WaitCommandResponse:
            if (transferLengthRemaining == 0) {
                transferComplete();
                return;
            }
            stateTransition(ReadData_DiscardSpuriousCommandResponse);
            launchDiscardTransaction(transferLengthRemaining);
            return;
        
        case ReadData_WaitDataResponse:
            if (processing.read->recvProcessing && dataLength != 0)
                Transport_dataReceived();
            stateTransition(ReadData_ReadDataBody);
            processing.read->argumentsRemaining = argLength;
            processing.read->dataRemaining = dataLength;
            dataRecvAdvance();
            return;
        
        default:
            protocolFault();
            return;
        }
        return;
    }
    case HCI_TYPE_EVNT: {
        uint16_t opcode = scratchData.readHeader.body.event.opcode;
        uint8_t dataLength = scratchData.readHeader.body.event.length;
        int8_t status = scratchData.readHeader.body.event.status;
        
        /* Always at least length one for the status code */
        if (dataLength == 0) {
            protocolFault();
            return;
        }
        --dataLength;
        
        switch (opcode) {
        case HCI_EVNT_WLAN_ASYNC_SIMPLE_CONFIG_DONE:
        case HCI_EVNT_WLAN_KEEPALIVE:
        case HCI_EVNT_WLAN_UNSOL_CONNECT:
        case HCI_EVNT_WLAN_UNSOL_DISCONNECT:
        case HCI_EVNT_WLAN_UNSOL_INIT:
        case HCI_EVNT_BSD_TCP_CLOSE_WAIT:
            beginUnsolicitedReceive(opcode, dataLength);
            return;

        case HCI_EVNT_WLAN_UNSOL_DHCP:
        case HCI_EVNT_WLAN_ASYNC_PING_REPORT:
            if (dataLength < 20) {
                protocolFault();
                return;
            }
            beginUnsolicitedReceive(opcode, dataLength);
            return;
        case HCI_EVNT_DATA_UNSOL_FREE_BUFF:
            if (dataLength < 2) {
                protocolFault();
                return;
            }
            beginUnsolicitedReceive(opcode, dataLength);
            return;
            
        default:
            break;
        }
        
        switch (state) {            
        case Command_WaitResponse:
            if (opcode != processing.command->opcode) {
                /* Maybe this should be a fault? */
                if (transferLengthRemaining != 0) {
                    launchDiscardTransaction(transferLengthRemaining);
                    stateTransition(Command_DiscardSpuriousResponse);
                    return;
                }
                transferComplete();
                return;
            }
            
            processing.command->bufferRemaining = dataLength;
            processing.command->status = status;
            
            stateTransition(Command_ReadResponseBody);
            commandReadAdvance();
            return;
            
        case WriteData_WaitResponse:
            if (opcode != processing.write->responseOpcode) {
                /* Maybe this should be a fault? */
                if (transferLengthRemaining != 0) {
                    launchDiscardTransaction(transferLengthRemaining);
                    stateTransition(WriteData_DiscardSpuriousResponse);
                    return;
                }
                transferComplete();
                return;
            }
            
            processing.write->responseRemaining = dataLength;
            processing.write->status = status;
            
            stateTransition(WriteData_ReadResponseBody);
            writeResponseReadAdvance();
            return;
            
        case ReadData_WaitCommandResponse:
            if (opcode != processing.read->opcode) {
                /* Maybe this should be a fault? */
                if (transferLengthRemaining != 0) {
                    launchDiscardTransaction(transferLengthRemaining);
                    stateTransition(ReadData_DiscardSpuriousCommandResponse);
                    return;
                }
                transferComplete();
                return;
            }
            if (status != 0) {
                processing.read->result = -(int)status;
                processing.read->completed = true;
                if (transferLengthRemaining != 0) {
                    stateTransition(Unsolicited_Discard);
                    launchDiscardTransaction(transferLengthRemaining);
                    return;
                }
                stateTransition(Idle);
                transferComplete();
                return;
            }
            
            stateTransition(ReadData_ReadCommandResponseBody);
            processing.read->dataRemaining = dataLength;
            dataResponseAdvance();            
            return;
        
        default:
            /* At least HCI_CMND_CONNECT comes in unsolicited for async
             * accept, so don't consider this a fault in case there are
             * other wierdness like that. */
            beginUnsolicitedReceive(opcode, dataLength);
            return;
        }
        break;
    }
    default:
        protocolFault();
        return;
    }
}

/* We have finished the handling of the start event, so begin the buffer
 * read sequence. */
static void initializeStartCompleted(void)
{
    transferComplete();
    stateTransition(Initialize_ReadBuffersSendWait);
    clearIRQExti();
}

static void initializeBuffersCompleted(void)
{
    uint8_t numberOfBuffers = scratchData.readBuffers.numberOfBuffers;
    
    semaphore_V(&freeBuffers, numberOfBuffers);
    totalBuffers = numberOfBuffers;
        
    transferComplete();
    lastTransportKeepAlive = systick;
    stateTransition(Idle);
}

/* We have finished reading a header of a packet from the device while in
 * a startup state. */
static void initializeHeaderComplete(void)
{
    transferLengthRemaining = ((uint16_t)scratchData.readHeader.lenHI << 8) |
             (uint16_t)scratchData.readHeader.lenLO;
    if (transferLengthRemaining < 5) {
        protocolFault();
        return;
    }
    transferLengthRemaining -= 5;
    
    uint8_t type = scratchData.readHeader.type;
    if (type == HCI_TYPE_EVNT) {
        uint16_t opcode = scratchData.readHeader.body.event.opcode;
        uint8_t dataLength = scratchData.readHeader.body.event.length;
        
        /* Always at least length one for the status code */
        if (dataLength == 0) {
            protocolFault();
            return;
        }
        --dataLength;
             
        switch (opcode) {
        case HCI_CMND_SIMPLE_LINK_START:
            if (state != Initialize_Start) {
                protocolFault();
                return;
            }
            if (transferLengthRemaining != 0) {
                stateTransition(Initialize_StartCompleting);
                launchDiscardTransaction(transferLengthRemaining);
                return;
            }
            initializeStartCompleted();
            return;
            
        case HCI_CMND_READ_BUFFER_SIZE:
            if (state != Initialize_ReadBuffersHeader || 
                    dataLength < sizeof(scratchData.readBuffers) ||
                    transferLengthRemaining > sizeof(scratchData) ||
                    transferLengthRemaining < dataLength) {
                protocolFault();
                return;
            }
            launchReadTransaction(&scratchData.readBuffers, 
                transferLengthRemaining);
            stateTransition(Initialize_ReadBuffersData);
            return;
            
        case HCI_EVNT_PATCHES_REQ:
            #ifdef ENABLE_CC3000_PATCHING
            if (state != Initialize_Start || 
                    dataLength < sizeof(scratchData.patchRequest) ||
                    transferLengthRemaining > sizeof(scratchData) ||
                    transferLengthRemaining < dataLength) {
                protocolFault();
                return;
            }
            launchReadTransaction(&scratchData.patchRequest, 
                transferLengthRemaining);
            stateTransition(Initialize_PatchRequestCompleting);
            #else
            protocolFault();
            #endif
            return;
        
        default:
            break;
        }
    }
    
    switch (state) {
    case Initialize_Start:
        if (transferLengthRemaining != 0) {
            stateTransition(Initialize_StartDiscard);
            launchDiscardTransaction(transferLengthRemaining);
            return;
        }
        break;
    case Initialize_ReadBuffersHeader:
        if (transferLengthRemaining != 0) {
            stateTransition(Initialize_ReadBuffersDiscard);
            launchDiscardTransaction(transferLengthRemaining);
            return;
        }
        break;
    default:
        protocolFault();
        return;
    }
    
    transferComplete();
}

#ifdef ENABLE_CC3000_PATCHING
static void beginPatchTransmission(void)
{
    stopActiveTransaction();        
    assertCS();
    
    const void *discard = NULL;
    uint16_t length = 0;
    Transport_patch(processing.patch.type, &discard, &length);
    
    if (discard == NULL)
        length = 0;
    
    scratchData.writeHeader.spiWrite = SPI_WRITE;
    scratchData.writeHeader.type = HCI_TYPE_PATCH;
    scratchData.writeHeader.body.patch.patchType = processing.patch.type;
        
    scratchData.writeHeader.body.patch.patchLength = length;
    length += 2;
    scratchData.writeHeader.body.patch.packetLength = length;
    
    length += 4;
    if (!(length & 1))
        ++length;
    
    scratchData.writeHeader.spiLenHI = length >> 8;
    scratchData.writeHeader.spiLenLO = length & 0xFF;
    
    launchWriteTransaction(&scratchData.writeHeader, 11);
    stateTransition(Initialize_StartSendingPatchHeader);
    
    clearIRQExti();
}
#endif

static uint8_t fillCommandBuffer(uint16_t opcode, uint16_t length)
{    
    scratchData.writeHeader.spiWrite = SPI_WRITE;
    scratchData.writeHeader.type = HCI_TYPE_CMND;
    scratchData.writeHeader.body.command.opcode = opcode;
    scratchData.writeHeader.body.command.length = length;
    
    length += 4;
    uint8_t padding;
    if (!(length & 1)) {
        ++length;
        padding = 1;
    } else {
        padding = 0;    
    }
    
    scratchData.writeHeader.spiLenHI = length >> 8;
    scratchData.writeHeader.spiLenLO = length & 0xFF;
    return padding;
} 

static void beginReadBuffersTransmission(void)
{
    stopActiveTransaction();        
    assertCS();
    
    stateTransition(Initialize_ReadBuffersSend);
    fillCommandBuffer(HCI_CMND_READ_BUFFER_SIZE, 0);
    launchWriteTransaction(&scratchData.writeHeader, 10);
}

static bool handleBuffersReleased(const ReleaseBuffersElement *data,
                                  uint16_t totalReleased)
{
    for (uint16_t i=0; i<totalReleased; i++) {
        uint16_t nFree = data[i].released;
        semaphore_V(&freeBuffers, nFree);
        bufferCounter += nFree;
    }
    
    if (transferLengthRemaining < sizeof(ReleaseBuffersElement))
        return false;
        
    uint16_t remaining = scratchData.releaseBuffersFirst.numberOfBuffers;
    if (remaining <= totalReleased)
        return false;
    remaining -= totalReleased;
    scratchData.releaseBuffersFirst.numberOfBuffers = remaining;
    
    if (remaining > sizeof(scratchData.releaseBuffers.elements) /
            sizeof(ReleaseBuffersElement)) {
        remaining = sizeof(scratchData.releaseBuffers.elements) /
            sizeof(ReleaseBuffersElement);
    }
    remaining *= sizeof(ReleaseBuffersElement);
    if (remaining > transferLengthRemaining)
        return false;
    launchReadTransaction(scratchData.releaseBuffers.elements, remaining);
    transferLengthRemaining -= remaining;
    return true;
}

/* Called for each completion of an unsolicited read.  true if there is yet
 * more to read, or false once the read has finished (return to the appropriate
 * idle state) */
static void unsolicitedReadCompleted(void)
{
    switch (unsolicited.opcode) {
    case HCI_EVNT_DATA_UNSOL_FREE_BUFF:
        if (unsolicited.data.bufferRelease.remainingHandles == 0xFFFF) {
            uint16_t buffers = 
                scratchData.releaseBuffersFirst.numberOfBuffers;
            unsolicited.data.bufferRelease.remainingHandles = buffers;
            
            if (buffers > sizeof(scratchData.releaseBuffersFirst.elements) /
                    sizeof(ReleaseBuffersElement)) {
                buffers = sizeof(scratchData.releaseBuffersFirst.elements) /
                    sizeof(ReleaseBuffersElement);
            }
            if (handleBuffersReleased(scratchData.releaseBuffersFirst.elements,
                    buffers))
                return;
            break;
        }
        
        uint16_t buffers = unsolicited.data.bufferRelease.remainingHandles;
        if (buffers >= sizeof(scratchData.releaseBuffers.elements) /
                sizeof(ReleaseBuffersElement)) {
            buffers = sizeof(scratchData.releaseBuffers.elements) /
                sizeof(ReleaseBuffersElement);
        }
        if (handleBuffersReleased(scratchData.releaseBuffers.elements,
                buffers))
            return;
        break;
    }
    
    if (transferLengthRemaining > 0) {
        launchDiscardTransaction(transferLengthRemaining);
        transferLengthRemaining = 0;
        return;
    }
    
    finishUnsolicitedReceive();    
}

void Transport_irqRxDMA(void)
{
    stopActiveTransaction();
    
    switch (state) {
    case Initialize_Start:
    case Initialize_ReadBuffersHeader:
        initializeHeaderComplete();
        return;
        
    case Initialize_StartCompleting:
        initializeStartCompleted();
        return;
    case Initialize_StartDiscard:
        stateTransition(Initialize_Start);
        transferComplete();
        return;
    
    #ifdef ENABLE_CC3000_PATCHING
    case Initialize_PatchRequestCompleting: {
        transferComplete();
        processing.patch.type = scratchData.patchRequest.type;
        
        stateTransition(Initialize_PatchRequestWait);
        clearIRQExti();
        return;
    }
    
    case Initialize_StartSendingPatchHeader: {
        const void *data;
        uint16_t length;
        Transport_patch(processing.patch.type, &data, &length);
        
        if (data == NULL)
            length = 0;
        if (length == 0) {
            /* Have to pad to even number of bytes total */
            stateTransition(Initialize_StartSendingPatchData);
            launchDiscardTransaction(1);
            return;
        }
        
        /* We'll over-run the buffer by one and send garbage, but it shouldn't
         * matter since it should just read another random byte of flash. */
        if (!(length & 1))
            ++length;
        
        stateTransition(Initialize_StartSendingPatchData);
        launchWriteTransaction(data, length);
        return;
    }
    
    case Initialize_StartSendingPatchData:
        stateTransition(Initialize_Start);
        transferComplete();
        return;
    #endif
        
    
    case Initialize_ReadBuffersSend:
        transferComplete();
        return;       
    case Initialize_ReadBuffersDiscard:
        stateTransition(Initialize_ReadBuffersHeader);
        transferComplete();
        return;
    case Initialize_ReadBuffersData:
        initializeBuffersCompleted();
        return;
        
        
    case Unsolicited_ReadHeader:
    case Command_WaitResponse:
    case WriteData_WaitResponse:
    case ReadData_WaitCommandResponse:
    case ReadData_WaitDataResponse:
        incomingHeaderReadCompleted();
        return;
        
        
    case Unsolicited_Read:
    case Command_UnsolicitedResponse:
    case WriteData_UnsolicitedResponse:
    case ReadData_UnsolicitedCommandResponse:
    case ReadData_UnsolicitedDataResponse:
        unsolicitedReadCompleted();
        return;
    case Unsolicited_Discard:
        stateTransition(Idle);
        transferComplete();
        return;
        
        
    case Command_WriteHeader:
        stateTransition(Command_WriteBody);
    case Command_WriteBody:
        if (writeCommandAdvance())
            return;
        stateTransition(Command_WaitResponse);
        transferComplete();
        return;
    case Command_ReadResponseBody:
        commandReadAdvance();
        return;
    case Command_DiscardSpuriousResponse:
        stateTransition(Command_WaitResponse);
        transferComplete();
        return;
    
    
    case WriteData_WriteHeader:
        stateTransition(WriteData_WriteBody);
    case WriteData_WriteBody:
        if (writeDataAdvance())
            return;
        finishWrite();
        return;
    case WriteData_ReadResponseBody:
        writeResponseReadAdvance();
        return;
    case WriteData_DiscardSpuriousResponse:
        stateTransition(WriteData_WaitResponse);
        transferComplete();
        return;
        
    
    case ReadData_WriteCommandHeader:
        stateTransition(ReadData_WriteCommandBody);
    case ReadData_WriteCommandBody:
        if (sendBuffer(&processing.read->command))
            return;
        if (processing.read->txPadding != 0) {
            launchDiscardTransaction(processing.read->txPadding);
            processing.read->txPadding = 0;
            return;
        }
        stateTransition(ReadData_WaitCommandResponse);
        transferComplete();
        return;
    case ReadData_ReadCommandResponseBody:
        dataResponseAdvance();
        return;
    case ReadData_ReadDataBody:
        dataRecvAdvance();
        return;
    case ReadData_DiscardSpuriousCommandResponse:
        stateTransition(ReadData_WaitCommandResponse);
        transferComplete();
        return;
    case ReadData_DiscardSpuriousDataResponse:
        stateTransition(ReadData_WaitDataResponse);
        transferComplete();
        return;
        
    default:
        return;
    }
}

static void beginReadHeader(void)
{
    stopActiveTransaction();    
    
    assertCS();
    
    scratchData.readHeader.spiRead = SPI_READ;
    launchReadTransaction(&scratchData.readHeader, 10);
}

void Transport_irqEXTI(void)
{
    clearIRQExti();
    if (!isIRQAsserted())
        return;
    
    switch (state) {
    case Initialize_Start:
    case Command_WaitResponse:
    case ReadData_WaitCommandResponse:
    case ReadData_WaitDataResponse:
    case WriteData_WaitResponse:
        beginReadHeader();
        return;
        
    case Initialize_ReadBuffersSend:
    case Initialize_ReadBuffersHeader:
        stateTransition(Initialize_ReadBuffersHeader);
        beginReadHeader();
        return;
        
    #ifdef ENABLE_CC3000_PATCHING
    case Initialize_PatchRequestWait:
        beginPatchTransmission();
        return;
    #endif
        
    case Initialize_ReadBuffersSendWait:
        beginReadBuffersTransmission();
        return;
    
    
    case Idle:
        stateTransition(Unsolicited_ReadHeader);
        beginReadHeader();
        return;
        
        
    case Command_WriteHeader:
    case ReadData_WriteCommandHeader:
        launchWriteTransaction(&scratchData.writeHeader, 9);
        return;
        
    case WriteData_WriteHeader:
        launchWriteTransaction(&scratchData.writeHeader, 10);
        return;
    
    default:
        return;
    }
}

void Transport_reinitializing() __attribute__((weak));
void Transport_reinitializing(void)
{
}

void Transport_closingTCP() __attribute__((weak));
void Transport_closingTCP(void)
{
}

void Transport_process(void)
{
    disableInterrupts();
    
    switch (state) {
    case Faulted:
        Transport_reinitializing();
        Transport_init();
        break;
    
    #ifdef ENABLE_CC3000_PATCHING
    /* Assume that by the time we get back here we've waited long enough
     * to be able to re-assert CS.  We'll then wait for the IRQ to be
     * asserted if it's not already. */
    case Initialize_PatchRequestWait:        
        assertCS();
        if (!isIRQAsserted())
            break;
        beginPatchTransmission();        
        break;
    #endif
    
    case Initialize_ReadBuffersSendWait:
        assertCS();
        if (!isIRQAsserted())
            break;
        beginReadBuffersTransmission();        
        break;
        
    default:
        break;
        
    }
    
    if (state > Initialize_Spinup && state != Idle) {
        /* Maximum time during startup should be 700ms */
        if (Kernel_elapsed(stateBeginTime) > (uint32_t)(TICK_RATE * 1.5 + 1) &&
                softwareWatchdog != SOFTWARE_WATCHDOG_MAGIC_DISABLED) {
            protocolFault();
            enableInterrupts();
            return;
        }
    }
    
    uint8_t callbackStatus = callbackNotification;
    callbackNotification = 0;
    
    enableInterrupts();
    
    if (callbackStatus & Callback_TCP_Closed) {
        Transport_closingTCP();
    }
}

bool Transport_ready(void)
{
    return state >= Idle;
}

static bool acquireWriteLockInner(SystemState target)
{
    do {
        if (state != Idle)
            break;
        if (isIRQAsserted())
            break;
            
        stateTransition(target);
        
        /* Try to assert CS to indicate that we're writing, but
         * clear any pending IRQ request, since we need to wait for
         * that to begin transmission */
        stopActiveTransaction();     
        assertCS();
        
        /* Assume that an assertion this fast means it was actually a
         * read request incoming, so handle that */
        if (isIRQAsserted()) {
            clearIRQExti();
            stateTransition(Unsolicited_ReadHeader);
            beginReadHeader();
            break;
        }
        
        clearIRQExti();
        
        /* Transitioned to the state, but leave interrupts disabled
         * until we can finish the set up */
        return true;
    } while (0);
        
    enableInterruptsClobber();
    
    /* Yield runs processing too */
    Thread_yield();
    
    return false;
}

/* Try to acquire a write lock on the system state.  This will transition
 * the state to the given target if possible (idle and no read request
 * pending).  Interrupts are left disabled so the caller must enable them
 * once the setup for the write is completed.  This will block until the
 * write state is achieved. */
static void acquireWriteLock(SystemState target)
{
    while (true) {
        disableInterrupts();

        if (acquireWriteLockInner(target))
            return;
    }
}

/* The same as above, but requires a buffer.  This will fail if the system
 * is reset rather than attempting to re-acquire a buffer (since the socket
 * has likely been lost anyway) */
static bool acquireWriteLockWithBuffer(SystemState target)
{
    while (true) {
        disableInterrupts();
        
        if (state < Idle) {
            enableInterrupts();
            return false;
        }
        
        if (acquireWriteLockInner(target))
            return true;
    }
}


int Transport_command(uint16_t opcode, TransmitBuffer *tx, ReceiveBuffer *rx) 
{
    acquireWriteLock(Command_WriteHeader);
    
    CommandProcessing command;
    
    processing.command = &command;
    command.opcode = opcode;
    command.status = 0xFF;
    command.tx = tx;
    command.rx = rx;
    command.bufferRemaining = 0;
    command.completed = false;
    
    uint16_t length = 0;
    for (; tx != NULL; tx = tx->next) {
        length += tx->length;
    }
    
    command.txPadding = fillCommandBuffer(opcode, length);
    
    /* Actual write will be issued from the EXTI handler triggering */
    enableInterruptsClobber();
    
    while (!command.completed) {
        Thread_yield();
    }
    
    return command.status;
}

static int writeInternal(uint8_t opcode, 
                         TransmitBuffer *arguments, TransmitBuffer *data,
                         uint16_t responseOpcode, ReceiveBuffer *response)
{
    WriteProcessing write;
    processing.write = &write;
    write.arguments = arguments;
    write.data = data;
    write.responseOpcode = responseOpcode;
    write.response = response;
    write.status = -1;
    write.completed = false;
    
    uint16_t lengthArguments = 0;
    for (TransmitBuffer *add=arguments; add != NULL; add=add->next) {
        lengthArguments += add->length;
    }
    
    uint16_t lengthData = 0;
    for (TransmitBuffer *add=data; add != NULL; add=add->next) {
        lengthData += add->length;
    }
    
    uint16_t length = lengthData + lengthArguments;
    
    scratchData.writeHeader.spiWrite = SPI_WRITE;
    scratchData.writeHeader.type = HCI_TYPE_DATA;
    scratchData.writeHeader.body.data.opcode = opcode;
    scratchData.writeHeader.body.data.argLength = lengthArguments;
    scratchData.writeHeader.body.data.totalLength = length;
    
    length += 5;
    if (!(length & 1)) {
        ++length;
        write.txPadding = 1;
    } else {
        write.txPadding = 0;
    }
    
    scratchData.writeHeader.spiLenHI = length >> 8;
    scratchData.writeHeader.spiLenLO = length & 0xFF;

    /* Actual write will be issued from the EXTI handler triggering */
    enableInterruptsClobber();
    
    while (!write.completed) {        
        Thread_yield();
    }
    return write.status;
}

int Transport_write(uint8_t opcode, 
                     TransmitBuffer *arguments, TransmitBuffer *data,
                     uint16_t responseOpcode, ReceiveBuffer *response)
{
    acquireWriteLock(WriteData_WriteHeader);
    return writeInternal(opcode, arguments, data, responseOpcode, response);
}

void Transport_send(uint8_t opcode, 
                    TransmitBuffer *arguments, TransmitBuffer *data)
{
    semaphore_P(&freeBuffers, 1);
    if (!acquireWriteLockWithBuffer(WriteData_WriteHeader)) {
        /* Don't need to release the semaphore, as the system is already in
         * a reset process */
        return;
    }
    writeInternal(opcode, arguments, data, 0, NULL);
}

static int readInternal(uint16_t opcode, TransmitBuffer *command, 
                        bool recvMode, ReceiveBuffer *arguments, 
                        ReceiveBuffer *data)
{
    acquireWriteLock(ReadData_WriteCommandHeader);
    
    ReadProcessing read;
    
    processing.read = &read;
    read.opcode = opcode;
    read.result = -1;
    read.command = command;
    read.recvProcessing = recvMode;
    read.arguments = arguments;
    read.data = data;
    read.completed = false;
    
    uint16_t length = 0;
    for (TransmitBuffer *add=command; add != NULL; add=add->next) {
        length += add->length;
    }

    read.txPadding = fillCommandBuffer(opcode, length);
   
    /* Actual write will be issued from the EXTI handler triggering */
    enableInterruptsClobber();
    
    while (!read.completed) {
        Thread_yield();
    }
    
    return read.result;
}

int Transport_read(uint16_t opcode, TransmitBuffer *command, 
                   ReceiveBuffer *arguments, ReceiveBuffer *data)
{
    return readInternal(opcode, command, false, arguments, data);
}

int Transport_recv(uint16_t opcode, TransmitBuffer *command, 
                   ReceiveBuffer *arguments, ReceiveBuffer *data)
{
    return readInternal(opcode, command, true, arguments, data);
}

void Transport_flush(uint32_t timeout)
{
    uint32_t startTime = systick;
    uint16_t startBuffers = bufferCounter;
    do {
        /* Don't need to bother with exclusive reading, since we don't need
         * this to be reliable: if it succeeds we know it's okay and if it
         * fails we'll just try again anyway. */
        if (freeBuffers.s >= totalBuffers)
            break;
        uint16_t currentBuffers = bufferCounter;
        if (currentBuffers >= startBuffers) {
            if ((currentBuffers - startBuffers) >= totalBuffers)
                break;
        } else {
            if ((0xFFFF - currentBuffers + startBuffers) >= totalBuffers)
                break;
        }
        
        Thread_yield();
    } while (Kernel_elapsed(startTime) < timeout);
}

void Transport_waitIdle(void)
{
    while (true) {
        disableInterrupts();
        if (state == Idle)
            break;
        enableInterrupts();
        Thread_yield();
    }
}
