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

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/crc.h>
#include <libopencm3/stm32/f1/dma.h>
#include <libopencm3/stm32/f1/gpio.h>
#include <libopencm3/stm32/f1/rcc.h>
#include <libopencm3/stm32/f1/flash.h>

#include <fulcrum.h>

#include "serial.h"
#include "led.h"
#include "cli.h"

static int tcpServerSocket;
static int tcpConnectedSocket;

#define USART_BUFFER_SIZE       256
static uint8_t usartBuffer[USART_BUFFER_SIZE];
static uint8_t *usartBufferWrite = &usartBuffer[0];
static uint8_t *usartBufferRead = &usartBuffer[0];

#define NETWORK_BUFFER_SIZE     128
static uint8_t networkBuffer[2][NETWORK_BUFFER_SIZE];
static uint32_t networkBufferSize;
static int networkBufferSelected;
static volatile bool networkDMASendingBuffer;

#define SETTINGS_VERSION        1
typedef struct {
    uint32_t crc;
    uint8_t version;
    
    uint16_t tcpPort;
    
    uint32_t baud;
    SerialParity parity;
    uint8_t dataBits;
    uint8_t stopBits;
    bool flowControl;
} SerialSettings;
extern SerialSettings serialSettingsSaved;
static SerialSettings serialSettings;
static bool serialSettingsUpdated;
static bool tcpPortUpdated;
static Mutex mutex;

#define COMMAND_BLANK_TIME  ((uint32_t)(0.25 * 65536))
typedef struct {
    enum {
        Command_Inactive,
        Command_SequenceChecking,
        Command_PostSequenceChecking,
        Command_Active,
    } mode;
    uint32_t timer;
    
    CLIContext context;
} CommandMode;
static CommandMode usartCommand;
static CommandMode networkCommand;

int Serial_getTCPPort(void)
{
    mutex_lock(&mutex);
    int port = serialSettings.tcpPort;
    mutex_unlock(&mutex);
    return port;
}

void Serial_setTCPPort(int port)
{
    if (port == 80 || port <= 0 || port >= 65536)
        return;
        
    mutex_lock(&mutex);
    if (serialSettings.tcpPort != port)
        tcpPortUpdated  = true;
    serialSettings.tcpPort = port;
    mutex_unlock(&mutex);
}

void Serial_getSettings(int *baud, SerialParity *parity, int *dataBits,
                        int *stopBits, bool *flowControl)
{
    mutex_lock(&mutex);
    if (baud != NULL)
        *baud = (int)serialSettings.baud;
    if (parity != NULL)
        *parity = serialSettings.parity;
    if (dataBits != NULL)
        *dataBits = (int)serialSettings.dataBits;
    if (stopBits != NULL)
        *stopBits = (int)serialSettings.stopBits;
    if (flowControl != NULL)
        *flowControl = serialSettings.flowControl;
    mutex_unlock(&mutex);
}

void Serial_setBaud(int baud)
{
    mutex_lock(&mutex);
    serialSettings.baud = baud;
    serialSettingsUpdated = true;
    mutex_unlock(&mutex);
}
void Serial_setParity(SerialParity parity)
{
    mutex_lock(&mutex);
    serialSettings.parity = parity;
    serialSettingsUpdated = true;
    mutex_unlock(&mutex);
}
void Serial_setDataBits(int dataBits)
{
    if (dataBits != 7 && dataBits != 8)
        return;
    mutex_lock(&mutex);
    serialSettings.dataBits = dataBits;
    serialSettingsUpdated = true;
    mutex_unlock(&mutex);
}
void Serial_setStopBits(int stopBits)
{
    if (stopBits != 1 && stopBits != 2)
        return;
    mutex_lock(&mutex);
    serialSettings.stopBits = stopBits;
    serialSettingsUpdated = true;
    mutex_unlock(&mutex);
}
void Serial_setFlowControl(bool flowControl)
{
    mutex_lock(&mutex);
    serialSettings.flowControl = flowControl;
    serialSettingsUpdated = true;
    mutex_unlock(&mutex);
}

static uint32_t calculateSettingsCRC(const SerialSettings *settings)
{
    crc_reset();
    for (const uint32_t *data = (const uint32_t *)(&settings->crc) + 1,
            *end = (const uint32_t *)(settings + 1); data < end; ++data) {
        CRC_DR = *data;
    }
    return CRC_DR;
}

void Serial_saveSettings(void)
{
    mutex_lock(&mutex);
    serialSettings.version = SETTINGS_VERSION;
    serialSettings.crc = calculateSettingsCRC(&serialSettings);
    
    asm volatile ("CPSID i");
    flash_unlock();
    flash_erase_page((uint32_t)(&serialSettingsSaved) + 4);
    FLASH_CR |= FLASH_PG;
    flash_wait_for_last_operation();
    
    volatile uint16_t *target = (volatile uint16_t *)(&serialSettingsSaved);
    for (const uint16_t *data = (const uint16_t *)(&serialSettings),
            *end = (const uint16_t *)(&serialSettings + 1); data < end; ) {
        flash_wait_for_last_operation();
        *target = *data;
        ++target; ++data;
        
        flash_wait_for_last_operation();
        *target = *data;
        ++target; ++data;
    }
    flash_wait_for_last_operation();
    flash_lock();
    asm volatile ("CPSIE i");
    mutex_unlock(&mutex);
}


/**
 * Accept a pending connection, if any.
 */
static void acceptTCPConnection(void)
{
    struct sockaddr_in addr;
    socklen_t len = sizeof(addr);
    if ((tcpConnectedSocket = accept(tcpServerSocket, (struct sockaddr *)&addr, 
            &len)) < 0) {
        tcpConnectedSocket = -1;
        yield();
        return;
    }
    
    /* Reduce the timeout so we run less risk of overflowing the UART receive
     * buffer: MinimumBuffer = this / 1000 * (MaximumBaud/10) */
    uint32_t val = 5;
    if (setsockopt(tcpConnectedSocket, SOL_SOCKET, SOCKOPT_RECV_TIMEOUT,
            &val, sizeof(val)) != 0) {
        closesocket(tcpConnectedSocket);
        yield();
        return;
    }
}

static void openTCPSocket(uint16_t port)
{
    if ((tcpServerSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0) {        
        tcpServerSocket = -1;
        yield();
        return;
    }
    
    struct sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr = INADDR_ANY;
    if (bind(tcpServerSocket, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
        closesocket(tcpServerSocket);
        tcpServerSocket = -1;
        return;
    }
    
    if (listen(tcpServerSocket, 5) != 0) {
        closesocket(tcpServerSocket);
        tcpServerSocket = -1;
        return;
    }
}

static void configureUART(void)
{
    usart_set_baudrate(USART1, serialSettings.baud);
    usart_set_databits(USART1, serialSettings.dataBits);
    usart_set_parity(USART1, serialSettings.parity);
    usart_set_stopbits(USART1, serialSettings.stopBits == 1 ? 
        USART_STOPBITS_1 : USART_STOPBITS_2);
    usart_set_flow_control(USART1, serialSettings.flowControl ? 
        USART_FLOWCONTROL_RTS_CTS : USART_FLOWCONTROL_NONE);
    usart_set_mode(USART1, USART_MODE_TX_RX);
}

static char getNextCharacter(uint8_t *wr, uint8_t **rd)
{
    if (*rd < wr) {
        char result = **rd;
        (*rd)++;
        return result;
    } else if (*rd > wr) {
        char result = **rd;
        (*rd)++;
        if (*rd >= &usartBuffer[USART_BUFFER_SIZE]) {
            *rd = &usartBuffer[0];
        }
        return result;
    } else {
        *rd = NULL;
        return 0;
    }
}
static bool handleUSARTRead(void)
{
    /* Write any data from the circular buffer */
    uint8_t *wr = usartBufferWrite;
    uint8_t *rd = usartBufferRead;
    
    switch (usartCommand.mode) {
    case Command_Inactive:
        if (wr == rd)
            return false;
            
        if (*rd != '$' || time_fractional_elapsed(usartCommand.timer) < 
                COMMAND_BLANK_TIME) {
            usartCommand.timer = time_fractional();
            break;
        }
        
        usartCommand.timer = time_fractional();
        usartCommand.mode = Command_SequenceChecking;        
        /* Fall through */
    case Command_SequenceChecking: 
    case Command_PostSequenceChecking: {
        if (time_fractional_elapsed(usartCommand.timer) > 65536) {
            usartCommand.timer = time_fractional();
            usartCommand.mode = Command_Inactive;
            break;
        }
        
        char check = getNextCharacter(wr, &rd);
        if (rd == NULL)
            return false;
        if (check != '$') {
            usartCommand.timer = time_fractional();
            usartCommand.mode = Command_Inactive;
            break;
        }
        
        check = getNextCharacter(wr, &rd);
        if (rd == NULL)
            return false;
        if (check != '$') {
            usartCommand.timer = time_fractional();
            usartCommand.mode = Command_Inactive;
            break;
        }
        
        check = getNextCharacter(wr, &rd);
        if (rd == NULL)
            return false;
        if (check != '$') {
            usartCommand.timer = time_fractional();
            usartCommand.mode = Command_Inactive;
            break;
        }
        
        if (usartCommand.mode == Command_SequenceChecking) {
            usartCommand.mode = Command_PostSequenceChecking;
            usartCommand.timer = time_fractional();
        }
        
        check = getNextCharacter(wr, &rd);
        if (rd == NULL)
            return false;
        if (check != '\r' && check != '\n') {
            usartCommand.timer = time_fractional();
            usartCommand.mode = Command_Inactive;
            break;
        }
        
        check = getNextCharacter(wr, &rd);
        if (rd == NULL)
            return false;
        if (check != '\r' && check != '\n') {
            usartCommand.timer = time_fractional();
            usartCommand.mode = Command_Inactive;
            break;
        }
        
        getNextCharacter(wr, &rd);
        if (rd != NULL) {
            usartCommand.timer = time_fractional();
            usartCommand.mode = Command_Inactive;
            break;
        }
        
        return false;
    }
    
    case Command_Active:  
        rd = usartBufferRead;
        if (wr == rd)
            return false;
        if (rd < wr) {
            CLI_incoming(&usartCommand.context, (const char *)rd, 
                wr-usartBufferRead);
            usartBufferRead = wr;
            LED_blink();
        } else {
            CLI_incoming(&usartCommand.context, (const char *)rd, 
                &usartBuffer[USART_BUFFER_SIZE-1] - rd);
            if (wr != &usartBuffer[0]) {
                CLI_incoming(&usartCommand.context, 
                    (const char *)&usartBuffer[0], wr - &usartBuffer[0]);
            }
            usartBufferRead = wr;
            LED_blink();
        }
        return true;    
    }
    
    if (networkCommand.mode != Command_Inactive)
        return false;
    
    rd = usartBufferRead;
    if (wr == rd)
        return false;
    if (rd < wr) {
        send(tcpConnectedSocket, rd, wr-usartBufferRead);
        usartBufferRead = wr;
        LED_blink();
    } else {
        send(tcpConnectedSocket, rd, 
            &usartBuffer[USART_BUFFER_SIZE-1] - usartBufferRead);
        if (wr != &usartBuffer[0]) {
            send(tcpConnectedSocket, &usartBuffer[0], 
                wr - &usartBuffer[0]);
        }
        usartBufferRead = wr;
        LED_blink();
    }
    return true;
}

static bool handleNetworkRead(void)
{
    /* No space, so stop receives.  TCP flow control through throttle
     * incoming if this lasts long enough. */
    if (networkBufferSize == NETWORK_BUFFER_SIZE) {
        return false;
    }
    
    /* Read any available data */
    int n = recv(tcpConnectedSocket, 
        &networkBuffer[networkBufferSelected][networkBufferSize], 
        NETWORK_BUFFER_SIZE - networkBufferSize);
    if (n < 0) {
        closesocket(tcpConnectedSocket);
        tcpConnectedSocket = -1;
        return true;
    }
    networkBufferSize += n;
    
    switch (networkCommand.mode) {
    case Command_Inactive:
        if (n == 0)
            break;
        if (networkBuffer[networkBufferSelected][0] != '$')
            break;
        
        networkCommand.timer = time_fractional();
        networkCommand.mode = Command_SequenceChecking;        
        /* Fall through */
    case Command_SequenceChecking: 
    case Command_PostSequenceChecking: {
        if (time_fractional_elapsed(networkCommand.timer) > 65536) {
            networkCommand.timer = time_fractional();
            networkCommand.mode = Command_Inactive;
            break;
        }
        
        uint32_t offset = 0;
        if (offset >= networkBufferSize)
            return false;
        if (networkBuffer[networkBufferSelected][offset] != '$') {
            networkCommand.timer = time_fractional();
            networkCommand.mode = Command_Inactive;
            break;
        }
        ++offset;
        
        if (offset >= networkBufferSize)
            return false;
        if (networkBuffer[networkBufferSelected][offset] != '$') {
            networkCommand.timer = time_fractional();
            networkCommand.mode = Command_Inactive;
            break;
        }
        ++offset;
        
        if (offset >= networkBufferSize)
            return false;
        if (networkBuffer[networkBufferSelected][offset] != '$') {
            networkCommand.timer = time_fractional();
            networkCommand.mode = Command_Inactive;
            break;
        }
        ++offset;
        
        if (networkCommand.mode == Command_SequenceChecking) {
            networkCommand.mode = Command_PostSequenceChecking;
            networkCommand.timer = time_fractional();
        }
        
        if (offset >= networkBufferSize)
            return false;
        if (networkBuffer[networkBufferSelected][offset] != '\r' &&
                networkBuffer[networkBufferSelected][offset] != '\n') {
            networkCommand.timer = time_fractional();
            networkCommand.mode = Command_Inactive;
            break;
        }
        ++offset;
        
        if (offset >= networkBufferSize)
            return false;
        if (networkBuffer[networkBufferSelected][offset] != '\r' &&
                networkBuffer[networkBufferSelected][offset] != '\n') {
            networkCommand.timer = time_fractional();
            networkCommand.mode = Command_Inactive;
            break;
        }
        ++offset;
        
        if (offset < networkBufferSize) {
            networkCommand.timer = time_fractional();
            networkCommand.mode = Command_Inactive;
            break;
        }
        
        return false;
        
    case Command_Active:
        if (networkBufferSize == 0)
            return false;
        if (n == 0)
            return false;
            
        LED_blink();
        CLI_incoming(&networkCommand.context,
            (const char *)&networkBuffer[networkBufferSelected], 
            networkBufferSize);
        networkBufferSize = 0;
        return true;
    }
    }
    
    if (n == 0)
        return false;
        
    LED_blink();
    return true;
}

/**
 * The handler for serial operations.
 */
void Serial_run(void)
{
    while (true) {
        mutex_lock(&mutex);
        uint16_t port = serialSettings.tcpPort;
        if (tcpPortUpdated) {
            tcpPortUpdated = false;
            mutex_unlock(&mutex);
            networkCommand.mode = Command_Inactive;
            if (tcpConnectedSocket >= 0) {
                closesocket(tcpConnectedSocket);
                tcpConnectedSocket = -1;
            }
            if (tcpServerSocket >= 0) {
                closesocket(tcpServerSocket);
                tcpServerSocket = -1;
            }
            continue;
        }
        
        if (serialSettingsUpdated) {
            serialSettingsUpdated = false;
            configureUART();
            mutex_unlock(&mutex);
            continue;
        }
        mutex_unlock(&mutex);
        
        if (tcpServerSocket < 0) {
            openTCPSocket(port);
            continue;
        }
        
        bool didUSART = handleUSARTRead();
        
        if (tcpConnectedSocket < 0) {
            networkBufferSize = 0;
            usartBufferRead = usartBufferWrite;
            networkCommand.mode = Command_Inactive;
            
            acceptTCPConnection();
        }
        
        if (usartCommand.mode == Command_PostSequenceChecking && 
                time_fractional_elapsed(usartCommand.timer) > 
                COMMAND_BLANK_TIME) {
            usartCommand.mode = Command_Active;
            usartBufferRead = usartBufferWrite;
            CLI_enter(&usartCommand.context);
            continue;
        }
        
        if (networkCommand.mode == Command_PostSequenceChecking && 
                time_fractional_elapsed(networkCommand.timer) > 
                COMMAND_BLANK_TIME) {
            networkCommand.mode = Command_Active;
            networkBufferSize = 0;
            CLI_enter(&networkCommand.context);
            continue;
        }
        
        if (networkBufferSize != 0 && !networkDMASendingBuffer &&
                networkCommand.mode == Command_Inactive &&
                usartCommand.mode == Command_Inactive) {
            /* Have data to be sent, to issue a DMA transaction */
            dma_disable_channel(DMA1, DMA_CHANNEL4);
            networkDMASendingBuffer = true;
            dma_set_memory_address(DMA1, DMA_CHANNEL4, 
                (u32)(&networkBuffer[networkBufferSelected]));
            dma_set_number_of_data(DMA1, DMA_CHANNEL4, 
                (u32)(networkBufferSize));
            USART_SR(USART1) = USART_SR_TC;
            DMA1_IFCR = DMA_ISR_GIF4;
            dma_enable_channel(DMA1, DMA_CHANNEL4);
            usart_enable_tx_dma(USART1);
            
            networkBufferSize = 0;
            if (networkBufferSelected == 0) {
                networkBufferSelected = 1;
            } else {
                networkBufferSelected = 0;
            }
        }
        
        if (usartCommand.mode == Command_Active) {
            if (!CLI_process(&usartCommand.context)) {
                usartCommand.timer = time_fractional();
                usartCommand.mode = Command_Inactive;
                usartBufferRead = usartBufferWrite;
            }
            if (usartCommand.context.output[0] &&
                    !networkDMASendingBuffer) {
                int target = networkBufferSelected;
                if (target == 0)
                    target = 1;
                else
                    target = 0;
                strcpy((char *)networkBuffer[target], 
                    usartCommand.context.output);
                usartCommand.context.output[0] = 0;
                
                dma_disable_channel(DMA1, DMA_CHANNEL4);
                networkDMASendingBuffer = true;
                dma_set_memory_address(DMA1, DMA_CHANNEL4, 
                    (u32)(&networkBuffer[target]));
                dma_set_number_of_data(DMA1, DMA_CHANNEL4, 
                    (u32)(strlen((char *)networkBuffer[target])));
                USART_SR(USART1) = USART_SR_TC;
                DMA1_IFCR = DMA_ISR_GIF4;
                dma_enable_channel(DMA1, DMA_CHANNEL4);
                usart_enable_tx_dma(USART1);
            }
        }
        
        if (tcpConnectedSocket < 0) {
            if (!didUSART)
                yield();
            continue;
        }
        
        bool didNetwork = handleNetworkRead();
        
        if (networkCommand.mode == Command_Active) {
            if (!CLI_process(&networkCommand.context)) {
                networkCommand.timer = time_fractional();
                networkCommand.mode = Command_Inactive;
                networkBufferSize = 0;
            }
            if (networkCommand.context.output[0]) {
                send(tcpConnectedSocket, networkCommand.context.output, 
                    strlen(networkCommand.context.output));
                networkCommand.context.output[0] = 0;
            }
        }
        
        if (!didUSART && !didNetwork)
            yield();
    }
}

void usart1_isr(void)
{
    uint8_t data = USART_DR(USART1);
    
    /* Parity will be in the 8th bit on E71, for example */
    if (serialSettings.dataBits != 8)
        data &= 0x7F;
    
    *usartBufferWrite = data;
    if (usartBufferWrite == &usartBuffer[USART_BUFFER_SIZE-1]) {
        if (usartBufferRead != &usartBuffer[0]) {
            usartBufferWrite = &usartBuffer[0];
        }
    } else {
        if (usartBufferWrite+1 != usartBufferRead) {
            ++usartBufferWrite;
        }
    }
}

void dma1_channel4_isr(void)
{
    dma_disable_channel(DMA1, DMA_CHANNEL4);
    usart_disable_tx_dma(USART1);
    DMA1_IFCR = DMA_ISR_GIF4;
    
    networkDMASendingBuffer = false;
}

/**
 * Handle a network reset by the kernel.
 */
static void handleNetworkReset(void)
{
    tcpServerSocket = -1;
    tcpConnectedSocket = -1;
    networkCommand.mode = Command_Inactive;
}

/**
 * Handle a remotely closed TCP socket.
 */
static void handleTCPClosed(void)
{
    if (tcpConnectedSocket == -1)
        return;
    closesocket(tcpConnectedSocket);
    tcpConnectedSocket = -1;
    networkCommand.mode = Command_Inactive;
}

void Serial_init(void)
{
    handler_network_reset(handleNetworkReset);
    handler_tcp_closed(handleTCPClosed);
    handleNetworkReset();
    
    rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPAEN);
    rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_USART1EN);
    rcc_peripheral_enable_clock(&RCC_AHBENR, RCC_AHBENR_CRCEN);
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
        GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, 
        GPIO_USART1_TX | GPIO11);
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
        GPIO_CNF_INPUT_FLOAT, 
        GPIO_USART1_RX | GPIO12);
        
        
    mutex_init(&mutex);
        
    if (calculateSettingsCRC(&serialSettingsSaved) == 
            serialSettingsSaved.crc && serialSettingsSaved.version ==   
            SETTINGS_VERSION) {
        memcpy(&serialSettings, &serialSettingsSaved,
            sizeof(serialSettings));
    } else {
        serialSettings.tcpPort = 23;
        serialSettings.baud = 9600;
        serialSettings.parity = Parity_None;
        serialSettings.dataBits = 8;
        serialSettings.stopBits = 1;
        serialSettings.flowControl = false;
    }
        
    configureUART();
    
    nvic_enable_irq(NVIC_USART1_IRQ);
    nvic_enable_irq(NVIC_DMA1_CHANNEL4_IRQ);
    nvic_set_priority(NVIC_DMA1_CHANNEL4_IRQ, 16);
    
    usart_enable_rx_interrupt(USART1);
    usart_enable(USART1);
    
    dma_channel_reset(DMA1, DMA_CHANNEL4);
    dma_set_priority(DMA1, DMA_CHANNEL4, DMA_CCR_PL_LOW);
    dma_set_memory_size(DMA1, DMA_CHANNEL4, DMA_CCR_MSIZE_8BIT);
    dma_set_peripheral_size(DMA1, DMA_CHANNEL4, DMA_CCR_PSIZE_8BIT);
    dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL4);
    dma_set_read_from_memory(DMA1, DMA_CHANNEL4);
    dma_set_peripheral_address(DMA1, DMA_CHANNEL4, (u32)&USART_DR(USART1));
    dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL4);
}
