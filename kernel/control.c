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
#include <string.h>

#include <libopencm3/stm32/iwdg.h>
#include <libopencm3/stm32/f1/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/f1/flash.h>

#include "kernel.h"
#include "control.h"
#include "netapi.h"
#include "transport.h"
#include "setup.h"
#include "server.h"
#include "init.h"

ControlState controlState;
ControlData controlData;
uint32_t controlStateStartTime;

static bool wlanConnected;

enum {
    /* Have seen recent activity, so just keep waiting */
    Activity_Recent,
    /* Just got a response so stop any active ping */
    Activity_PingStop,
    /* Currently waiting for the response (if any) to the gateway 
     * address ping */
    Activity_Ping_Gateway,
    /* Enough time has elapsed that we've requested an explicit ping status */
    Activity_Ping_GatewayStatusRequested,
    /* Waiting for a resposne to the DNS server ping */
    Activity_Ping_DNS,
    /* Enough time has elapsed that we've requested an explicit ping status */
    Activity_Ping_DNSStatusRequested,
    /* A response to the DHCP server ping */
    Activity_Ping_DHCP,
    /* Enough time has elapsed that we've requested an explicit ping status */
    Activity_Ping_DHCPStatusRequested,
} activityDetectState;
static uint32_t activityDetectTime;

#pragma pack(push,4)
#define PERSISTENTCONFIGURATIONVERSION  1
typedef struct {
    uint8_t crc;
    uint8_t version;
    bool networkConfigured;
    
    char authorization[64];    
    uint32_t userCodeLength;
} PersistentConfiguration;
#pragma pack(pop)
extern const PersistentConfiguration _config;
extern unsigned _kernelbegin, _userbegin;
static bool savedConfigurationValid;

static void flashWriteBegin(void)
{
    disableInterrupts();
    FLASH_KEYR = FLASH_KEY1;
	FLASH_KEYR = FLASH_KEY2;
    FLASH_SR = FLASH_WRPRTERR|FLASH_PGERR;
}
static void flashWriteEnd(void)
{
    FLASH_CR = FLASH_LOCK;
    enableInterrupts();
}
static void flashWaitCompleted(void)
{
    while ((FLASH_SR & FLASH_BSY) == FLASH_BSY) { }
}
static void flashWrite(uint32_t address, const void *data, uint32_t n) 
{        
    const uint16_t *word = (const uint16_t *)data;
    
    flashWaitCompleted();
    FLASH_CR = FLASH_PG;
    for (uint32_t end=address+n; address<end; word++, address+=2) {
        (*(volatile uint16_t *)address) = *word;
        flashWaitCompleted();
    }
    FLASH_CR = 0;
}
static void flashErasePage(uint32_t address)
{
    FLASH_SR = FLASH_WRPRTERR|FLASH_PGERR;
    flashWaitCompleted();
    FLASH_CR = FLASH_PER;
    FLASH_AR = address+4;
    FLASH_CR |= FLASH_STRT;
    flashWaitCompleted();
    FLASH_CR = 0;
}

/* Don't use the built in CRC-32 because we want to be able to do this
 * while the user code is running (which may have invoked it). */
static uint8_t crc8(const uint8_t *data, uint32_t length)
{
    uint8_t result = 0;
    for (; length>0; --length, ++data) {
        uint16_t working = (*data) ^ result;
        working <<= 8;
        for (int i=0; i<8; i++) {
            if (working & 0x8000) {
                working ^= 0x1070 << 3;
            }
            working <<= 1;
        }
        result = (uint8_t)(working >> 8);
    }
    return result;
}
static uint8_t calculateConfigCRC(const PersistentConfiguration *config)
{
    return crc8(&config->crc + 1, sizeof(PersistentConfiguration)-1);
}
static void writeConfig(PersistentConfiguration *input)
{
    input->version = PERSISTENTCONFIGURATIONVERSION;
    input->crc = calculateConfigCRC(input);
    flashWriteBegin();
    flashErasePage((uint32_t)(&_config));
    flashWrite((uint32_t)(&_config), input, sizeof(PersistentConfiguration));
    flashWriteEnd();
    savedConfigurationValid = true;
} 

static bool haveValidConfiguration(void)
{
    return savedConfigurationValid;
}
static bool haveNetworkConfiguration(void)
{
    if (!haveValidConfiguration())
        return false;
    return _config.networkConfigured;
}

bool Control_haveUserCode(void)
{
    if (!haveValidConfiguration())
        return false;
    return _config.userCodeLength != 0;
}

uint32_t Control_userCodeLength(void)
{
    if (!haveValidConfiguration())
        return 0;
    return _config.userCodeLength;
}

bool Control_requireAuthorization(void)
{
    if (!haveValidConfiguration())
        return false;
    return _config.authorization[0] != 0;
}

bool Control_checkAuthorization(const char *encoded)
{
    if (!Control_requireAuthorization())
        return true;
    for (const char *check=_config.authorization; ; ++check, ++encoded) {
        char e = *encoded;
        char c = *check;
        if (!c)
            return !e || e == '\r' || e == '\n';
        if (c != e)
            return false;
    }
    return true;
}

static void buildEffectiveConfig(PersistentConfiguration *output)
{
    if (!haveValidConfiguration()) {
        output->networkConfigured = false;
        output->authorization[0] = 0;
        output->userCodeLength = 0;
        return;
    }
    memcpy(output, &_config, sizeof(PersistentConfiguration));
}

static void resetSystem(void)
{
    flashWriteBegin();
    flashErasePage((uint32_t)(&_config));
    flashWriteEnd();
    savedConfigurationValid = false;
    systemStatus.state = RunState_Setup;
}

static void systemConfigured(void)
{
    PersistentConfiguration config;
    buildEffectiveConfig(&config);
    config.networkConfigured = true;
    writeConfig(&config);
}

static void setUserCodeLength(uint32_t length)
{
    PersistentConfiguration config;
    buildEffectiveConfig(&config);
    config.userCodeLength = length;
    writeConfig(&config);
}

void Control_setAuthorization(const char *encoded)
{
    PersistentConfiguration config;
    buildEffectiveConfig(&config);
    uint32_t len = strlen(encoded);
    if (len == 0 || len >= sizeof(config.authorization)) {
        config.authorization[0] = 0;
    } else {
        memcpy(config.authorization, encoded, len+1);
    }
    writeConfig(&config);
}

static uint32_t maximumUserCodeAddress(void)
{
    return (uint32_t)&_kernelbegin + Kernel_totalFlashSize() * 1024;
}
void Control_writeUserCodeField(const char *name)
{
    (void)name;
    controlData.userCodeUpload.address = (uint32_t)&_userbegin;
    controlData.userCodeUpload.nPending = 0;
    Kernel_haltUserCode();
}

extern void (*const vector_table[]) (void);
static void writeUserCodePending(void)
{
    if (controlData.userCodeUpload.address < (uint32_t)&_userbegin) {
        controlData.userCodeUpload.nPending = 0;
        return;
    }
    if (controlData.userCodeUpload.address >= maximumUserCodeAddress()) {
        controlData.userCodeUpload.nPending = 0;
        controlData.userCodeUpload.address = 0;
        return;
    }
    flashWriteBegin();
    if (controlData.userCodeUpload.address % 1024 == 0) {
        flashErasePage(controlData.userCodeUpload.address);    
    }
    
    uint32_t irq = (controlData.userCodeUpload.address - 
        (uint32_t)&_userbegin)/4;
    switch (irq) {
    case 2:
    case 3:
    case 4:
    case 5:
    case 6:
    case 14:
    case 15:
    case 28:
    case 56:
        flashWrite(controlData.userCodeUpload.address, 
            &vector_table[irq], 4);
        break;
    default:
        flashWrite(controlData.userCodeUpload.address, 
            controlData.userCodeUpload.pending, 
            controlData.userCodeUpload.nPending);
        break;
    }
    
    flashWriteEnd();
    controlData.userCodeUpload.address += controlData.userCodeUpload.nPending;
    controlData.userCodeUpload.nPending = 0;
}

void Control_writeUserCodeData(const void *data, uint32_t n)
{
    const uint8_t *add = data;
    while (n > 0) {
        uint32_t nAdd = n;
        uint32_t available = 4 - controlData.userCodeUpload.nPending;
        if (nAdd > available) {
            nAdd = available;
        }
        memcpy(&controlData.userCodeUpload.pending[
            controlData.userCodeUpload.nPending], add, nAdd);
        controlData.userCodeUpload.nPending += nAdd;
        add += nAdd;
        n -= nAdd;
        if (controlData.userCodeUpload.nPending >= 4) {
            writeUserCodePending();
        }
    }
}

void Control_writeUserCodeCompleted(int fd)
{
    if (fd == -1) {
        setUserCodeLength(0);
        return;
    }
    
    if (controlData.userCodeUpload.nPending != 0)
        writeUserCodePending();
    
    if (controlData.userCodeUpload.address < (uint32_t)&_userbegin + 56 * 4) {
        setUserCodeLength(0);
        Server_reloadStringResponse("Invalid data, user code disabled.");
        return;
    }
    
    setUserCodeLength(controlData.userCodeUpload.address - 
        (uint32_t)&_userbegin);
    systemStatus.state = RunState_Initialize;
    Server_reloadStringResponse("User code uploaded.");
}


static void networkReset(void)
{
    Server_clearSockets();
    wlanConnected = false;
    controlStateStartTime = systick;
    activityDetectState = Activity_Recent;
    activityDetectTime = systick;
    
    if (systemStatus.state == RunState_UserExecuting && 
            systemStatus.detailed.normal.networkResetHandler) {
        (systemStatus.detailed.normal.networkResetHandler)();
    }
}

void Control_networkFault(void)
{
    if (!Transport_ready()) {
        Server_clearSockets();
        controlState = Offline_WaitingForInitialize;
    } else {
        Server_closeSockets();
    }
    
    if (systemStatus.state != RunState_UserExecuting)
        return;
    /* Too many errors and we trigger a fault that halts user code and
     * assume it's mucked with the network layer in a bad way. */
    if (++systemStatus.detailed.normal.networkFaultCounter > 10) {
        Kernel_haltUserCode();
        networkReset();
        Transport_init();
        controlState = Offline_WaitingForInitialize;
        Kernel_softwareUserFault(RunState_FailureHalted_SocketIO);
        return;
    }
    systemStatus.detailed.normal.lastNetworkFault = systick;
}

void Transport_reinitializing(void)
{
    networkReset();
        
    if (systemStatus.state != RunState_UserExecuting)
        return;
        
    if (++systemStatus.detailed.normal.networkFaultCounter > 10) {
        controlState = Offline_TooManyUserFaultsPending;
        return;
    }
    systemStatus.detailed.normal.lastNetworkFault = systick;
}

void Transport_closingTCP(void)
{
    if (httpClientSocket != -1) {
        closesocket(httpClientSocket);
        httpClientSocket = -1;
    }
    if (systemStatus.state != RunState_UserExecuting)
        return;
    if (!systemStatus.detailed.normal.tcpClosedHandler)
        return;
    (systemStatus.detailed.normal.tcpClosedHandler)();
}

void Transport_connected(bool connected)
{
    wlanConnected = connected;
}

void Transport_dataReceived(void)
{
    switch (activityDetectState) {
    case Activity_Recent:
    case Activity_PingStop:
        break;
    default:
        activityDetectState = Activity_PingStop;
        break;
    }
    activityDetectTime = systick;
}
void Transport_pingReport(uint32_t packetsSent, uint32_t packetsReceived,
                          uint32_t minRTT, uint32_t maxRTT, uint32_t avgRTT)
{
    (void)packetsSent;
    (void)minRTT;
    (void)maxRTT;
    (void)avgRTT;
    if (packetsReceived != 0)
        Transport_dataReceived();
}

static bool networkWatchdogProcess(void)
{
    if (Kernel_elapsed(lastTransportKeepAlive) > 300 * TICK_RATE)
        return false;
        
    switch (activityDetectState) {
    case Activity_PingStop:
        activityDetectState = Activity_Recent;
        ioctl_ping_stop();
        break;
    case Activity_Recent: {
        if (Kernel_elapsed(activityDetectTime) < 120 * TICK_RATE)
            break;
            
        activityDetectState = Activity_Ping_Gateway;
        struct in_addr addr;
        if (ioctl_network_status(NULL, NULL, &addr, NULL, NULL, 
                NULL, NULL, 0) == 0 && addr.s_addr != 0) {
            if (ioctl_ping_start(addr, 10000, 4, 8) == 0) {
                activityDetectTime = systick;
                break;
            }
        }
    }
    /* Fall through */
    case Activity_Ping_GatewayStatusRequested: {            
        activityDetectState = Activity_Ping_DNS;
        struct in_addr addr;
        if (ioctl_network_status(NULL, NULL, NULL, &addr, NULL, 
                NULL, NULL, 0) == 0 && addr.s_addr != 0) {
            if (ioctl_ping_start(addr, 10000, 4, 8) == 0) {
                activityDetectTime = systick;
                break;
            }
        }
    }
    /* Fall through */
    case Activity_Ping_DNSStatusRequested: {
        if (Kernel_elapsed(activityDetectTime) < 60 * TICK_RATE)
            break;
            
        activityDetectState = Activity_Ping_DHCP;
        struct in_addr addr;
        if (ioctl_network_status(NULL, NULL, NULL, NULL, &addr, 
                NULL, NULL, 0) == 0 && addr.s_addr != 0) {
            if (ioctl_ping_start(addr, 10000, 4, 8) == 0) {
                activityDetectTime = systick;
                break;
            }
        }
    }
    /* Fall through */
    case Activity_Ping_DHCPStatusRequested:
        if (Kernel_elapsed(activityDetectTime) < 60 * TICK_RATE)
            break;
        return false;
        
        
    case Activity_Ping_Gateway:
        if (Kernel_elapsed(activityDetectTime) < 50 * TICK_RATE)
            break;
        ioctl_ping_status();
        activityDetectState = Activity_Ping_GatewayStatusRequested;
        break;
    
    case Activity_Ping_DNS:
        if (Kernel_elapsed(activityDetectTime) < 50 * TICK_RATE)
            break;
        ioctl_ping_status();
        activityDetectState = Activity_Ping_DNSStatusRequested;
        break;
        
    case Activity_Ping_DHCP:
        if (Kernel_elapsed(activityDetectTime) < 50 * TICK_RATE)
            break;
        ioctl_ping_status();
        activityDetectState = Activity_Ping_DHCPStatusRequested;
        break;
    }
        
    return true;
}


static void systemProcess(void)
{
    switch (controlState) {
    case Offline_Spinup:
        controlState = Offline_WaitingForInitialize;
        Transport_init();
        break;
    case Offline_WaitingForInitialize:
        Thread_yield();
        networkReset();
        if (Kernel_elapsed(controlStateStartTime) > 60 * TICK_RATE) {
            Transport_init();
            break;
        }
        if (!Transport_ready())
            break;
        controlState = Idle;
        break;
    case Offline_TooManyUserFaultsPending:
        if (systemStatus.state == RunState_UserExecuting) {
            Kernel_softwareUserFault(RunState_FailureHalted_SocketIO);
        }
        controlState = Idle;
        break;
    case Offline_RestartPending:
        networkReset();
        Transport_init();
        controlState = Offline_WaitingForInitialize;
        break;
        
    case Setup_ModeEnter:
        networkReset();
        controlState = Setup_ModeInitializing;
        Transport_init();
        break;
    case Setup_ModeInitializing:
        if (!Transport_ready()) {
            if (Kernel_elapsed(controlStateStartTime) > 60 * TICK_RATE)
                Transport_init();
            break;            
        }
        ioctl_set_connection_policy(false, false, false);
        Kernel_refreshWatchdog();
        ioctl_set_timeouts(1800, 600, 20, 0);
        Kernel_refreshWatchdog();
        ioctl_smart_config_start();
        Kernel_refreshWatchdog();            
        controlState = Setup_Mode;
        /* Fall through */
    case Setup_Mode:
        if (!Transport_ready()) {
            controlState = Setup_ModeInitializing;
            break;
        }
        Setup_process();
        break;
    case Setup_SimpleConfigDone:
        ioctl_network_dhcp();
        Kernel_refreshWatchdog();
        ioctl_set_connection_policy(true, true, false);
        Kernel_refreshWatchdog();
        /* Fall through */
    case Setup_Apply:
        Transport_init();
        controlState = Setup_Connecting;
        systemStatus.detailed.setup.connectionAttemptStart = systick;
        break;
    case Setup_Connecting:
        if (wlanConnected) {
            Setup_leave();
            systemConfigured();
        } else if (Kernel_elapsed(systemStatus.detailed.setup.
                connectionAttemptStart) > 30 * TICK_RATE) {
            /* Re-enter so we reset everything */
            Setup_enter();
        }
        break;
        
    case Idle:
        if (!Transport_ready()) {
            controlState = Offline_WaitingForInitialize;
            controlStateStartTime = systick;
            break;
        }
        if (!networkWatchdogProcess()) {
            networkReset();
            Transport_init();
            controlState = Offline_WaitingForInitialize;
            controlStateStartTime = systick;
            break;
        }
        if (Server_clientConnected()) {
            Server_genericRead();
        } else if (wlanConnected) {
            Server_waitForConnection();
        }
        break;
        
    case POST_UserCodeUpload:
        Server_uploadPOST();
        break;
        
    case Upload_Memory:
        Server_uploadMemory();
        break;
        
    case User_GET_Raw:
        Server_handleRaw(controlData.userGET.callback.raw);
        break;
    case User_POST_Raw:
        Server_handleRaw(controlData.userPOST.callback.raw);
        break;
        
    case User_POST:
        Server_userPOST();
        break;
        
	default:
		Server_genericRead();
		break;
    }
}



static void startUserCodeIfNeeded(uint32_t elapsedSeconds)
{
    if (systemStatus.state == RunState_UserExecuting)
        return;
    if (!Control_haveUserCode())
        return;
        
    switch (systemStatus.state) {
    case RunState_Initialize:
        Kernel_startUserCode();
        break;
    case RunState_ResetInitialize:
    case RunState_Setup:
    case RunState_UserExecuting:
    case RunState_UserHalted:
        break;
    case RunState_FailureHalted_SocketIO:
    case RunState_FailureHalted_SoftwareWatchdog:    
    case RunState_ResetHalted_LowPower:
    case RunState_ResetHalted_WindowWatchDog:
    case RunState_ResetHalted_IndependentWatchDog:
    case RunState_ResetHalted_SoftwareReset:    
    case RunState_Fault_NMI:
    case RunState_Fault_Hard:
    case RunState_Fault_Memory:
    case RunState_Fault_Bus:
    case RunState_Fault_Usage:
        if (systemStatus.detailed.userFaulted.retryTime <= 
                elapsedSeconds) {
            Kernel_startUserCode();
        } else {
            systemStatus.detailed.userFaulted.retryTime -= 
                elapsedSeconds;
        }
        break;
    }
}

void Kernel_setLED(bool on)
{
    if (on) {
        GPIO_BSRR(GPIOA) = GPIO3 << 16;
    } else {
        GPIO_BSRR(GPIOA) = GPIO3;
    }
}


static void blinkLED( uint32_t rate1, uint32_t rate2, uint32_t rateAlternate ) {
    uint32_t st = systick;
    if (rateAlternate == 0 || 
            ((st / rateAlternate) & 1)) {
        Kernel_setLED(((st / rate1) & 1) != 0);
    } else if (rate2 == 0) {
        Kernel_setLED(false);
    } else {
        Kernel_setLED(((st / rate2) & 1) != 0);
    }
}

static void updateLED(void)
{
    if (systemStatus.state == RunState_UserExecuting)
        return;
    switch (systemStatus.state) {
    case RunState_Setup:
        switch (controlState) {
        default:
            blinkLED(TICK_RATE/32, 0, 0);
            break;

        case Setup_Mode:
            blinkLED(TICK_RATE/32, TICK_RATE/2, TICK_RATE*4);
            break;
        case Setup_Connecting:
            blinkLED(TICK_RATE/32, TICK_RATE/2, TICK_RATE*8);
            break;
        }
        break;
    case RunState_UserExecuting:
        break;
    case RunState_UserHalted:
        if (Control_networkUp()) {
            blinkLED(TICK_RATE/2, 0, TICK_RATE*4);
        } else {
            blinkLED(TICK_RATE/16, 0, TICK_RATE*4);
        }
        break;
    case RunState_Initialize:
    case RunState_ResetInitialize:
        if (Control_networkUp()) {
            blinkLED(TICK_RATE/2, 0, 0);
        } else {
            blinkLED(TICK_RATE/16, 0, 0);
        }
        break;
    case RunState_FailureHalted_SocketIO:
    case RunState_FailureHalted_SoftwareWatchdog:    
    case RunState_ResetHalted_LowPower:
    case RunState_ResetHalted_WindowWatchDog:
    case RunState_ResetHalted_IndependentWatchDog:
    case RunState_ResetHalted_SoftwareReset:    
    case RunState_Fault_NMI:
    case RunState_Fault_Hard:
    case RunState_Fault_Memory:
    case RunState_Fault_Bus:
    case RunState_Fault_Usage:
        blinkLED(TICK_RATE/16, TICK_RATE/2, TICK_RATE*4);
        break;
    }
}


bool Control_networkUp(void)
{
    return wlanConnected && Server_socketOpen() && Transport_ready();
}

void Control_main(void)
{
    Kernel_enableWatchdog();
    
    savedConfigurationValid = 
        (_config.version == PERSISTENTCONFIGURATIONVERSION) &&
        (_config.crc == calculateConfigCRC(&_config));
    
    networkReset();
    if (systemStatus.state == RunState_ResetInitialize) {
        resetSystem();
    } else if (!haveNetworkConfiguration()) {
        systemStatus.state = RunState_Setup;
    }
    
    if (systemStatus.state == RunState_Setup) {
        Setup_enter();
    }
    
    uint32_t lastSystick = systick;
    controlStateStartTime = systick;
    activityDetectTime = systick;
    while (true) {
        Kernel_refreshWatchdog();
        
        uint32_t elapsedSeconds = systick;
        {
            uint32_t now = systick;
            if (now < lastSystick)
                elapsedSeconds = 0xFFFFFFFF - now + lastSystick;
            else
                elapsedSeconds = now - lastSystick;
            if (elapsedSeconds >= TICK_RATE) {
                elapsedSeconds /= TICK_RATE;
                lastSystick += elapsedSeconds * TICK_RATE;
            } else {
                elapsedSeconds = 0;
            }
        }
        
        startUserCodeIfNeeded(elapsedSeconds);
        updateLED();        
        systemProcess();
        
        /* Release faults over time in case they where spurious */
        if (systemStatus.state == RunState_UserExecuting) {
            if (systemStatus.detailed.normal.networkFaultCounter != 0) {
                if (Kernel_elapsed(systemStatus.detailed.normal.
                        lastNetworkFault) > 120 * TICK_RATE) {
                    systemStatus.detailed.normal.lastNetworkFault--;
                    systemStatus.detailed.normal.lastNetworkFault =
                        systick;
                }
            }
        }
        
        Transport_process();
    }
}


int User_nvmem_write(NetAPINVMemFileID file, const void *buffer, 
                     uint32_t length, uint32_t seek)
{
    if (file != FileID_User1 && file != FileID_User2)
        return -1;
    return nvmem_write(file, buffer, length, seek);
}
                
int User_nvmem_create(NetAPINVMemFileID file, uint32_t length)
{
    if (file != FileID_User1 && file != FileID_User2)
        return -1;
    return nvmem_create(file, length);
}
