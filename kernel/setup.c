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

#include <libopencm3/stm32/usart.h>

#include "kernel.h"
#include "control.h"
#include "netapi.h"
#include "transport.h"
#include "setup.h"
#include "util.h"
#include "init.h"

void Transport_simpleConfigDone(void)
{
    if (systemStatus.state == RunState_Setup)
        systemStatus.detailed.setup.simpleConfigDone = true;
    if (controlState == Setup_Mode)
        controlState = Setup_SimpleConfigDone;
}

static char *serialBufferStartsWith(const char *token) 
{
    char *result = Util_bufferStartsWith(token, controlData.base.buffer.data);
    if (result == NULL)
        return result;
    if (!(*result))
        return NULL;
    return result;
}

static bool processSerialBuffer(void)
{
    char *end = Util_findCharacter(controlData.base.buffer.data, 
        '|', '\r', '\n');
    if (end == NULL)
        return false;
    bool isNewline = (*end != '|');
    *end = 0;
    
    const char *parse;    
    if ((parse = serialBufferStartsWith("IP:")) != NULL) {        
        systemStatus.detailed.setup.ip = Util_parseIP(parse);
    } else if ((parse = serialBufferStartsWith("NETMASK:")) != NULL ||
            (parse = serialBufferStartsWith("SUBNET:")) != NULL) {        
        systemStatus.detailed.setup.ipData.fixed.netmask = Util_parseIP(parse);
    } else if ((parse = serialBufferStartsWith("GATEWAY:")) != NULL ||
            (parse = serialBufferStartsWith("GW:")) != NULL) {        
        systemStatus.detailed.setup.ipData.fixed.gateway = Util_parseIP(parse);
    } else if ((parse = serialBufferStartsWith("DNS:")) != NULL) {        
        systemStatus.detailed.setup.ipData.fixed.dns = Util_parseIP(parse);
    } else if ((parse = serialBufferStartsWith("DHCP:")) != NULL) { 
        systemStatus.detailed.setup.ip.s_addr = 0;
        memset(&systemStatus.detailed.setup.ipData, 0,
            sizeof(systemStatus.detailed.setup.ipData));
        systemStatus.detailed.setup.ipData.dhcp.leaseTimeout = 
            Util_parseDecimal(parse);
    } else if ((parse = serialBufferStartsWith("DHCP")) != NULL) { 
        systemStatus.detailed.setup.ip.s_addr = 0;
        memset(&systemStatus.detailed.setup.ipData, 0,
            sizeof(systemStatus.detailed.setup.ipData));
    } else if ((parse = serialBufferStartsWith("SSID:")) != NULL) {
        strncpy(systemStatus.detailed.setup.ssid, parse, 32);
        systemStatus.detailed.setup.ssid[32] = 0;
    } else if ((parse = serialBufferStartsWith("BSSID:")) != NULL ||
            (parse = serialBufferStartsWith("MAC:")) != NULL) {
        Util_parseHex(parse, systemStatus.detailed.setup.bssid, 6);
    } else if ((parse = serialBufferStartsWith("OPEN")) != NULL) {
        systemStatus.detailed.setup.securityMode = Setup_Security_Open;
    } else if ((parse = serialBufferStartsWith("WPA2:")) != NULL) {
        systemStatus.detailed.setup.securityMode = Setup_Security_WPA2;
        strncpy(systemStatus.detailed.setup.securityData.passphrase, parse, 32);
        systemStatus.detailed.setup.securityData.passphrase[32] = 0;
    } else if ((parse = serialBufferStartsWith("WPA:")) != NULL) {
        systemStatus.detailed.setup.securityMode = Setup_Security_WPA;
        strncpy(systemStatus.detailed.setup.securityData.passphrase, parse, 32);
        systemStatus.detailed.setup.securityData.passphrase[32] = 0;
    } else if ((parse = serialBufferStartsWith("WEP64:")) != NULL) {
        systemStatus.detailed.setup.securityMode = Setup_Security_WEP64;
        Util_parseHex(parse, systemStatus.detailed.setup.securityData.
            wepKey, 5);
    } else if ((parse = serialBufferStartsWith("WEP128:")) != NULL) {
        systemStatus.detailed.setup.securityMode = Setup_Security_WEP128;
        Util_parseHex(parse, systemStatus.detailed.setup.securityData.
            wepKey, 13);
    } else if ((parse = serialBufferStartsWith("AUTH:")) != NULL ||
            (parse = serialBufferStartsWith("AUTHORIZATION:")) != NULL) {        
        Control_setAuthorization(parse);
    }
    
        
    if (isNewline) {
        /* Deleting profiles takes 0.5 seconds and adding one can take upwards
         * of 7 seconds (key calculation I guess), so turn this off while
         * we modify it. */
        Kernel_disableWatchdog();
        
        for (int i=0; i<7; i++) {
            ioctl_delete_profile(i);
        }

        if (systemStatus.detailed.setup.ip.s_addr == 0) {
            ioctl_network_dhcp();
            if (systemStatus.detailed.setup.ipData.dhcp.leaseTimeout != 0) {
                ioctl_set_timeouts(systemStatus.detailed.setup.ipData.dhcp.
                    leaseTimeout, 600, 20, 60);
            }
        } else {
            ioctl_network_set(systemStatus.detailed.setup.ip,
                systemStatus.detailed.setup.ipData.fixed.netmask,
                systemStatus.detailed.setup.ipData.fixed.gateway,
                systemStatus.detailed.setup.ipData.fixed.dns);
        }

        switch (systemStatus.detailed.setup.securityMode) {
        case Setup_Security_Open:
            ioctl_add_profile_open(1,
                systemStatus.detailed.setup.ssid,
                systemStatus.detailed.setup.bssid);
            break;
        case Setup_Security_WPA2:
            ioctl_add_profile_wpa2(1,
                systemStatus.detailed.setup.ssid,
                systemStatus.detailed.setup.bssid,
                systemStatus.detailed.setup.securityData.passphrase);
            break;
        case Setup_Security_WPA:
            ioctl_add_profile_wpa(1,
                systemStatus.detailed.setup.ssid,
                systemStatus.detailed.setup.bssid,
                systemStatus.detailed.setup.securityData.passphrase);
            break;
        case Setup_Security_WEP64:
            ioctl_add_profile_wep(1,
                systemStatus.detailed.setup.ssid,
                systemStatus.detailed.setup.bssid,
                5, systemStatus.detailed.setup.securityData.wepKey);
            break;
        case Setup_Security_WEP128:
            ioctl_add_profile_wep(1,
                systemStatus.detailed.setup.ssid,
                systemStatus.detailed.setup.bssid,
                13, systemStatus.detailed.setup.securityData.wepKey);
            break;
        }
        
        ioctl_set_connection_policy(true, true, false);
        
        Kernel_enableWatchdog();
        
        controlState = Setup_Apply;
        return false;
    } else {
        controlData.base.buffer.length = Util_shiftConsumeBuffer(
            controlData.base.buffer.data, end+1,
            controlData.base.buffer.length);
        return true;
    }
}

void Setup_process(void)
{
    if ((USART_SR(USART1) & USART_SR_RXNE)) {
        char add = (USART_DR(USART1) & 0x7F);   /* Mask off parity bit */
        
        /* Filter breaks */
        if (add != 0) {
            if (controlData.base.buffer.length < 
                    sizeof(controlData.base.buffer.data)-1) {
                controlData.base.buffer.data[controlData.base.buffer.length++] = 
                    add;
            } else {
                for (size_t i=1; i<sizeof(controlData.base.buffer.data)-1;
                        i++) {
                    controlData.base.buffer.data[i-1] =
                            controlData.base.buffer.data[i];
                }
                controlData.base.buffer.data[
                    sizeof(controlData.base.buffer.data)-2] = add;
            }
            
            while (processSerialBuffer()) { }
        }
    }
}

void Setup_enter(void)
{
    /* This can never be called while user code is executing (only every
     * called in a startup state), so we don't need to do this.  This also
     * blocks the transport layer, which we don't want to do just yet. */
    /*Kernel_haltUserCode();*/
    systemStatus.state = RunState_Setup;
    
    Kernel_initializeUART();
    
    memset(&systemStatus.detailed.setup, 0, 
        sizeof(systemStatus.detailed.setup));
    memset(&controlData, 0, sizeof(controlData));
    controlState = Setup_ModeEnter;
}

void Setup_leave(void)
{
    Kernel_disableUART();
        
    systemStatus.state = RunState_Initialize;
    memset(&controlData, 0, sizeof(controlData));
    controlState = Idle;
}
