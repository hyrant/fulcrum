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

#include <libopencm3/stm32/f1/gpio.h>

#include "init.h"
#include "transport.h"
#include "kernel.h"
#include "netapi.h"
#include "data.h"

volatile uint32_t systick = 0;
void sys_tick_handler(void)
{
    systick++;
}
void pend_sv_handler(void) { }

void dma1_channel2_isr(void)
{
    Transport_irqRxDMA();
}

void faulted(void) { while(1) { } }

void hard_fault_handler(void) { faulted(); }
void mem_manage_handler(void) { faulted(); }
void bus_fault_handler(void) { faulted(); }
void usage_fault_handler(void) { faulted(); }

void exti15_10_isr(void)
{
    Transport_irqEXTI();
}

uint32_t Kernel_elapsed(uint32_t start)
{
    uint32_t now = systick;
    if (now < start)
        return 0xFFFFFFFF - now + start;
    return now - start;
}
void Kernel_msleep(uint32_t total)
{
    uint32_t start = systick;
    total *= TICK_RATE;
    total += TICK_RATE/2;
    total /= 1000;
    do {
        Thread_yield();
    } while (Kernel_elapsed(start) <= total);
}

bool Kernel_interruptsEnabled(void)
{
    uint32_t status;
    asm volatile (
        " MRS %0, PRIMASK \n" 
        : "=r" (status) );
    return (status & 1) == 0;
}

static void blinkLED( uint32_t rate1, uint32_t rate2, uint32_t rateAlternate ) {
    uint32_t st = systick;
    if (rateAlternate == 0 || 
            ((st / rateAlternate) & 1)) {
        if ((st / rate1) & 1) {
            GPIO_BSRR(GPIOA) = GPIO3 << 16;
        } else {
            GPIO_BSRR(GPIOA) = GPIO3;
        }
    } else if (rate2 == 0) {
        GPIO_BSRR(GPIOA) = GPIO3;
    } else {
        if ((st / rate2) & 1) {
            GPIO_BSRR(GPIOA) = GPIO3 << 16;
        } else {
            GPIO_BSRR(GPIOA) = GPIO3;
        }
    }
}

/* I'm not sure we actually need to do this 32 bytes at a time like the
 * TI driver does, but it doesn't really hurt either so be safe. */
static int writeChunked(NetAPINVMemFileID target, const void *data, 
                       uint16_t length)
{
    uint16_t offset = 0;
    while (length >= 32) {
        if (nvmem_write(target, data, 32, offset) != 0)
            return -1;
        offset += 32;
        data += 32;
        length -= 32;
    }
    if (length > 0) {
        if (nvmem_write(target, data, length, offset) != 0)
            return -1;
    }
    return 0;
}
/* Likewise I don't think the chunking is needed, but be safe anyway. */
static int readChunked(NetAPINVMemFileID source, void *data, 
                       uint16_t length)
{
    uint16_t offset = 0;
    while (length >= 8) {
        if (nvmem_read(source, data, 8, offset) <= 0)
            return -1;
        offset += 8;
        data += 8;
        length -= 8;
    }
    if (length > 0) {
        if (nvmem_read(source, data, length, offset) <= 0)
            return -1;
    }
    return 0;
}
static int writeFAT(const uint16_t *address, const uint16_t *length)
{
    uint8_t table[48];
    if (nvmem_write(FileID_MAX, "LS", 2, 0) != 0)
        return -1;        
    uint8_t *ptr = table;
    for (int i=0; i<=FileID_RM; i++) {
        *ptr++ = (uint8_t)(address[i] & 0xFF) | 1;
        *ptr++ = (uint8_t)(address[i] >> 8);
        *ptr++ = (uint8_t)(length[i] & 0xFF);
        *ptr++ = (uint8_t)(length[i] >> 8);
    }
    if (nvmem_write(FileID_MAX, table, 48, 4) != 0)
        return -1;
    memset(table, 0, 16);
    if (nvmem_write(FileID_MAX, table, 16, 52) != 0)
        return -1;
    return 0;
}

void Kernel_softwareUserFault(SystemRunState fault)
{
    (void)fault;
}

static enum {
    Initialize_DevicePatches = 0,
    
    Initialize_NoPatches,
    Patching_P1P5_DR,
    Patching_P1P5_FW,
    
    Initialize_P1P5,
    Patching_FAT,
    Initialize_FATPatched,
    Patching_RestoreSavedFiles,
    
    Patching_Driver,
    Patching_Firmware,
    
    Finalize,
    
    Completed,
} patchState;

void Thread_yield(void)
{
    switch (patchState) {
    case Initialize_DevicePatches:
    case Initialize_NoPatches:
        blinkLED(TICK_RATE/32, 0, 0);
        break;
    
    case Patching_P1P5_DR:
        blinkLED(TICK_RATE/32, TICK_RATE/8, TICK_RATE);
        break;
    case Patching_P1P5_FW:
        blinkLED(TICK_RATE/16, TICK_RATE/8, TICK_RATE);
        break;
    
    case Initialize_P1P5:
        blinkLED(TICK_RATE/16, 0, 0);
        break;    
    case Patching_FAT:
        blinkLED(TICK_RATE/8, TICK_RATE/2, TICK_RATE*2);
        break;
    case Initialize_FATPatched:
        blinkLED(TICK_RATE/8, 0, 0);
        break;        
    case Patching_RestoreSavedFiles:
        blinkLED(TICK_RATE/4, TICK_RATE/2, TICK_RATE*2);
        break;
    
    case Patching_Driver:
        blinkLED(TICK_RATE/4, TICK_RATE, TICK_RATE*4);
        break;
    case Patching_Firmware:
        blinkLED(TICK_RATE/2, TICK_RATE, TICK_RATE*4);
        break;
        
    case Finalize:
        blinkLED(TICK_RATE/16, 0, TICK_RATE/2);
        break;        
    case Completed:
        blinkLED(TICK_RATE/32, 0, TICK_RATE/2);
        break;
    }
}

static void waitReady(void)
{
    while (!Transport_ready()) {
        Transport_process();
        Thread_yield();
    }
}

static void haltCompleted(void)
{
    patchState = Completed;
    while (true) {
        Transport_process();
        Thread_yield();
    }
}

static void delayTime(uint32_t ticks)
{
    uint32_t st = systick;
    while (Kernel_elapsed(st) <= ticks) {
        Transport_process();
        Thread_yield();
    }
}

static uint8_t saveMAC[6];
static uint8_t saveRMParams[128];

volatile uint32_t softwareWatchdog = SOFTWARE_WATCHDOG_MAGIC_DISABLED;
int main(void)
{
    Kernel_initializePeripherals();
    
    Transport_init();
    do {
        uint32_t st = systick;
        while (!Transport_ready() && Kernel_elapsed(st) < 30 * TICK_RATE) {
            Transport_process();
            Thread_yield();
        }
        if (Kernel_elapsed(st) >= 30 * TICK_RATE)
            break;
        
        if (Data_version == ioctl_get_version()) {
            haltCompleted();
        }
        
        delayTime(2 * TICK_RATE);
        break;
    } while (true);
    
    patchState = Initialize_NoPatches;
    Transport_initPatched(Transport_NoPatches);
    waitReady();
    
    patchState = Patching_P1P5_DR;
    while (true) {
        if (writeChunked(FileID_DriverPatches, Data_wlanDriverP1P5, 
                Data_wlanDriverP1P5_Length) == 0)
            break;
    }
    patchState = Patching_P1P5_FW;
    while (true) {
        if (writeChunked(FileID_FirmwarePatches, Data_firmwareP1P5, 
                Data_firmwareP1P5_Length) == 0)
            break;
    }
    
    patchState = Initialize_P1P5;
    delayTime(1 * TICK_RATE);
    Transport_init();
    waitReady();
    
    patchState = Patching_FAT;
    bool haveMAC = (readChunked(FileID_MAC, saveMAC, 6) >= 0);
    bool haveRM = false;
    for (int t=0; t<3 && !haveRM; ++t) {
        haveRM = (readChunked(FileID_RM, saveRMParams, 128) >= 0);
    }
    const uint8_t *programRM;
    if (haveRM) {
        programRM = saveRMParams;
    } else {
        programRM = Data_cRMdefaultParams;
    }    
    while (true) {
        if (writeFAT(Data_FATEntriesAddress, Data_FATEntriesLength) == 0)
            break;
    }
    
    patchState = Initialize_FATPatched;
    delayTime(1 * TICK_RATE);
    /* Don't boot with patches as apparently there's something wrong we're
     * doing and it requests a firmware patch and doesn't accept it
     * when we give it none.  However, I think the only reason for
     * booting with 1.5 above was to re-write the FAT, so we don't actually
     * need it anymore. */
    Transport_initPatched(Transport_NoPatches);
    waitReady();
    
    patchState = Patching_RestoreSavedFiles;
    while (true) {
        if (writeChunked(FileID_RM, programRM, 128) == 0)
            break;
    }
    if (haveMAC) {
        saveMAC[0] &= 0xFE;
        while (true) {
            if (writeChunked(FileID_MAC, saveMAC, 6) == 0)
                break;
        }
    }
    
    patchState = Patching_Driver;
    while (true) {
        if (writeChunked(FileID_DriverPatches, 
                Data_wlanDriver, Data_wlanDriver_Length) == 0)
            break;
    }
    
    patchState = Patching_Firmware;
    while (true) {
        if (writeChunked(FileID_FirmwarePatches, 
                Data_firmware, Data_firmware_Length) == 0)
            break;
    }
    
    patchState = Finalize;
    delayTime(1 * TICK_RATE);
    Transport_init();
    waitReady();
    
    haltCompleted();
}
