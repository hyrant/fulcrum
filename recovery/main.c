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
#include <string.h>

#include <libopencm3/stm32/f1/rcc.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/f1/gpio.h>

#include "eeprom.h"
#include "data.h"

#ifndef MAC
#define MAC "\x08\x00\x28\x01\x73\xFA"
#endif

enum {
    FileID_NVS = 0,
    FileID_NVS_Shadow,
    FileID_WLANConfig,
    FileID_WLANConfig_Shadow,
    FileID_DriverPatches,
    FileID_FirmwarePatches,
    FileID_MAC,
    FileID_FrontendVariables,
    FileID_IPConfig,
    FileID_IPConfig_Shadow,
    FileID_BootloaderPatches,
    FileID_RM,
    FileID_AESKey,
    FileID_SmartConfigShared,
    FileID_User1,
    FileID_User2,
    
    FileID_MAX,
};

static void writeFAT(void)
{
    #pragma pack(push,1)
    struct {
        uint8_t magic1;
        uint8_t magic2;
        uint8_t padding[2];
        struct {
            uint16_t address;
            uint16_t length;
        } table[16];
    } fat;
    #pragma pack(pop)
    
    memset(&fat, 0, sizeof(fat));
    
    fat.magic1 = 'L';
    fat.magic2 = 'S';    
    for (int i=0; i<FileID_RM+1; i++) {
        fat.table[i].address = Data_FATEntriesAddress[i] | 1;
        fat.table[i].length = Data_FATEntriesLength[i];
    }
    
    eeprom_write(0, &fat, sizeof(fat));
}

static void clocks_init(void)
{
    rcc_clock_setup_in_hsi_out_24mhz();
    
    rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPBEN);
    rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_AFIOEN);
    rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_I2C1EN);
}

static void i2c_init(void)
{
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
        GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN, 
        GPIO_I2C1_SCL);
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
        GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN, 
        GPIO_I2C1_SDA);
    
    i2c_reset(I2C1);
    I2C_CR1(I2C1) = I2C_CR1_SWRST;
    asm volatile ("nop");
    I2C_CR1(I2C1) = 0;
    i2c_peripheral_disable(I2C1);
    i2c_set_clock_frequency(I2C1, I2C_CR2_FREQ_24MHZ);
    i2c_set_ccr(I2C1, 120);
    i2c_set_trise(I2C1, 25);
    i2c_set_standard_mode(I2C1);
    i2c_peripheral_enable(I2C1);
}

int main(void)
{
    clocks_init();
    i2c_init();
    
    writeFAT();
    
    /* Magic constants are from 
     * http://processors.wiki.ti.com/index.php/CC3000_Flashing_Guide */
    /*eeprom_zero(Data_FATEntriesAddress[FileID_NVS], 0x1A);
    eeprom_zero(Data_FATEntriesAddress[FileID_NVS_Shadow], 0x1A);
    eeprom_zero(Data_FATEntriesAddress[FileID_WLANConfig], 0x64);
    eeprom_zero(Data_FATEntriesAddress[FileID_WLANConfig_Shadow], 0x64);
    eeprom_zero(Data_FATEntriesAddress[FileID_FrontendVariables], 0x10);
    eeprom_zero(Data_FATEntriesAddress[FileID_IPConfig], 0x40);
    eeprom_zero(Data_FATEntriesAddress[FileID_IPConfig_Shadow], 0x40);
    eeprom_zero(Data_FATEntriesAddress[FileID_BootloaderPatches], 0x400);*/
    
    eeprom_write(Data_FATEntriesAddress[FileID_DriverPatches], 
        Data_wlanDriver, Data_wlanDriver_Length);
    eeprom_write(Data_FATEntriesAddress[FileID_FirmwarePatches], 
        Data_firmware, Data_firmware_Length);
    eeprom_write(Data_FATEntriesAddress[FileID_MAC], 
        MAC, 6);
    eeprom_write(Data_FATEntriesAddress[FileID_RM], 
        Data_cRMdefaultParams, 128);
        
    while (true) { }    
    return 0;
}
