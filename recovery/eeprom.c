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

#include <libopencm3/stm32/i2c.h>

#define ADDR    0x50

static void begin(bool write)
{
    while (I2C_SR2(I2C1) & I2C_SR2_BUSY) { }
    i2c_send_start(I2C1);
    while (!(I2C_SR1(I2C1) & I2C_SR1_SB)) { }
    i2c_send_7bit_address(I2C1, ADDR, write ? I2C_WRITE : I2C_READ);
    while (!(I2C_SR1(I2C1) & I2C_SR1_ADDR)) { }
    { uint8_t discard = I2C_SR2(I2C1); (void)discard; }
}

static void write(uint8_t byte)
{
    while (!(I2C_SR1(I2C1) & I2C_SR1_TxE)) { }
    I2C_DR(I2C1) = byte;
}

static uint8_t read(bool more)
{
    if (!more) {
        uint32_t data = I2C_CR1(I2C1);
        data &= ~I2C_CR1_ACK;
        data |= I2C_CR1_STOP;
        I2C_CR1(I2C1) = data;
    } else {
        I2C_CR1(I2C1) |= I2C_CR1_ACK;
    }
    while (!(I2C_SR1(I2C1) & I2C_SR1_RxNE)) { }
    return I2C_DR(I2C1) & 0xFF;
}

static void stop(void)
{
    while (I2C_CR1(I2C1) & (I2C_CR1_START)) { }
    while ((I2C_SR1(I2C1) & (I2C_SR1_TxE|I2C_SR1_BTF)) != 
        (I2C_SR1_TxE|I2C_SR1_BTF)) { }
    i2c_send_stop(I2C1);
    while (I2C_SR1(I2C1) & (I2C_SR1_TxE|I2C_SR1_BTF)) { }
    while (I2C_CR1(I2C1) & (I2C_CR1_STOP)) { }
}

void eeprom_read(uint16_t address, void *data, uint16_t length)
{
    if (length == 0)
        return;
    begin(true);
    write((address >> 8) & 0xFF);
    write(address & 0xFF);
    stop();
    
    begin(false);
    
    uint8_t *target = data;
    while (length != 0) {
        *target = read(length != 1);
        --length;
        ++target;
    }
}

void eeprom_write(uint16_t address, const void *data, uint16_t length)
{
    const uint8_t *source = data;
    while (length != 0) {
        /* Just delay for it to finish writing */
        for (int i=0; i<100000; i++) { asm volatile ("nop"); }
        begin(true);
        write((address >> 8) & 0xFF);
        write(address & 0xFF);
        while (true) {
            write(*source);            
            ++source;
            ++address;
            --length;
            /* Done or at a page boundary */
            if (address % 32 == 0 || length == 0) {
                stop();
                break;
            }
        }
    }
}

void eeprom_zero(uint16_t address, uint16_t length)
{
    while (length != 0) {
        /* Just delay for it to finish writing */
        for (int i=0; i<100000; i++) { asm volatile ("nop"); }
        begin(true);
        write((address >> 8) & 0xFF);
        write(address & 0xFF);
        while (true) {
            write(0);
            ++address;
            --length;
            /* Done or at a page boundary */
            if (address % 32 == 0 || length == 0) {
                stop();
                break;
            }
        }
    }
}
