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
#include <stdlib.h>

#include <libopencm3/stm32/f1/gpio.h>
#include <libopencm3/stm32/f1/rcc.h>

#include "gpio.h"

typedef struct {
    uint32_t port;
    uint16_t pin;
    const char *name;
} GPIOData;
#define GPIO_NC { .port = 0, .pin = 0, .name = NULL }

static GPIOData data[16] = {
    /* GPIO0, NC */     GPIO_NC,
    /* GPIO1, PIN 9 */  { .port=GPIOD, .pin=GPIO0, .name="PD0(Pin 9)" },
    /* GPIO2, PIN 19 */ { .port=GPIOB, .pin=GPIO10, .name="PB10(Pin 19)" },
    /* GPIO3, PIN 18 */ { .port=GPIOA, .pin=GPIO8, .name="PA8(Pin 18)" },
    /* GPIO4, PIN 13 */ { .port=GPIOB, .pin=GPIO13, .name="PB13(Pin 13)" },
    /* GPIO5, PIN 6 */  { .port=GPIOC, .pin=GPIO13, .name="PC13(Pin 6)" },
    /* GPIO6, PIN 15 */ { .port=GPIOB, .pin=GPIO15, .name="PB15(Pin 15)" },
    /* GPIO7, PIN 7 */  { .port=GPIOB, .pin=GPIO15, .name="PC14(Pin 7)" },
    /* GPIO8, PIN 4 */  { .port=GPIOA, .pin=GPIO5, .name="PA5(Pin 4)" },
    /* GPIO9, PIN 8 */  { .port=GPIOC, .pin=GPIO15, .name="PC15(Pin 8)" },
    /* GPIO10, TX */    GPIO_NC,
    /* GPIO11, RX */    GPIO_NC,
    /* GPIO12, CTS */   GPIO_NC,
    /* GPIO13, RTS */   GPIO_NC,
    /* GPIO14, PIN 11 */{ .port=GPIOB, .pin=GPIO12, .name="PB12(Pin 11)" },
    /* GPIO15, NC */    GPIO_NC,
};

void GPIO_setMode(uint16_t mask, bool output)
{
    uint16_t check = 1;
    for (int i=0; i<16; i++, check <<= 1) {
        if (!(mask & check))
            continue;
        uint32_t port = data[i].port;
        uint16_t pin = data[i].pin;
        if (!port || !pin)
            continue;
        
        if (output) {
            gpio_set_mode(port, GPIO_MODE_OUTPUT_50_MHZ,
                GPIO_CNF_OUTPUT_PUSHPULL, pin);
        } else {
            gpio_set_mode(port, GPIO_MODE_OUTPUT_50_MHZ,
                GPIO_CNF_INPUT_FLOAT, pin);
        }
    }
}

void GPIO_setOutput(uint16_t mask, uint16_t bits)
{
    uint16_t check = 1;
    for (int i=0; i<16; i++, check <<= 1) {
        if (!(mask & check))
            continue;
        uint32_t port = data[i].port;
        uint32_t pin = data[i].pin;
        if (!port || !pin)
            continue;
            
        if (!(bits & mask))
            pin <<= 16;
        
        GPIO_BSRR(port) = pin;
    }
}

const char *GPIO_getName(int index)
{
    if (index < 0 || index > 15)
        return  NULL;
    return data[index].name;
}

void GPIO_init(void)
{
    rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPAEN);
    rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPBEN);
    rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPCEN);
    rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPDEN);
}
