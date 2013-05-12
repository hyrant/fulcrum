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

#include <fulcrum.h>

#include <libopencm3/stm32/f1/gpio.h>
#include <libopencm3/stm32/f1/rcc.h>

#include "led.h"
#include "adc.h"
#include "http.h"
#include "server.h"

extern u32 rcc_ppre1_frequency;
extern u32 rcc_ppre2_frequency;

int main(void)
{
    /* Set these since libopencm3 expects them and the kernel has already
     * set the clock. */
    rcc_ppre1_frequency = 24000000;
    rcc_ppre2_frequency = 24000000;
    
    LED_init();
    ADC_init();
    HTTP_init();
    Server_init();
}
