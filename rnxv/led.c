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

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/f1/rcc.h>

#include <fulcrum.h>

static volatile int blinkInProgress;

void tim2_isr(void)
{
    timer_clear_flag(TIM2, TIM_SR_UIF);
        
    if (++blinkInProgress > 2) {
        blinkInProgress = 0;
    } else {
        set_led(false);
        TIM2_CNT = 0;
        timer_enable_counter(TIM2);
    }
}

void LED_blink(void)
{
    if (blinkInProgress)
        return;
    blinkInProgress = 1;
    
    set_led(true);
    TIM2_CNT = 0;
    timer_clear_flag(TIM2, TIM_SR_UIF);
    timer_enable_counter(TIM2);
}

void LED_init(void)
{
    rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_TIM2EN);
    
    set_led(false);
     
    timer_reset(TIM2);
    timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    timer_set_prescaler(TIM2, 256);
    timer_enable_preload(TIM2);
    timer_one_shot_mode(TIM2);
    timer_set_period(TIM2, (uint16_t)((rcc_ppre1_frequency/256) / 20));
    timer_enable_irq(TIM2, TIM_DIER_UIE);
    timer_clear_flag(TIM2, TIM_SR_UIF);
    
    nvic_enable_irq(NVIC_TIM2_IRQ);
    nvic_set_priority(NVIC_TIM2_IRQ, 128);
}
