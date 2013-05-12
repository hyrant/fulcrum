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

#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/f1/gpio.h>
#include <libopencm3/stm32/f1/rcc.h>

#include <fulcrum.h>

#include "led.h"

static Mutex mutex;
static LED_MODE ledMode;
static uint16_t ledUp;
static uint16_t ledDown;

/**
 * The internal LED thread handler.  Does not return.
 */
static void ledThread(void *param)
{
    (void)param;
    
    /* Setup the LED for PWM output (zero is completely on) */
    timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    timer_set_prescaler(TIM2, 1);
    timer_enable_preload(TIM2);
    timer_continuous_mode(TIM2);
    timer_set_period(TIM2, 32768);
    timer_set_oc_mode(TIM2, TIM_OC4, TIM_OCM_PWM1);
    timer_set_oc_value(TIM2, TIM_OC4, 32768);
    timer_enable_oc_output(TIM2, TIM_OC4);    
    timer_enable_counter(TIM2);
    
    bool direction = true;
    uint32_t last = time_fractional();
    while (true) {
        /* Grab synchronized parameters. */  
        mutex_lock(&mutex);
        LED_MODE mode = ledMode;
        uint32_t up = ledUp;
        uint32_t down = ledDown;
        mutex_unlock(&mutex);
        
        switch (mode) {
        case LED_BLINK:
            /* Just sleep and go around again */
            if (direction) {
                timer_set_oc_value(TIM2, TIM_OC4, 0);
                msleep(up);
                direction = false;
            } else {
                timer_set_oc_value(TIM2, TIM_OC4, 32768);
                msleep(down);
                direction = true;
            }
            break;
            
        case LED_FADE: {
            /* Calculate the elapsed time and figure out how much of the
             * fraction that puts us at, then go around again. */
            uint32_t elapsed = time_fractional_elapsed(last);
            
            if (direction) {
                up <<= 16;
                up /= 1000;
                if (elapsed >= up) {
                    direction = false;
                    timer_set_oc_value(TIM2, TIM_OC4, 0);
                    last = time_fractional();
                } else {
                    elapsed *= 32768;
                    elapsed /= up;
                    timer_set_oc_value(TIM2, TIM_OC4, 32768 - elapsed);
                }
            } else {
                down <<= 16;
                down /= 1000;
                if (elapsed >= down) {
                    direction = true;
                    timer_set_oc_value(TIM2, TIM_OC4, 32768);
                    last = time_fractional();
                } else {
                    elapsed *= 32768;
                    elapsed /= down;
                    timer_set_oc_value(TIM2, TIM_OC4, elapsed);
                }
            }
            
            yield();
            break;
        }
        }
    }
}

void LED_setMode(LED_MODE mode, uint16_t up, uint16_t down)
{
    mutex_lock(&mutex);
    ledMode = mode;
    ledUp = up;
    ledDown = down;
    mutex_unlock(&mutex);
}

void LED_getMode(LED_MODE *mode, uint16_t *up, uint16_t *down)
{
    mutex_lock(&mutex);
    *mode = ledMode;
    *up = ledUp;
    *down = ledDown;
    mutex_unlock(&mutex);
}

extern unsigned _stack;
void LED_init(void)
{
    mutex_init(&mutex);
    ledMode = LED_FADE;
    ledUp = 900;
    ledDown = 100;
    
    /* Set things that operate on common registers here, before we start
     * the thread. */
    rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_TIM2EN);
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
		GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN, GPIO_TIM2_CH4);
    timer_reset(TIM2);
    
    thread_start(ledThread, NULL, (uint8_t *)&_stack - 1024);
}
