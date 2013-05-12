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

#include <libopencm3/stm32/f1/dma.h>
#include <libopencm3/stm32/f1/adc.h>
#include <libopencm3/stm32/f1/rcc.h>
#include <libopencm3/cm3/nvic.h>

#include <fulcrum.h>

#include "led.h"

/* Don't bother with a mutex as these are only written by one thread
 * and having them out of sync doesn't actually matter. */
static volatile uint16_t outputTemperature;
static volatile uint16_t outputReference;

#define IDX_TEMPERATURE     0
#define IDX_REFERENCE       1
#define N_SAMPLE_CHANNELS   2

#define N_AVG_SAMPLES       8
static uint16_t dmaADCBuffer[N_SAMPLE_CHANNELS * N_AVG_SAMPLES];

/* Just increment a counter every time we complete a scan so we don't
 * duplicate thread processing. */
static volatile uint32_t scanID;
void dma1_channel1_isr(void)
{
    DMA1_IFCR |= DMA_IFCR_CGIF1;
    scanID++;
}

/* Just a simple single pole low pass digital filter. */
#define FILTER_ALPHA        0.904837418035960 /* TC = 10 */
static uint16_t applySinglePoleLowPassFilter(uint32_t previous, uint32_t next)
{
    if (previous != 0xFFFF) {
        previous *= (uint32_t)(FILTER_ALPHA * 65536.0);
        next *= (uint32_t)((1.0-FILTER_ALPHA) * 65536.0);
        previous += next;
        previous >>= 16;
        return previous;
    } else {
        return next;
    }
}

/**
 * The internal ADC thread handler.  Does not return.
 */
static void adcThread(void *param)
{
    (void)param;
    
    adc_off(ADC1);

    adc_enable_scan_mode(ADC1);
    adc_disable_discontinuous_mode_regular(ADC1);
    adc_set_continuous_conversion_mode(ADC1);
    adc_disable_external_trigger_regular(ADC1);
    adc_set_right_aligned(ADC1);
    adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_13DOT5CYC);

    adc_power_on(ADC1);
    
    /* Supposedly Tstab is 1us, so this is massive overkill, a single yield
     * would probably be enough. */
    msleep(1);

    adc_reset_calibration(ADC1);
    
    /* Don't use libopencm3's version of adc_calibration() so we can yield
     * instead of doing a pure busy wait. */
    ADC_CR2(ADC1) |= ADC_CR2_CAL;
	while (ADC_CR2(ADC1) & ADC_CR2_CAL) { yield(); }
    
    adc_enable_temperature_sensor(ADC1);
    
    uint8_t sampleOrder[N_SAMPLE_CHANNELS];
    sampleOrder[IDX_TEMPERATURE] = 16;
    sampleOrder[IDX_REFERENCE] = 17;
    
    adc_set_regular_sequence(ADC1, N_SAMPLE_CHANNELS, sampleOrder);
    
    /* Read it out to clear anything out for DMA */
    sampleOrder[0] = ADC_DR(ADC1);
    
    dma_channel_reset(DMA1, 1);
    /* Set priority (high so we don't lose sync due to an overwrite) */
    dma_set_priority(DMA1, 1, DMA_CCR_PL_HIGH);
    dma_set_memory_size(DMA1, 1, DMA_CCR_MSIZE_16BIT);
    dma_set_peripheral_size(DMA1, 1, DMA_CCR_PSIZE_16BIT);
    dma_enable_memory_increment_mode(DMA1, 1);
    dma_set_read_from_peripheral(DMA1, 1);
    dma_set_peripheral_address(DMA1, 1, (u32)&ADC1_DR);
    dma_set_memory_address(DMA1, 1, (u32)(&dmaADCBuffer[0]));
    dma_set_number_of_data(DMA1, 1, N_SAMPLE_CHANNELS * N_AVG_SAMPLES);
    dma_enable_circular_mode(DMA1, 1);
    dma_enable_transfer_complete_interrupt(DMA1, 1);
    DMA1_IFCR |= DMA_IFCR_CGIF1;
    nvic_enable_irq(NVIC_DMA1_CHANNEL1_IRQ);
    dma_enable_channel(DMA1, 1);
    adc_enable_dma(ADC1);
    
    /* Start conversions */
    adc_start_conversion_direct(ADC1);
    
    uint32_t previousScanID = 0;
    while (true) {
        if (scanID == previousScanID) {
            yield();
            continue;
        }
        previousScanID = scanID;
        
        
        /* Don't actually care if we grab a partially written value from the
         * buffer as the access should still be consistent (since each half
         * word access should be atomic). */
         
        uint32_t raw[N_SAMPLE_CHANNELS];
        memset(raw, 0, sizeof(raw));
        for (int i=0; i<N_SAMPLE_CHANNELS * N_AVG_SAMPLES; i++) {
            raw[i%N_SAMPLE_CHANNELS] += dmaADCBuffer[i];
        }
        
        outputTemperature = applySinglePoleLowPassFilter(outputTemperature, 
            raw[IDX_TEMPERATURE] / N_AVG_SAMPLES);
        outputReference = applySinglePoleLowPassFilter(outputReference, 
            raw[IDX_REFERENCE] / N_AVG_SAMPLES);
    }
}

void ADC_get(uint16_t *temperature, uint16_t *reference)
{
    *temperature = outputTemperature;
    *reference = outputReference;
}

extern unsigned _stack;
void ADC_init(void)
{
    outputTemperature = 0xFFFF;
    outputReference = 0xFFFF;
    
    /* Set things that operate on common registers here, before we start
     * the thread. */
    rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_ADC1EN);
    /* DMA1 is already enabled by the kernel */
    
    thread_start(adcThread, NULL, (uint8_t *)&_stack - 1024 - 256);
}
