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

#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/scs.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/iwdg.h>
#include <libopencm3/stm32/desig.h>
#include <libopencm3/stm32/f1/dma.h>
#include <libopencm3/stm32/f1/rcc.h>
#include <libopencm3/stm32/f1/flash.h>
#include <libopencm3/stm32/f1/gpio.h>

#include "kernel.h"
#include "init.h"

#define PIN_CONFIG(n, mode, conf)   (((mode) << (((n)%8)*4)) | ((conf) << (((n)%8)*4 + 2)))
#define PIN_CONFIG_MASK(n)          (0xF << (((n)%8)*4))

#define F_CPU   24000000
#define STMAX   ((uint32_t)(24000000 / TICK_RATE / 8 + 0.5))

static void setUsedClocks(void)
{
    RCC_AHBENR = RCC_AHBENR_DMA1EN;
    RCC_APB1ENR = 0;
    RCC_APB2ENR = RCC_APB2ENR_SPI1EN | RCC_APB2ENR_AFIOEN | 
        RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN;
}

bool Kernel_initializePeripherals(void)
{
    /* Disable and reset everything */
    RCC_AHBENR = 0;
    RCC_APB1ENR = 0;
    RCC_APB2ENR = 0;
    RCC_AHBRSTR = 0xFFFFFFFF;
    RCC_APB1RSTR = 0xFFFFFFFF;
    RCC_APB2RSTR = 0xFFFFFFFF;
    /* All device memory is strongly ordered, so no barrier */    
    RCC_AHBRSTR = 0;
    RCC_APB1RSTR = 0;
    RCC_APB2RSTR = 0;
    
    /* Start the HSI (and the LSI for the IWDG), then clock to 24 Mhz */
    RCC_CR |= RCC_CR_HSION;
    RCC_CSR |= RCC_CSR_LSION;
    while ((RCC_CR & RCC_CR_HSIRDY) == 0) { }
    /* First just switch to the HSI directly and set the prescalars we'll
     * need later */
    RCC_CFGR = RCC_CFGR_HPRE_SYSCLK_NODIV | RCC_CFGR_ADCPRE_PCLK2_DIV2 |
        RCC_CFGR_PPRE1_HCLK_NODIV | RCC_CFGR_PPRE2_HCLK_NODIV |
        RCC_CFGR_SW_SYSCLKSEL_HSICLK;
    /* Zero wait states needed */
    FLASH_ACR = FLASH_LATENCY_0WS | FLASH_PRFTBE | FLASH_HLFCYA;
    
    /*Switch to PLL */
    RCC_CFGR = RCC_CFGR_HPRE_SYSCLK_NODIV | RCC_CFGR_ADCPRE_PCLK2_DIV2 |
        RCC_CFGR_PPRE1_HCLK_NODIV | RCC_CFGR_PPRE2_HCLK_NODIV |
        RCC_CFGR_SW_SYSCLKSEL_HSICLK | 
        (RCC_CFGR_PLLMUL_PLL_CLK_MUL6 << 18) | 
        (RCC_CFGR_PLLSRC_HSI_CLK_DIV2 << 16);
    RCC_CR |= RCC_CR_PLLON;
    while ((RCC_CR & RCC_CR_PLLRDY) == 0) { }
    RCC_CFGR = RCC_CFGR_HPRE_SYSCLK_NODIV | RCC_CFGR_ADCPRE_PCLK2_DIV2 |
        RCC_CFGR_PPRE1_HCLK_NODIV | RCC_CFGR_PPRE2_HCLK_NODIV |
        RCC_CFGR_SW_SYSCLKSEL_PLLCLK |
        (RCC_CFGR_PLLMUL_PLL_CLK_MUL6 << 18) | 
        (RCC_CFGR_PLLSRC_HSI_CLK_DIV2 << 16);
    
    /* Enable the clocks to the components the kernel uses */
    setUsedClocks();
    
    GPIO_CRL(GPIOA) = 
        PIN_CONFIG(0, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT) |
        PIN_CONFIG(1, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT) |
        PIN_CONFIG(2, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT) |
        PIN_CONFIG(3, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN) |                /* Initialize the LED as an input for reset request detection */
        PIN_CONFIG(4, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT) |
        PIN_CONFIG(5, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT) |
        PIN_CONFIG(6, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT) |
        PIN_CONFIG(7, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT);
    GPIO_CRH(GPIOA) = 
        PIN_CONFIG(8, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT) |
        PIN_CONFIG(9, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT) |
        PIN_CONFIG(10, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT) |
        PIN_CONFIG(11, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT) |
        PIN_CONFIG(12, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT) |
        PIN_CONFIG(13, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT) |
        PIN_CONFIG(14, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL) |         /* Module enable */
        PIN_CONFIG(15, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT);                      /* IRQ */
    GPIO_CRL(GPIOB) = 
        PIN_CONFIG(0, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT) |
        PIN_CONFIG(1, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT) |
        PIN_CONFIG(2, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT) |
        PIN_CONFIG(3, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL) |    /* SCLK */
        PIN_CONFIG(4, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT) |                      /* MISO */
        PIN_CONFIG(5, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL) |    /* MOSI */
        PIN_CONFIG(6, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT) |
        PIN_CONFIG(7, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT);
    GPIO_CRH(GPIOB) = 
        PIN_CONFIG(8, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT) |
        PIN_CONFIG(9, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL) |          /* CS */
        PIN_CONFIG(10, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT) |
        PIN_CONFIG(11, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT) |
        PIN_CONFIG(12, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT) |
        PIN_CONFIG(13, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT) |
        PIN_CONFIG(14, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT) |
        PIN_CONFIG(15, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT);
    GPIO_CRH(GPIOC) = 
        PIN_CONFIG(8, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT) |
        PIN_CONFIG(9, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT) |
        PIN_CONFIG(10, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT) |
        PIN_CONFIG(11, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT) |
        PIN_CONFIG(12, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT) |
        PIN_CONFIG(13, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT) |
        PIN_CONFIG(14, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT) |
        PIN_CONFIG(15, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT);
    GPIO_CRL(GPIOD) = 
        PIN_CONFIG(0, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT) |
        PIN_CONFIG(1, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT) |
        PIN_CONFIG(2, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT) |
        PIN_CONFIG(3, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT) |
        PIN_CONFIG(4, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT) |
        PIN_CONFIG(5, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT) |
        PIN_CONFIG(6, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT) |
        PIN_CONFIG(7, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT);
        
    /* Pull LED up, and disable module */
    GPIO_ODR(GPIOA) = GPIO3;
    /* Disable CS */
    GPIO_ODR(GPIOB) = GPIO9;
    
    /* Setup SPI */
    SPI_CR1(SPI1) = SPI_CR1_MSTR | SPI_CR1_BAUDRATE_FPCLK_DIV_8 | 
            SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE |
            SPI_CR1_CPHA_CLK_TRANSITION_2 |
            SPI_CR1_DFF_8BIT | SPI_CR1_MSBFIRST | 
            SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_SPE;
    SPI_CR2(SPI1) = SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN;
    
    /* Tx DMA */
    DMA_CCR(DMA1, DMA_CHANNEL3) = 0;
	DMA_CNDTR(DMA1, DMA_CHANNEL3) = 0;
	DMA_CPAR(DMA1, DMA_CHANNEL3) = (uint32_t)(&SPI_DR(SPI1));
	DMA_CMAR(DMA1, DMA_CHANNEL3) = 0;
    
    /* Rx DMA */
    DMA_CCR(DMA1, DMA_CHANNEL2) = 0;
	DMA_CNDTR(DMA1, DMA_CHANNEL2) = 0;
	DMA_CPAR(DMA1, DMA_CHANNEL2) = (uint32_t)(&SPI_DR(SPI1));
	DMA_CMAR(DMA1, DMA_CHANNEL2) = 0;
    
    /* Set EXTI15 to PA15 for module IRQ */
    AFIO_EXTICR4 = 0;
    EXTI_RTSR = 0;
    EXTI_FTSR = 0;
    EXTI_IMR = EXTI15;
	EXTI_EMR = EXTI15;
    
    /* Remap PD0, since we're normally clocked internally */
    AFIO_MAPR = AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_OFF | AFIO_MAPR_SPI1_REMAP | 
        AFIO_MAPR_PD01_REMAP;
    
    /* Disable all interrupts */
    NVIC_ICER(0) = 0xFFFFFFFF;
    NVIC_ICER(1) = 0xFFFFFFFF;
    NVIC_ICER(2) = 0xFFFFFFFF;
    
    /* Enable kernel interrupts */
    NVIC_ISER(0) = (1<<NVIC_DMA1_CHANNEL2_IRQ);
    NVIC_ISER(1) = (1<<(NVIC_EXTI15_10_IRQ-32));
    
    /* Reset all interrupt priorities */
    for (int i=0; i<NVIC_IRQ_COUNT; i += 4)
        MMIO32(NVIC_BASE + 0x300 + i) = 0;
    for (int i=0; i<12; i += 4)
        MMIO32(SCS_BASE + 0xD18 + i) = 0;
        
    /* Enable fault handlers for these for better debugging and clear
     * any other bits. */
    SCB_SHCSR = SCB_SHCSR_USGFAULTENA | SCB_SHCSR_BUSFAULTENA | 
        SCB_SHCSR_MEMFAULTENA;
    /* Clear any old bits. */
    SCB_CFSR = SCB_CFSR_DIVBYZERO | SCB_CFSR_UNALIGNED | SCB_CFSR_NOCP | 
        SCB_CFSR_INVPC | SCB_CFSR_INVSTATE | SCB_CFSR_UNDEFINSTR;
    SCB_HFSR = SCB_HFSR_FORCED | SCB_HFSR_VECTTBL;
    
    /* Set all kernel interrupts to low priority; this also stops us
     * from having to deal with exception nesting in context switches. */
    NVIC_IPR(NVIC_DMA1_CHANNEL2_IRQ) = 0xFF;
    NVIC_IPR(NVIC_EXTI15_10_IRQ) = 0xFF;
    SCS_SHPR(11) = 0xFF;    /* Systick */
    SCS_SHPR(10) = 0xFF;    /* PendSV */
    
    /* Initially disabled */
    softwareWatchdog = SOFTWARE_WATCHDOG_MAGIC_DISABLED;
    /* Make sure LSI is ready for IWDG */
    while ((RCC_CSR & RCC_CSR_LSIRDY) == 0) { }
    
    /* Start the system timer */
    STK_LOAD = STMAX;
    STK_CTRL = STK_CTRL_CLKSOURCE_AHB_DIV8|STK_CTRL_TICKINT;
    STK_CTRL |= STK_CTRL_ENABLE;
    
    /* Have had enough time now so read the LED pin state */
    bool reset = ((GPIO_IDR(GPIOA) & GPIO3) == 0);    
    /* Change the LED to be an open drain output now */
    uint32_t reg = GPIO_CRL(GPIOA);
    reg &= ~PIN_CONFIG_MASK(3);
    reg |= PIN_CONFIG(3, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_OPENDRAIN);
    GPIO_CRL(GPIOA) = reg;
    return reset;
}

void Kernel_disableUserPeripherals(void)
{
    setUsedClocks();
    
    NVIC_ICER(0) = 0xFFFFFFFF & (~(1<<NVIC_DMA1_CHANNEL2_IRQ));
    NVIC_ICER(1) = 0xFFFFFFFF & (~(1<<(NVIC_EXTI15_10_IRQ-32)));
    NVIC_ICER(2) = 0xFFFFFFFF;
}

static void resetUART(void)
{
    RCC_APB2RSTR |= RCC_APB2RSTR_USART1RST;
    /* All device memory is strongly ordered, so no barrier */
    RCC_APB2RSTR &= ~RCC_APB2RSTR_USART1RST;
}

static void changeIWDG(uint32_t prescalar, uint32_t reload)
{
    while (IWDG_SR & IWDG_SR_PVU) { }
    IWDG_KR = IWDG_KR_UNLOCK;
    IWDG_PR = prescalar;
    while (IWDG_SR & IWDG_SR_RVU) { }
    IWDG_KR = IWDG_KR_UNLOCK;
    IWDG_RLR = reload;
    
    IWDG_KR = IWDG_KR_RESET;
    IWDG_KR = IWDG_KR_START;
}

void Kernel_disableWatchdog(void)
{
    /* Don't/can't actually stop it, so just set it to max (~26 seconds) */
    changeIWDG(IWDG_PR_DIV256, 0xFFE);
    softwareWatchdog = SOFTWARE_WATCHDOG_MAGIC_DISABLED;
}

void Kernel_enableWatchdog(void)
{
    /* ~ 3 seconds (since the module reset takes ~700ms) */
    changeIWDG(IWDG_PR_DIV32, 3750);
    softwareWatchdog = SOFTWARE_WATCHDOG_MAGIC_MAX;
}

void Kernel_refreshWatchdog(void)
{
    IWDG_KR = IWDG_KR_RESET;

    uint32_t wdt = softwareWatchdog;
    if (wdt <= SOFTWARE_WATCHDOG_MAGIC_MIN ||
            wdt > SOFTWARE_WATCHDOG_MAGIC_MAX) {
        Kernel_softwareUserFault(
            RunState_FailureHalted_SoftwareWatchdog);
    }
    softwareWatchdog = SOFTWARE_WATCHDOG_MAGIC_MAX;
}

/* Initialize the UART as an input */
void Kernel_initializeUART(void)
{
    uint32_t reg = GPIO_CRH(GPIOA);
    reg &= ~PIN_CONFIG_MASK(10);
    reg |= PIN_CONFIG(10, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT);
    GPIO_CRH(GPIOA) = reg;
    
    resetUART();
    RCC_APB2ENR |= RCC_APB2ENR_USART1EN;
    
    /* 9600 7E1 */
    USART_BRR(USART1) = ((2 * F_CPU) + 9600) / (2 * 9600);
    USART_CR2(USART1) = USART_CR2_STOPBITS_1;
    USART_CR3(USART1) = 0;
    USART_CR1(USART1) = USART_STOPBITS_1 | USART_CR1_PCE | USART_CR1_RE | 
        USART_CR1_UE;
}

/* Disable the UART */
void Kernel_disableUART(void)
{
    RCC_APB2ENR &= ~RCC_APB2ENR_USART1EN;    
    resetUART();
}

uint32_t Kernel_totalFlashSize(void)
{
    return (uint32_t)DESIG_FLASH_SIZE;
}

uint32_t Kernel_fractionalSeconds(void)
{
    bool interruptable = Kernel_interruptsEnabled();
    if (interruptable)
        disableInterrupts(); 
    uint32_t stf = STK_VAL;
    uint32_t st = systick;
    uint32_t stf2 = STK_VAL;
    if (interruptable)
        enableInterrupts();
        
    if (stf2 > stf) {
        ++st;
        stf = stf2;
    }
    stf = STK_VAL - stf;
    
    st *= (uint32_t)(65536 / TICK_RATE + 0.5);    
    stf *= (uint32_t)(65536 / TICK_RATE + 0.5);
    stf /= STMAX;    
    return st + stf;
}
