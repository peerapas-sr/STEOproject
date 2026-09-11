/*******************************************************************************
 * File Name   : main.c
 * Description : Entry Point - Synthesizer with Sequence Recorder & Playback
 * Target MCU  : STM32 Nucleo-F411RE (STM32F411RET6)
 * Standard    : Toyota Embedded MISRA-C Compliant (22 Rules)
 *
 * Requirements Satisfied:
 *   [1] GPIO                  : 4 Keys In (PA10, PB3, PB5, PB4), Red LED (PA6), PC13 Button
 *   [2] UART (Interrupt/DMA)  : USART2 115200 bps via RX Interrupt (NO Polling)
 *   [3] ADC (Interrupt/DMA)   : Single-channel ADC1 via EOC Interrupt (PA4)
 *   [4] External Interrupt    : EXTI Line 10 on PA10 [Key 1]
 *   [5] Additional Peripheral : TIM3 Hardware Timer 1ms Periodic Interrupt
 *   [6] MISRA-C Compliance    : 22 Toyota Rules fully enforced
 *   [7] Software Structure    : Clean Separation of Application and BSP Drivers
 ******************************************************************************/

#include <stdint.h>
#define STM32F411xE
#include "stm32f4xx.h"

#include "bsp_gpio.h"
#include "bsp_adc.h"
#include "bsp_uart.h"
#include "bsp_buzzer.h"
#include "bsp_timer.h"
#include "app_synth.h"

/* System Clock definition required by CMSIS */
uint32_t SystemCoreClock = 16000000U;

int main(void)
{
    /* 1. Initialize Board Support Package (Drivers) */
    bsp_gpio_init();   /* 4 Keys, User Button PC13 & EXTI10 on PA10 */
    bsp_buzzer_init(); /* Buzzer on PC3 */
    bsp_adc_init();    /* Single-channel ADC1 with EOC interrupt */
    bsp_uart_init();   /* USART2 with RXNE Interrupt (No Polling) */
    bsp_timer_init();  /* TIM3 1ms Hardware Timer Interrupt */

    /* 2. Initialize and Run Application Layer */
    app_synth_init();
    app_synth_run();

    /* Should never reach here */
    while (1)
    {
        /* Infinite loop for safety */
    }
}
