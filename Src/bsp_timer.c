/*******************************************************************************
 * File Name   : bsp_timer.c
 * Description : Board Support Package - TIM3 Hardware Timer Driver Implementation
 * Target MCU  : STM32F411RET6
 * Standard    : Toyota Embedded MISRA-C Compliant (22 Rules)
 ******************************************************************************/

#include "bsp_timer.h"
#define STM32F411xE
#include "stm32f4xx.h"

/* Named Constants (Rule 5 & Rule 10) */
#define TIM3_PRESCALER_1MHZ     (15U)
#define TIM3_ARR_1MS            (999U)
#define TIM3_NVIC_PRIORITY      (3U)

/* Global millisecond tick counter updated by TIM3 Interrupt */
static volatile uint32_t g_u4t_system_ms = 0U;

void bsp_timer_init(void)
{
    /* 1. Enable TIM3 Clock on APB1 */
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    /* 2. Configure TIM3 for 1 kHz (1 ms period) at 16 MHz clock:
     *    Prescaler = 16 - 1 -> Timer clock = 1 MHz (1 tick = 1 us)
     *    Auto-reload = 1000 - 1 -> Overflow period = 1000 us = 1 ms
     */
    TIM3->PSC = TIM3_PRESCALER_1MHZ;
    TIM3->ARR = TIM3_ARR_1MS;

    /* 3. Enable Update Interrupt */
    TIM3->DIER |= TIM_DIER_UIE;

    /* 4. Configure NVIC for TIM3 */
    NVIC_SetPriority(TIM3_IRQn, TIM3_NVIC_PRIORITY);
    NVIC_EnableIRQ(TIM3_IRQn);

    /* 5. Start TIM3 Counter */
    TIM3->CR1 |= TIM_CR1_CEN;
}

uint32_t bsp_timer_get_ms(void)
{
    return g_u4t_system_ms;
}

void bsp_timer_delay_ms(uint32_t ms)
{
    uint32_t u4t_start = g_u4t_system_ms;
    while ((g_u4t_system_ms - u4t_start) < ms)
    {
        __NOP();
    }
}

/* TIM3 Interrupt Service Routine */
void TIM3_IRQHandler(void)
{
    if ((TIM3->SR & TIM_SR_UIF) != 0U)
    {
        TIM3->SR &= ~TIM_SR_UIF; /* Clear interrupt flag */
        g_u4t_system_ms++;
    }
    else
    {
        /* Other timer event */
    }
}
