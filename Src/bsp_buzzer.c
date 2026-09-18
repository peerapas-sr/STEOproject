/*******************************************************************************
 * File Name   : bsp_buzzer.c
 * Description : Board Support Package - TIM4 Hardware Timer Continuous Buzzer (PC3)
 * Target MCU  : STM32F411RET6 (Nucleo-F411RE)
 * Standard    : Toyota Embedded MISRA-C Compliant (22 Rules)
 ******************************************************************************/

#include "bsp_buzzer.h"
#include <stdbool.h>
#define STM32F411xE
#include "stm32f4xx.h"

/* Named Constants (Rule 5 & Rule 10) */
#define BUZZER_PIN              (3UL)      /* PC3 */
#define BUZZER_VOL_MIN_THRESH   (80U)
#define SEC_TO_US_FACTOR        (1000000U)
#define ADC_MAX_VAL             (4095U)
#define BUZZER_FREQ_MIN_HZ      (50U)
#define BUZZER_FREQ_MAX_HZ      (5000U)
#define TIM4_PRESCALER_1MHZ     (15U)      /* 16MHz / (15 + 1) = 1 MHz (1 tick = 1 us) */
#define TIM4_NVIC_PRIORITY      (1U)       /* High priority to eliminate audio jitter */
#define DELAY_US_CALIB_COUNT    (1U)
#define DELAY_MS_CALIB_COUNT    (2000U)

/* Hardware Timer Wave Generation State */
static volatile uint16_t g_u2t_buzzer_high_ticks = 250U;
static volatile uint16_t g_u2t_buzzer_low_ticks = 250U;
static volatile bool     g_b_buzzer_pin_state = false;
static volatile bool     g_b_buzzer_is_active = false;

/* Precise software delay loops */
void bsp_delay_us(uint32_t us)
{
    uint32_t u4t_us_rem = us;

    while (u4t_us_rem > 0U)
    {
        for (volatile uint32_t u4t_i = 0U; u4t_i < DELAY_US_CALIB_COUNT; u4t_i++)
        {
            __NOP();
        }
        u4t_us_rem--;
    }
}

void bsp_delay_ms(uint32_t ms)
{
    uint32_t u4t_ms_rem = ms;

    while (u4t_ms_rem > 0U)
    {
        for (volatile uint32_t u4t_i = 0U; u4t_i < DELAY_MS_CALIB_COUNT; u4t_i++)
        {
            __NOP();
        }
        u4t_ms_rem--;
    }
}

void bsp_buzzer_init(void)
{
    /* 1. Enable GPIOC and TIM4 Clocks */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;

    /* 2. Configure PC3 as Output Push-Pull, High Speed */
    GPIOC->MODER &= ~(3UL << (BUZZER_PIN * 2U));
    GPIOC->MODER |=  (1UL << (BUZZER_PIN * 2U)); /* Output mode */
    GPIOC->OTYPER &= ~(1UL << BUZZER_PIN);        /* Push-Pull */
    GPIOC->OSPEEDR |= (3UL << (BUZZER_PIN * 2U)); /* High Speed */
    GPIOC->PUPDR &= ~(3UL << (BUZZER_PIN * 2U));  /* No-pull */
    GPIOC->BSRR = (1UL << (BUZZER_PIN + 16U));    /* Initial LOW */

    /* 3. Configure TIM4 for 1 MHz tick rate (1 us per tick) */
    TIM4->PSC = TIM4_PRESCALER_1MHZ;
    TIM4->CR1 = TIM_CR1_ARPE; /* Auto-reload preload enable */
    TIM4->DIER |= TIM_DIER_UIE;

    /* 4. Configure NVIC for TIM4 with High Priority */
    NVIC_SetPriority(TIM4_IRQn, TIM4_NVIC_PRIORITY);
    NVIC_EnableIRQ(TIM4_IRQn);

    /* Initially stopped */
    TIM4->CR1 &= ~TIM_CR1_CEN;
    g_b_buzzer_pin_state = false;
    g_b_buzzer_is_active = false;
}

/* Stop continuous buzzer generation immediately */
void bsp_buzzer_off(void)
{
    g_b_buzzer_is_active = false;
    TIM4->CR1 &= ~TIM_CR1_CEN;
    TIM4->SR &= ~TIM_SR_UIF;
    TIM4->CNT = 0U;
    GPIOC->BSRR = (1UL << (BUZZER_PIN + 16U)); /* Atomic LOW */
    g_b_buzzer_pin_state = false;
}

/* Sets hardware timer tone with cubic perceptual volume curve (Continuous, Zero Jitter) */
void bsp_buzzer_set_tone(uint32_t freq_hz, uint16_t vol_adc)
{
    if ((freq_hz < BUZZER_FREQ_MIN_HZ) || (freq_hz > BUZZER_FREQ_MAX_HZ) || (vol_adc < BUZZER_VOL_MIN_THRESH))
    {
        bsp_buzzer_off();
    }
    else
    {
        uint32_t u4t_period_us = SEC_TO_US_FACTOR / freq_hz;
        uint32_t u4t_max_high = u4t_period_us / 2U;

        if (u4t_max_high == 0U)
        {
            u4t_max_high = 1U;
        }
        else
        {
            /* Period is valid */
        }

        /* Cubic perceptual curve: (vol_adc / 4095)^3 to match human hearing */
        uint32_t u4t_v = (uint32_t)vol_adc;
        uint32_t u4t_v_cube = (u4t_v * u4t_v) / ADC_MAX_VAL;
        u4t_v_cube = (u4t_v_cube * u4t_v) / ADC_MAX_VAL;

        uint32_t u4t_high = (u4t_v_cube * u4t_max_high) / ADC_MAX_VAL;
        if (u4t_high < 2U)
        {
            u4t_high = 2U; /* Minimum audible impulse */
        }
        else if (u4t_high >= u4t_period_us)
        {
            u4t_high = u4t_max_high;
        }
        else
        {
            /* Valid high duration */
        }

        uint32_t u4t_low = u4t_period_us - u4t_high;
        if (u4t_low < 2U)
        {
            u4t_low = 2U;
        }
        else
        {
            /* Valid low duration */
        }

        /* Update timer reload values */
        g_u2t_buzzer_high_ticks = (uint16_t)u4t_high;
        g_u2t_buzzer_low_ticks = (uint16_t)u4t_low;

        if (g_b_buzzer_is_active == false)
        {
            g_b_buzzer_is_active = true;
            TIM4->ARR = (uint32_t)g_u2t_buzzer_high_ticks - 1U;
            TIM4->EGR = TIM_EGR_UG;
            TIM4->SR &= ~TIM_SR_UIF;
            GPIOC->BSRR = (1UL << BUZZER_PIN); /* Atomic HIGH */
            g_b_buzzer_pin_state = true;
            TIM4->CR1 |= TIM_CR1_CEN; /* Start hardware counter */
        }
        else
        {
            /* Timer already running, new ticks apply seamlessly on next period */
        }
    }
}

/* Backward-compatible API wrapper for synthesizer */
void bsp_buzzer_play_chunk(uint32_t freq_hz, uint16_t vol_adc)
{
    bsp_buzzer_set_tone(freq_hz, vol_adc);
}

/* TIM4 Hardware Interrupt: Toggles PC3 with exact microsecond precision */
void TIM4_IRQHandler(void)
{
    if ((TIM4->SR & TIM_SR_UIF) != 0U)
    {
        TIM4->SR &= ~TIM_SR_UIF;

        if (g_b_buzzer_is_active == false)
        {
            GPIOC->BSRR = (1UL << (BUZZER_PIN + 16U)); /* Ensure LOW */
            g_b_buzzer_pin_state = false;
        }
        else if (g_b_buzzer_pin_state == true)
        {
            GPIOC->BSRR = (1UL << (BUZZER_PIN + 16U)); /* Atomic LOW */
            g_b_buzzer_pin_state = false;
            TIM4->ARR = (uint32_t)g_u2t_buzzer_low_ticks - 1U;
        }
        else
        {
            GPIOC->BSRR = (1UL << BUZZER_PIN);         /* Atomic HIGH */
            g_b_buzzer_pin_state = true;
            TIM4->ARR = (uint32_t)g_u2t_buzzer_high_ticks - 1U;
        }
    }
    else
    {
        /* Other timer flag */
    }
}
