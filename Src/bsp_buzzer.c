/*******************************************************************************
 * File Name   : bsp_buzzer.c
 * Description : Board Support Package - Buzzer Sound Generator (PC3) Implementation
 * Target MCU  : STM32F411RET6
 * Standard    : Toyota Embedded MISRA-C Compliant (22 Rules)
 ******************************************************************************/

#include "bsp_buzzer.h"
#define STM32F411xE
#include "stm32f4xx.h"

/* Named Constants (Rule 5 & Rule 10) */
#define BUZZER_PIN              (3UL)      /* PC3 */
#define CHUNK_SILENCE_US        (5000U)
#define BUZZER_VOL_MIN_THRESH   (80U)
#define SEC_TO_US_FACTOR        (1000000U)
#define SEC_TO_MS_FACTOR        (1000U)
#define CHUNK_MS_DURATION       (5U)
#define ADC_MAX_VAL             (4095U)
#define DELAY_US_CALIB_COUNT    (1U)
#define DELAY_MS_CALIB_COUNT    (2000U)

/* Precise delay using calibrated software cycles */
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
    /* 1. Enable GPIOC Clock */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;

    /* 2. Configure PC3 as Output Push-Pull, High Speed */
    GPIOC->MODER &= ~(3UL << (BUZZER_PIN * 2U));
    GPIOC->MODER |=  (1UL << (BUZZER_PIN * 2U)); /* Output mode */
    GPIOC->OTYPER &= ~(1UL << BUZZER_PIN);        /* Push-Pull */
    GPIOC->OSPEEDR |= (3UL << (BUZZER_PIN * 2U)); /* High Speed */
    GPIOC->PUPDR &= ~(3UL << (BUZZER_PIN * 2U));  /* No-pull */
    GPIOC->ODR &= ~(1UL << BUZZER_PIN);           /* Initial LOW */
}

/* Turn off buzzer pin immediately */
void bsp_buzzer_off(void)
{
    GPIOC->ODR &= ~(1UL << BUZZER_PIN);
}

/* Synthesizes sound waves for ~5 milliseconds with cubic perceptual curve volume */
void bsp_buzzer_play_chunk(uint32_t freq_hz, uint16_t vol_adc)
{
    if ((freq_hz == 0U) || (vol_adc < BUZZER_VOL_MIN_THRESH))
    {
        bsp_buzzer_off();
        bsp_delay_us(CHUNK_SILENCE_US);
    }
    else
    {
        uint32_t u4t_period_us = SEC_TO_US_FACTOR / freq_hz;
        uint32_t u4t_max_high = u4t_period_us / 2U; /* 50% Duty cycle = Max Volume */

        /* Cubic perceptual curve: (vol_adc / 4095)^3 to match human hearing & piezo physics */
        uint32_t u4t_v = (uint32_t)vol_adc;
        uint32_t u4t_v_cube = (u4t_v * u4t_v) / ADC_MAX_VAL;
        u4t_v_cube = (u4t_v_cube * u4t_v) / ADC_MAX_VAL;

        uint32_t u4t_high = (u4t_v_cube * u4t_max_high) / ADC_MAX_VAL;
        if (u4t_high == 0U)
        {
            u4t_high = 1U; /* Minimum audible impulse */
        }
        else
        {
            /* Value within valid bounds */
        }
        uint32_t u4t_low = u4t_period_us - u4t_high;

        uint32_t u4t_cycles = (freq_hz * CHUNK_MS_DURATION) / SEC_TO_MS_FACTOR;
        if (u4t_cycles == 0U)
        {
            u4t_cycles = 1U;
        }
        else
        {
            /* Cycle count is valid */
        }

        for (uint32_t u4t_i = 0U; u4t_i < u4t_cycles; u4t_i++)
        {
            GPIOC->ODR |= (1UL << BUZZER_PIN);  /* HIGH */
            bsp_delay_us(u4t_high);
            GPIOC->ODR &= ~(1UL << BUZZER_PIN); /* LOW */
            bsp_delay_us(u4t_low);
        }
    }
}
