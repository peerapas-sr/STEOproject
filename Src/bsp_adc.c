/*******************************************************************************
 * File Name   : bsp_adc.c
 * Description : Board Support Package - ADC Driver for Potentiometer (PA4)
 *               Hardware Continuous Conversion with Digital Smoothing Filter
 * Target MCU  : STM32F411RET6
 * Standard    : Toyota Embedded MISRA-C Compliant (22 Rules)
 ******************************************************************************/

#include "bsp_adc.h"
#define STM32F411xE
#include "stm32f4xx.h"

/* Named Constants to eliminate magic numbers (Rule 5 & Rule 10) */
#define ADC_VOL_MUTE_THRESHOLD  (80U)
#define ADC_MAX_VALUE           (4095U)
#define PERCENT_MAX             (100U)
#define PIN_PA4                 (4U)
#define FILTER_WEIGHT_OLD       (3U)
#define FILTER_DIVISOR          (4U)

/* Filtered ADC storage updated continuously by ADC Interrupt (Toyota prefix) */
static volatile uint16_t g_u2t_adc_pot = 2048U; /* Channel 4 (PA4) - Potentiometer (Volume) */

void bsp_adc_init(void)
{
    /* 1. Enable Clocks for GPIOA and ADC1 */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    /* 2. Configure PA4 in Analog Mode (0b11) and No-Pull */
    GPIOA->MODER |= (3UL << (PIN_PA4 * 2U));
    GPIOA->PUPDR &= ~(3UL << (PIN_PA4 * 2U));

    /* 3. Configure Sampling Time (480 cycles for maximum impedance stability) */
    ADC1->SMPR2 |= (7UL << (PIN_PA4 * 3U));

    /* 4. Select Channel 4 (PA4) as single regular sequence channel */
    ADC1->SQR3 = PIN_PA4;
    ADC1->SQR1 &= ~ADC_SQR1_L;

    /* 5. Enable Continuous Conversion Mode in Hardware (Never stalls!) */
    ADC1->CR2 |= ADC_CR2_CONT;

    /* 6. Enable End-Of-Conversion Interrupt */
    ADC1->CR1 |= ADC_CR1_EOCIE;

    /* 7. Enable ADC Peripheral */
    ADC1->CR2 |= ADC_CR2_ADON;

    /* 8. Configure NVIC Priority and Enable ADC Interrupt */
    NVIC_SetPriority(ADC_IRQn, 1U);
    NVIC_EnableIRQ(ADC_IRQn);

    /* 9. Trigger first conversion (continuous hardware loop begins) */
    ADC1->CR2 |= ADC_CR2_SWSTART;
}

uint16_t bsp_adc_get_pot_raw(void)
{
    return g_u2t_adc_pot;
}

uint16_t bsp_adc_get_vrx_raw(void)
{
    return g_u2t_adc_pot;
}

uint16_t bsp_adc_get_vry_raw(void)
{
    return g_u2t_adc_pot;
}

/* No pitch bend when using single volume potentiometer */
int32_t bsp_adc_get_pitch_bend_percent(void)
{
    return 0;
}

/* Calculate Volume percentage (0 to 100%) from Potentiometer on PA4 */
uint8_t bsp_adc_get_volume_percent(void)
{
    uint8_t u1t_vol_pct = 0U;
    uint32_t u4t_val = (uint32_t)g_u2t_adc_pot;

    if (u4t_val <= ADC_VOL_MUTE_THRESHOLD)
    {
        u1t_vol_pct = 0U; /* Mute when turned to the bottom */
    }
    else
    {
        /* Rescale smoothly from MUTE_THRESHOLD..ADC_MAX_VALUE to 1..100% */
        uint32_t u4t_span = (uint32_t)(ADC_MAX_VALUE - ADC_VOL_MUTE_THRESHOLD);
        uint32_t u4t_adj = u4t_val - (uint32_t)ADC_VOL_MUTE_THRESHOLD;
        uint32_t u4t_pct = (u4t_adj * PERCENT_MAX) / u4t_span;

        if (u4t_pct > PERCENT_MAX)
        {
            u4t_pct = PERCENT_MAX;
        }
        else if (u4t_pct == 0U)
        {
            u4t_pct = 1U;
        }
        else
        {
            /* In range 1 to 100 */
        }
        u1t_vol_pct = (uint8_t)u4t_pct;
    }

    return u1t_vol_pct;
}

/* ADC Interrupt Service Routine (Continuously updates smoothed reading) */
void ADC_IRQHandler(void)
{
    if ((ADC1->SR & ADC_SR_EOC) != 0U)
    {
        uint16_t u2t_raw = (uint16_t)ADC1->DR; /* Reading DR clears EOC */

        /* Low-pass exponential moving average to filter potentiometer noise */
        uint32_t u4t_filtered = (((uint32_t)g_u2t_adc_pot * FILTER_WEIGHT_OLD) + (uint32_t)u2t_raw) / FILTER_DIVISOR;
        g_u2t_adc_pot = (uint16_t)u4t_filtered;
    }
    else
    {
        /* Other ADC interrupt events */
    }
}
