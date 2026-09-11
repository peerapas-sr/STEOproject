/*******************************************************************************
 * File Name   : bsp_adc.c
 * Description : Board Support Package - 3-Channel ADC Driver
 *               - Channel 4  : PA4 (Volume Potentiometer)
 *               - Channel 10 : PC0 (HW-504 Joystick VRx)
 *               - Channel 11 : PC1 (HW-504 Joystick VRy)
 *               Interrupt-Driven Sequential Single Conversion (Zero Polling)
 * Target MCU  : STM32F411RET6
 * Standard    : Toyota Embedded MISRA-C Compliant (22 Rules)
 ******************************************************************************/

#include "bsp_adc.h"
#define STM32F411xE
#include "stm32f4xx.h"

/* Named Constants (Rule 5 & Rule 10) */
#define ADC_VOL_MUTE_THRESHOLD      (80U)
#define ADC_MAX_VALUE               (4095U)
#define PERCENT_MAX                 (100U)

#define ADC_CH4_PA4                 (4U)
#define ADC_CH10_PC0                (10U)
#define ADC_CH11_PC1                (11U)

#define PIN_PA4                     (4U)
#define PIN_PC0                     (0U)
#define PIN_PC1                     (1U)

#define ADC_SAMPLING_TIME_480_CYC   (7UL)
#define ADC_TRIGGER_INTERVAL_MS     (5U)
#define ADC_TIMEOUT_LIMIT_MS        (20U)
#define ADC_NVIC_PRIORITY           (1U)

/* ADC Sequence Conversion States */
typedef enum {
    ADC_SEQ_IDLE = 0,
    ADC_SEQ_CH4_POT,
    ADC_SEQ_CH10_X,
    ADC_SEQ_CH11_Y
} adc_seq_state_t;

/* Internal conversion and published snapshot buffers */
static volatile adc_seq_state_t g_adc_state = ADC_SEQ_IDLE;
static volatile uint16_t g_u2t_raw_pot = 2048U;
static volatile uint16_t g_u2t_raw_x = 2048U;

static volatile uint16_t g_u2t_pub_pot = 2048U;
static volatile uint16_t g_u2t_pub_x = 2048U;
static volatile uint16_t g_u2t_pub_y = 2048U;
static volatile bool g_b_adc_ready = false;

static uint32_t g_u4t_last_adc_trigger_ms = 0U;
static uint32_t g_u4t_seq_start_time_ms = 0U;

void bsp_adc_init(void)
{
    /* 1. Enable Clocks for GPIOA, GPIOC and ADC1 */
    RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOCEN);
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    /* 2. Configure PA4 in Analog Mode (0b11) and No-Pull */
    GPIOA->MODER |= (3UL << (PIN_PA4 * 2U));
    GPIOA->PUPDR &= ~(3UL << (PIN_PA4 * 2U));

    /* 3. Configure PC0 and PC1 in Analog Mode (0b11) and No-Pull */
    GPIOC->MODER |= ((3UL << (PIN_PC0 * 2U)) | (3UL << (PIN_PC1 * 2U)));
    GPIOC->PUPDR &= ~((3UL << (PIN_PC0 * 2U)) | (3UL << (PIN_PC1 * 2U)));

    /* 4. Configure Sampling Time (480 cycles for maximum impedance stability) */
    ADC1->SMPR2 |= (ADC_SAMPLING_TIME_480_CYC << (ADC_CH4_PA4 * 3U));
    ADC1->SMPR1 |= ((ADC_SAMPLING_TIME_480_CYC << 0U) | (ADC_SAMPLING_TIME_480_CYC << 3U));

    /* 5. Configure ADC Common Prescaler: PCLK2 / 4 */
    ADC->CCR &= ~ADC_CCR_ADCPRE;
    ADC->CCR |= ADC_CCR_ADCPRE_0;

    /* 6. Single regular conversion sequence (1 conversion per trigger) */
    ADC1->SQR1 &= ~ADC_SQR1_L;
    ADC1->SQR3 = ADC_CH4_PA4;

    /* 7. Ensure Single Conversion Mode (Continuous mode disabled) */
    ADC1->CR2 &= ~ADC_CR2_CONT;

    /* 8. Enable End-Of-Conversion Interrupt */
    ADC1->CR1 |= ADC_CR1_EOCIE;

    /* 9. Configure NVIC Priority and Enable ADC Interrupt */
    NVIC_SetPriority(ADC_IRQn, ADC_NVIC_PRIORITY);
    NVIC_EnableIRQ(ADC_IRQn);

    /* 10. Enable ADC Peripheral */
    ADC1->CR2 |= ADC_CR2_ADON;
}

/* Service function called periodically in main loop (Zero busy-wait) */
void bsp_adc_service(uint32_t now_ms)
{
    /* Watchdog & error recovery: check timeout (> 20 ms) or overrun */
    if (g_adc_state != ADC_SEQ_IDLE)
    {
        uint32_t u4t_elapsed = now_ms - g_u4t_seq_start_time_ms;
        if ((u4t_elapsed > ADC_TIMEOUT_LIMIT_MS) || ((ADC1->SR & ADC_SR_OVR) != 0U))
        {
            ADC1->SR &= ~ADC_SR_OVR; /* Clear overrun flag */
            g_adc_state = ADC_SEQ_IDLE;
        }
        else
        {
            /* Sequence in progress within safe time limit */
        }
    }
    else
    {
        /* ADC state is idle */
    }

    /* Start new sequential conversion every >= 5 ms when idle */
    if (g_adc_state == ADC_SEQ_IDLE)
    {
        if ((now_ms - g_u4t_last_adc_trigger_ms) >= ADC_TRIGGER_INTERVAL_MS)
        {
            g_u4t_last_adc_trigger_ms = now_ms;
            g_u4t_seq_start_time_ms = now_ms;
            g_adc_state = ADC_SEQ_CH4_POT;
            ADC1->SQR3 = ADC_CH4_PA4;
            ADC1->CR2 |= ADC_CR2_SWSTART;
        }
        else
        {
            /* Awaiting next periodic trigger */
        }
    }
    else
    {
        /* ADC busy */
    }
}

/* Retrieve raw snapshot of Joystick X and Y (atomic) */
bool bsp_adc_get_joystick_raw(uint16_t *p_x_raw, uint16_t *p_y_raw)
{
    bool b_success = false;

    if ((p_x_raw != (uint16_t *)0) && (p_y_raw != (uint16_t *)0))
    {
        if (g_b_adc_ready == true)
        {
            NVIC_DisableIRQ(ADC_IRQn);
            *p_x_raw = g_u2t_pub_x;
            *p_y_raw = g_u2t_pub_y;
            NVIC_EnableIRQ(ADC_IRQn);
            b_success = true;
        }
        else
        {
            b_success = false;
        }
    }
    else
    {
        b_success = false;
    }

    return b_success;
}

/* Calculate Volume percentage (0 to 100%) from Potentiometer snapshot */
uint8_t bsp_adc_get_volume_percent(void)
{
    uint8_t u1t_vol_pct = 0U;
    uint16_t u2t_pot_val = g_u2t_pub_pot; /* Single 16-bit read is natively atomic */
    uint32_t u4t_val = (uint32_t)u2t_pot_val;

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

/* ADC Interrupt Service Routine: Sequential Conversions (PA4 -> PC0 -> PC1) */
void ADC_IRQHandler(void)
{
    if ((ADC1->SR & ADC_SR_EOC) != 0U)
    {
        uint16_t u2t_raw = (uint16_t)ADC1->DR; /* Clears EOC */

        switch (g_adc_state)
        {
            case ADC_SEQ_CH4_POT:
                g_u2t_raw_pot = u2t_raw;
                g_adc_state = ADC_SEQ_CH10_X;
                ADC1->SQR3 = ADC_CH10_PC0;
                ADC1->CR2 |= ADC_CR2_SWSTART;
                break;

            case ADC_SEQ_CH10_X:
                g_u2t_raw_x = u2t_raw;
                g_adc_state = ADC_SEQ_CH11_Y;
                ADC1->SQR3 = ADC_CH11_PC1;
                ADC1->CR2 |= ADC_CR2_SWSTART;
                break;

            case ADC_SEQ_CH11_Y:
                g_u2t_pub_pot = g_u2t_raw_pot;
                g_u2t_pub_x = g_u2t_raw_x;
                g_u2t_pub_y = u2t_raw;
                g_b_adc_ready = true;
                g_adc_state = ADC_SEQ_IDLE;
                break;

            default:
                g_adc_state = ADC_SEQ_IDLE;
                break;
        }
    }
    else
    {
        /* Other ADC interrupt events */
    }
}
