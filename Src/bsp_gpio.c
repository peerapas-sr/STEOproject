/*******************************************************************************
 * File Name   : bsp_gpio.c
 * Description : Board Support Package - 4 Independent Push Buttons Implementation
 *               - Key 1: PA10 [Key 1: Do / Sol] with EXTI10 Interrupt
 *               - Key 2: PB3  [Key 2: Re / La]
 *               - Key 3: PB5  [Key 3: Mi / Ti]
 *               - Key 4: PB4  [Key 4: Fa / High Do]
 * Target MCU  : STM32F411RET6 (Nucleo-F411RE)
 * Standard    : Toyota Embedded MISRA-C Compliant (22 Rules)
 ******************************************************************************/

#include "bsp_gpio.h"
#define STM32F411xE
#include "stm32f4xx.h"

/* Named Constants (Rule 5 & Rule 10) */
#define EXTI10_NVIC_PRIORITY    (2U)
#define EXTI_LINE_10_MASK       (1UL << 10U)

/* Volatile flag for EXTI10 interrupt event (Key 1 PA10 pressed) */
static volatile bool g_b_exti10_flag = false;

void bsp_gpio_init(void)
{
    /* 1. Enable AHB1 Clocks for GPIOA, GPIOB, GPIOC and APB2 for SYSCFG */
    RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN);
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    /* 2. Configure Output LED: PA6 (Red LED) */
    GPIOA->MODER &= ~(3UL << (LED_RED_PIN * 2U));
    GPIOA->MODER |=  (1UL << (LED_RED_PIN * 2U)); /* Output mode */
    GPIOA->OTYPER &= ~(1UL << LED_RED_PIN);       /* Push-Pull */
    GPIOA->OSPEEDR |= (3UL << (LED_RED_PIN * 2U));/* High Speed */
    GPIOA->PUPDR &= ~(3UL << (LED_RED_PIN * 2U)); /* No pull */
    GPIOA->ODR &= ~(1UL << LED_RED_PIN);          /* Start OFF */

    /* 3. Configure Input Key 1: PA10 (Pull-up) */
    GPIOA->MODER &= ~(3UL << (KEY1_PIN * 2U));    /* Input mode */
    GPIOA->PUPDR &= ~(3UL << (KEY1_PIN * 2U));
    GPIOA->PUPDR |=  (1UL << (KEY1_PIN * 2U));    /* Pull-up */

    /* 4. Configure Input Keys 2, 3, 4: PB3, PB5, PB4 (Pull-up) */
    GPIOB->MODER &= ~((3UL << (KEY2_PIN * 2U)) |
                      (3UL << (KEY3_PIN * 2U)) |
                      (3UL << (KEY4_PIN * 2U)));
    GPIOB->PUPDR &= ~((3UL << (KEY2_PIN * 2U)) |
                      (3UL << (KEY3_PIN * 2U)) |
                      (3UL << (KEY4_PIN * 2U)));
    GPIOB->PUPDR |=  ((1UL << (KEY2_PIN * 2U)) |
                      (1UL << (KEY3_PIN * 2U)) |
                      (1UL << (KEY4_PIN * 2U)));

    /* 5. Configure HW-504 Joystick Center Switch: PC2 (Pull-up) */
    GPIOC->MODER &= ~(3UL << (JOY_SW_PIN * 2U));
    GPIOC->PUPDR &= ~(3UL << (JOY_SW_PIN * 2U));
    GPIOC->PUPDR |=  (1UL << (JOY_SW_PIN * 2U));

    /* 6. Configure External Interrupt (EXTI) Line 10 on PA10 */
    SYSCFG->EXTICR[2] &= ~(0x0FUL << (2U * 4U)); /* 0x0 = Port A on Line 10 */

    EXTI->IMR  |= EXTI_LINE_10_MASK;              /* Unmask Line 10 */
    EXTI->FTSR |= EXTI_LINE_10_MASK;              /* Falling Edge Trigger (Active Low) */
    EXTI->RTSR &= ~EXTI_LINE_10_MASK;

    /* 7. Enable EXTI15_10 Interrupt in NVIC */
    NVIC_SetPriority(EXTI15_10_IRQn, EXTI10_NVIC_PRIORITY);
    NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/* Key State Readers (Active-Low: Pressed = true) */
bool bsp_gpio_read_key1(void)
{
    return ((GPIOA->IDR & (1UL << KEY1_PIN)) == 0U);
}

bool bsp_gpio_read_key2(void)
{
    return ((GPIOB->IDR & (1UL << KEY2_PIN)) == 0U);
}

bool bsp_gpio_read_key3(void)
{
    return ((GPIOB->IDR & (1UL << KEY3_PIN)) == 0U);
}

bool bsp_gpio_read_key4(void)
{
    return ((GPIOB->IDR & (1UL << KEY4_PIN)) == 0U);
}

bool bsp_gpio_read_joystick_switch(void)
{
    return ((GPIOC->IDR & (1UL << JOY_SW_PIN)) == 0U);
}

/* Red LED Setter */
void bsp_gpio_led_red_set(bool state)
{
    if (state == true)
    {
        GPIOA->ODR |= (1UL << LED_RED_PIN);
    }
    else
    {
        GPIOA->ODR &= ~(1UL << LED_RED_PIN);
    }
}

/* EXTI Flag Accessors */
bool bsp_gpio_get_exti_flag(void)
{
    return g_b_exti10_flag;
}

void bsp_gpio_clear_exti_flag(void)
{
    g_b_exti10_flag = false;
}

/* EXTI Lines 10 to 15 Interrupt Handler (Fired when Key 1 PA10 is pressed) */
void EXTI15_10_IRQHandler(void)
{
    if ((EXTI->PR & EXTI_LINE_10_MASK) != 0U)
    {
        EXTI->PR = EXTI_LINE_10_MASK; /* Clear pending flag by writing 1 */
        g_b_exti10_flag = true;       /* Set event flag */
    }
    else
    {
        /* Other interrupt on Lines 11 to 15 */
    }
}
