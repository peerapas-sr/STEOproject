/*******************************************************************************
 * File Name   : bsp_gpio.h
 * Description : Board Support Package - 4 Independent Push Buttons & Red LED
 *               - Key 1: PA10 (D2) [ปุ่มซ้ายสุด] พร้อม EXTI Line 10 Interrupt
 *               - Key 2: PB3  (D3)
 *               - Key 3: PB5  (D4)
 *               - Key 4: PB4  (D5)
 * Target MCU  : STM32F411RET6
 * Standard    : MISRA-C Compliant
 ******************************************************************************/

#ifndef BSP_GPIO_H
#define BSP_GPIO_H

#include <stdint.h>
#include <stdbool.h>

/* 4 Independent Push Buttons */
#define KEY1_DO_PIN       (10UL) /* PA10 (D2) [ซ้ายสุด] - Note Do + EXTI10 */
#define KEY2_RE_PIN       (3UL)  /* PB3  (D3) - Note Re */
#define KEY3_MI_PIN       (5UL)  /* PB5  (D4) - Note Mi */
#define KEY4_SOL_PIN      (4UL)  /* PB4  (D5) - Note Sol */

/* Status LED */
#define LED_RED_PIN       (6UL)  /* PA6  - Red Status LED */

/* Function Prototypes */
void bsp_gpio_init(void);
bool bsp_gpio_read_key1_do(void);
bool bsp_gpio_read_key2_re(void);
bool bsp_gpio_read_key3_mi(void);
bool bsp_gpio_read_key4_sol(void);
void bsp_gpio_led_red_set(bool state);
bool bsp_gpio_get_exti_flag(void);
void bsp_gpio_clear_exti_flag(void);

#endif /* BSP_GPIO_H */
