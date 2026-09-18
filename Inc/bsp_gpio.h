/*******************************************************************************
 * File Name   : bsp_gpio.h
 * Description : Board Support Package - 4 Independent Push Buttons & Red LED
 *               - Key 1: PA10 (D2) [Key 1: Do / Sol] with EXTI Line 10 Interrupt
 *               - Key 2: PB3  (D3) [Key 2: Re / La]
 *               - Key 3: PB5  (D4) [Key 3: Mi / Ti]
 *               - Key 4: PB4  (D5) [Key 4: Fa / High Do]
 * Target MCU  : STM32F411RET6 (Nucleo-F411RE)
 * Standard    : Toyota Embedded MISRA-C Compliant (22 Rules)
 ******************************************************************************/

#ifndef BSP_GPIO_H
#define BSP_GPIO_H

#include <stdint.h>
#include <stdbool.h>

/* 4 Piano Push Buttons */
#define KEY1_PIN          (10UL) /* PA10 (D2) [Key 1: Do / Sol] + EXTI10 */
#define KEY2_PIN          (3UL)  /* PB3  (D3) [Key 2: Re / La] */
#define KEY3_PIN          (5UL)  /* PB5  (D4) [Key 3: Mi / Ti] */
#define KEY4_PIN          (4UL)  /* PB4  (D5) [Key 4: Fa / High Do] */

/* Backwards-compatible aliases */
#define KEY1_DO_PIN       KEY1_PIN
#define KEY2_RE_PIN       KEY2_PIN
#define KEY3_MI_PIN       KEY3_PIN
#define KEY4_SOL_PIN      KEY4_PIN

/* Status LED */
#define LED_RED_PIN       (6UL)  /* PA6  - Red Status LED */

/* HW-504 Joystick Center Switch */
#define JOY_SW_PIN        (2UL)  /* PC2  - Joystick Center Push Button (Active Low) */

/* Function Prototypes */
void bsp_gpio_init(void);

bool bsp_gpio_read_key1(void);
bool bsp_gpio_read_key2(void);
bool bsp_gpio_read_key3(void);
bool bsp_gpio_read_key4(void);

/* Backwards-compatible function aliases */
#define bsp_gpio_read_key1_do()   bsp_gpio_read_key1()
#define bsp_gpio_read_key2_re()   bsp_gpio_read_key2()
#define bsp_gpio_read_key3_mi()   bsp_gpio_read_key3()
#define bsp_gpio_read_key4_sol()  bsp_gpio_read_key4()

bool bsp_gpio_read_joystick_switch(void);
void bsp_gpio_led_red_set(bool state);
bool bsp_gpio_get_exti_flag(void);
void bsp_gpio_clear_exti_flag(void);

#endif /* BSP_GPIO_H */
