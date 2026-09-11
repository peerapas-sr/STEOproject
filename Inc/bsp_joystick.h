/*******************************************************************************
 * File Name   : bsp_joystick.h
 * Description : Board Support Package - HW-504 Dual-Axis Analog Joystick Driver
 *               - VRx: PC0 (ADC1_IN10)
 *               - VRy: PC1 (ADC1_IN11)
 *               - SW : PC2 (GPIO Pull-up)
 * Target MCU  : STM32F411RET6
 * Standard    : Toyota Embedded MISRA-C Compliant (22 Rules)
 ******************************************************************************/

#ifndef BSP_JOYSTICK_H
#define BSP_JOYSTICK_H

#include <stdint.h>
#include <stdbool.h>

/* Joystick Center Switch Events */
typedef enum {
    JOY_SW_EVT_NONE = 0,
    JOY_SW_EVT_SHORT_PRESS,
    JOY_SW_EVT_LONG_PRESS
} joy_sw_event_t;

void           bsp_joystick_init(void);
void           bsp_joystick_service(uint32_t now_ms);
int32_t        bsp_joystick_get_norm_x(void);
int32_t        bsp_joystick_get_norm_y(void);
joy_sw_event_t bsp_joystick_get_event(void);
bool           bsp_joystick_is_pressed(void);

#endif /* BSP_JOYSTICK_H */
