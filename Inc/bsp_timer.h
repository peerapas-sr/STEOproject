/*******************************************************************************
 * File Name   : bsp_timer.h
 * Description : Board Support Package - TIM3 Hardware Timer Driver
 * Target MCU  : STM32F411RET6
 * Standard    : MISRA-C Compliant
 ******************************************************************************/

#ifndef BSP_TIMER_H
#define BSP_TIMER_H

#include <stdint.h>

void bsp_timer_init(void);
uint32_t bsp_timer_get_ms(void);

#endif /* BSP_TIMER_H */
