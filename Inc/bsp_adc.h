/*******************************************************************************
 * File Name   : bsp_adc.h
 * Description : Board Support Package - ADC Driver for Potentiometer (PA4)
 *               Interrupt-Driven (Zero Polling) Volume Control
 * Target MCU  : STM32F411RET6
 * Standard    : Toyota Embedded MISRA-C Compliant (22 Rules)
 ******************************************************************************/

#ifndef BSP_ADC_H
#define BSP_ADC_H

#include <stdint.h>
#include <stdbool.h>

void bsp_adc_init(void);
void bsp_adc_service(uint32_t now_ms);
uint8_t bsp_adc_get_volume_percent(void);
bool bsp_adc_get_joystick_raw(uint16_t *p_x_raw, uint16_t *p_y_raw);

#endif /* BSP_ADC_H */
