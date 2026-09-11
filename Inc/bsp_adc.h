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

void bsp_adc_init(void);
uint8_t bsp_adc_get_volume_percent(void);

#endif /* BSP_ADC_H */
