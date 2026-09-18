/*******************************************************************************
 * File Name   : bsp_buzzer.h
 * Description : Board Support Package - Hardware Timer Continuous Buzzer Generator (PC3)
 * Target MCU  : STM32F411RET6 (Nucleo-F411RE)
 * Standard    : Toyota Embedded MISRA-C Compliant (22 Rules)
 ******************************************************************************/

#ifndef BSP_BUZZER_H
#define BSP_BUZZER_H

#include <stdint.h>

void bsp_buzzer_init(void);
void bsp_buzzer_set_tone(uint32_t freq_hz, uint16_t vol_adc);
void bsp_buzzer_play_chunk(uint32_t freq_hz, uint16_t vol_adc);
void bsp_buzzer_off(void);
void bsp_delay_us(uint32_t us);
void bsp_delay_ms(uint32_t ms);

#endif /* BSP_BUZZER_H */
