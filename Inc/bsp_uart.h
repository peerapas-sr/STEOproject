/*******************************************************************************
 * File Name   : bsp_uart.h
 * Description : USART2 Driver - Interrupt RX and Polling TX with Timeout
 * Target MCU  : STM32F411RET6
 * Standard    : MISRA-C Compliant
 ******************************************************************************/

#ifndef BSP_UART_H
#define BSP_UART_H

#include <stdint.h>
#include <stdbool.h>

void bsp_uart_init(void);
void bsp_uart_send_char(char c);
void bsp_uart_send_string(const char *str);
bool bsp_uart_has_rx_char(void);
char bsp_uart_get_rx_char(void);

#endif /* BSP_UART_H */
