/*******************************************************************************
 * File Name   : bsp_uart.c
 * Description : USART2 Driver - Interrupt RX and Polling TX with Timeout
 * Target MCU  : STM32F411RET6
 * Standard    : Toyota Embedded MISRA-C Compliant (22 Rules)
 ******************************************************************************/

#include "bsp_uart.h"
#define STM32F411xE
#include "stm32f4xx.h"

/* Named Constants (Rule 5 & Rule 10) */
#define UART_RX_BUFFER_SIZE     (64U)
#define USART2_BRR_115200       (139U)
#define UART_TX_TIMEOUT         (100000U)
#define USART2_NVIC_PRIORITY    (1U)

/* Circular RX buffer managed by Interrupt Service Routine */
static volatile char    g_u1t_rx_buffer[UART_RX_BUFFER_SIZE];
static volatile uint8_t g_u1t_rx_head = 0U;
static volatile uint8_t g_u1t_rx_tail = 0U;

void bsp_uart_init(void)
{
    /* 1. Enable Clocks for GPIOA and USART2 */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    /* 2. Configure PA2 (TX) and PA3 (RX) as Alternate Function AF7 */
    GPIOA->MODER &= ~((3UL << (2U * 2U)) | (3UL << (3U * 2U)));
    GPIOA->MODER |=  ((2UL << (2U * 2U)) | (2UL << (3U * 2U))); /* AF mode */

    GPIOA->AFR[0] &= ~((0xFUL << (2U * 4U)) | (0xFUL << (3U * 4U)));
    GPIOA->AFR[0] |=  ((7UL   << (2U * 4U)) | (7UL   << (3U * 4U))); /* AF7 (USART2) */

    GPIOA->OSPEEDR |= ((3UL << (2U * 2U)) | (3UL << (3U * 2U)));
    GPIOA->PUPDR   &= ~((3UL << (2U * 2U)) | (3UL << (3U * 2U)));
    GPIOA->PUPDR   |=  ((1UL << (2U * 2U)) | (1UL << (3U * 2U))); /* Pull-up */

    /* 3. Configure Baud Rate = 115200 at 16 MHz APB1 clock (BRR = 139) */
    USART2->BRR = USART2_BRR_115200;

    /* 4. Enable Transmitter, Receiver, and RXNE Interrupt */
    USART2->CR1 = (USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE);
    USART2->CR1 |= USART_CR1_UE; /* Enable USART */

    /* 5. Configure NVIC for USART2 Interrupt */
    NVIC_SetPriority(USART2_IRQn, USART2_NVIC_PRIORITY);
    NVIC_EnableIRQ(USART2_IRQn);
}

/* Transmit a single character */
void bsp_uart_send_char(char c)
{
    uint32_t u4t_timeout = UART_TX_TIMEOUT;
    while (((USART2->SR & USART_SR_TXE) == 0U) && (u4t_timeout > 0U))
    {
        u4t_timeout--;
    }
    if (u4t_timeout > 0U)
    {
        USART2->DR = (uint8_t)c;
    }
    else
    {
        /* Timeout occurred: transmission skipped */
    }
}

/* Transmit null-terminated string */
void bsp_uart_send_string(const char *str)
{
    if (str != (const char *)0)
    {
        while (*str != '\0')
        {
            bsp_uart_send_char(*str);
            str++;
        }
    }
    else
    {
        /* Null pointer passed */
    }
}

/* Check if character available in RX buffer */
bool bsp_uart_has_rx_char(void)
{
    return (g_u1t_rx_head != g_u1t_rx_tail);
}

/* Read character from RX buffer */
char bsp_uart_get_rx_char(void)
{
    char c = '\0';
    if (g_u1t_rx_head != g_u1t_rx_tail)
    {
        c = g_u1t_rx_buffer[g_u1t_rx_tail];
        g_u1t_rx_tail = (uint8_t)((g_u1t_rx_tail + 1U) % UART_RX_BUFFER_SIZE);
    }
    else
    {
        /* Buffer empty */
    }
    return c;
}

/* USART2 Interrupt Service Routine (No Polling) */
void USART2_IRQHandler(void)
{
    if ((USART2->SR & USART_SR_RXNE) != 0U)
    {
        char received_byte = (char)(USART2->DR & 0xFFU);
        uint8_t u1t_next_head = (uint8_t)((g_u1t_rx_head + 1U) % UART_RX_BUFFER_SIZE);

        /* Prevent buffer overflow */
        if (u1t_next_head != g_u1t_rx_tail)
        {
            g_u1t_rx_buffer[g_u1t_rx_head] = received_byte;
            g_u1t_rx_head = u1t_next_head;
        }
        else
        {
            /* Buffer full: drop byte to prevent corruption */
        }
    }
    else
    {
        /* Other USART2 interrupt */
    }
}
