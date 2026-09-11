/*******************************************************************************
 * File Name   : test_joystick.c
 * Description : Standalone HW-504 Joystick Diagnostic & Test Utility
 *               Tests PC0 (VRx), PC1 (VRy), PC2 (SW), PA4 (Vol), PA6 (LED)
 * Target MCU  : STM32F411RET6
 * Standard    : Toyota Embedded MISRA-C Compliant (22 Rules)
 ******************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include "test_joystick.h"
#include "bsp_gpio.h"
#include "bsp_adc.h"
#include "bsp_uart.h"
#include "bsp_buzzer.h"
#include "bsp_timer.h"

/* Named Constants (Rule 5 & Rule 10) */
#define TEST_PRINT_INTERVAL_MS      (100U)
#define TEST_BEEP_FREQ_HZ           (880U)
#define TEST_BEEP_VOL_RAW           (2000U)
#define TEST_THRESH_LOW             (1400U)
#define TEST_THRESH_HIGH            (2700U)
#define TEST_NUM_BUF_LEN            (12U)
#define TEST_DECIMAL_BASE           (10U)

/* Helper: Send unsigned 32-bit integer over UART */
static void test_uart_send_uint(uint32_t u4t_val)
{
    char str_buf[TEST_NUM_BUF_LEN];
    uint32_t u4t_idx = TEST_NUM_BUF_LEN - 1U;
    uint32_t u4t_temp = u4t_val;

    str_buf[u4t_idx] = '\0';
    if (u4t_temp == 0U)
    {
        u4t_idx--;
        str_buf[u4t_idx] = '0';
    }
    else
    {
        while ((u4t_temp > 0U) && (u4t_idx > 0U))
        {
            u4t_idx--;
            str_buf[u4t_idx] = (char)('0' + (char)(u4t_temp % TEST_DECIMAL_BASE));
            u4t_temp = u4t_temp / TEST_DECIMAL_BASE;
        }
    }
    bsp_uart_send_string(&str_buf[u4t_idx]);
}

void test_joystick_init(void)
{
    bsp_uart_send_string("\r\n====================================================\r\n");
    bsp_uart_send_string("  HW-504 Dual-Axis Joystick Diagnostic Utility\r\n");
    bsp_uart_send_string("  Wiring Check:\r\n");
    bsp_uart_send_string("    VRx -> PC0 (ADC1_IN10)\r\n");
    bsp_uart_send_string("    VRy -> PC1 (ADC1_IN11)\r\n");
    bsp_uart_send_string("    SW  -> PC2 (GPIO Pull-up)\r\n");
    bsp_uart_send_string("    Vol -> PA4 (ADC1_IN4)\r\n");
    bsp_uart_send_string("    LED -> PA6 (Lights on SW press)\r\n");
    bsp_uart_send_string("====================================================\r\n");

    /* Startup test chime */
    bsp_buzzer_play_chunk(TEST_BEEP_FREQ_HZ, TEST_BEEP_VOL_RAW);
    bsp_delay_ms(80U);
    bsp_buzzer_off();
}

void test_joystick_run(void)
{
    uint32_t u4t_last_print_ms = 0U;
    bool b_prev_sw = false;

    while (true)
    {
        uint32_t u4t_now = bsp_timer_get_ms();

        /* 1. Service ADC background conversion */
        bsp_adc_service(u4t_now);

        /* 2. Read Joystick Switch (PC2) */
        bool b_sw_pressed = bsp_gpio_read_joystick_switch();

        /* Light LED and short beep on SW press event */
        bsp_gpio_led_red_set(b_sw_pressed);

        if ((b_sw_pressed == true) && (b_prev_sw == false))
        {
            bsp_buzzer_play_chunk(TEST_BEEP_FREQ_HZ, TEST_BEEP_VOL_RAW);
            bsp_delay_ms(15U);
            bsp_buzzer_off();
        }
        else
        {
            /* Switch steady */
        }
        b_prev_sw = b_sw_pressed;

        /* 3. Periodic report output */
        if ((u4t_now - u4t_last_print_ms) >= TEST_PRINT_INTERVAL_MS)
        {
            u4t_last_print_ms = u4t_now;

            uint16_t u2t_raw_x = 0U;
            uint16_t u2t_raw_y = 0U;
            bool b_ready = bsp_adc_get_joystick_raw(&u2t_raw_x, &u2t_raw_y);
            uint8_t u1t_vol_pct = bsp_adc_get_volume_percent();

            if (b_ready == true)
            {
                bsp_uart_send_string("[HW-504] X: ");
                test_uart_send_uint((uint32_t)u2t_raw_x);

                if (u2t_raw_x < TEST_THRESH_LOW)
                {
                    bsp_uart_send_string(" [LEFT  ]");
                }
                else if (u2t_raw_x > TEST_THRESH_HIGH)
                {
                    bsp_uart_send_string(" [RIGHT ]");
                }
                else
                {
                    bsp_uart_send_string(" [CENTER]");
                }

                bsp_uart_send_string(" | Y: ");
                test_uart_send_uint((uint32_t)u2t_raw_y);

                if (u2t_raw_y < TEST_THRESH_LOW)
                {
                    bsp_uart_send_string(" [DOWN  ]");
                }
                else if (u2t_raw_y > TEST_THRESH_HIGH)
                {
                    bsp_uart_send_string(" [UP    ]");
                }
                else
                {
                    bsp_uart_send_string(" [CENTER]");
                }

                bsp_uart_send_string(" | SW: ");
                if (b_sw_pressed == true)
                {
                    bsp_uart_send_string("PRESSED ");
                }
                else
                {
                    bsp_uart_send_string("RELEASED");
                }

                bsp_uart_send_string(" | Vol: ");
                test_uart_send_uint((uint32_t)u1t_vol_pct);
                bsp_uart_send_string("%\r\n");
            }
            else
            {
                bsp_uart_send_string("[HW-504] Awaiting ADC snapshot ready...\r\n");
            }
        }
        else
        {
            /* Waiting for print timer */
        }
    }
}
