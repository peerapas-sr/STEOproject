/*******************************************************************************
 * File Name   : bsp_joystick.c
 * Description : Board Support Package - HW-504 Dual-Axis Analog Joystick Driver
 *               - VRx: PC0 (ADC1_IN10) -> Pitch Bend Axis (-1000..+1000)
 *               - VRy: PC1 (ADC1_IN11) -> Vibrato Axis    (-1000..+1000)
 *               - SW : PC2 (GPIO Pull-up) -> Event Generation (Click / Long Press)
 * Target MCU  : STM32F411RET6
 * Standard    : Toyota Embedded MISRA-C Compliant (22 Rules)
 ******************************************************************************/

#include "bsp_joystick.h"
#include "bsp_adc.h"
#include "bsp_gpio.h"

/* Named Constants (Rule 5 & Rule 10) */
#define JOY_CENTER_VAL          (2048U)
#define JOY_DEADZONE_COUNTS     (160U)
#define JOY_MAX_ADC             (4095U)
#define JOY_NORM_MAX            (1000)

#define SW_HOLD_MS              (600U)
#define SW_DEBOUNCE_MS          (30U)

/* Filtering Trackers */
static uint16_t         g_u2t_ema_x = JOY_CENTER_VAL;
static uint16_t         g_u2t_ema_y = JOY_CENTER_VAL;
static bool             g_b_ema_init = false;

static int32_t          g_s4t_norm_x = 0;
static int32_t          g_s4t_norm_y = 0;

/* Switch Debounce & Event Trackers */
static bool             g_b_sw_boot_locked = true;
static bool             g_b_sw_raw_prev = false;
static bool             g_b_sw_debounced = false;
static uint32_t         g_u4t_sw_edge_time_ms = 0U;
static uint32_t         g_u4t_sw_press_start_ms = 0U;
static bool             g_b_sw_long_fired = false;
static joy_sw_event_t   g_joy_sw_event = JOY_SW_EVT_NONE;

/* Helper: Piecewise deadzone normalization to -1000..+1000 */
static int32_t joystick_calc_norm(uint16_t u2t_ema, uint16_t u2t_center)
{
    int32_t s4t_norm = 0;
    int32_t s4t_diff = (int32_t)u2t_ema - (int32_t)u2t_center;

    if (s4t_diff > (int32_t)JOY_DEADZONE_COUNTS)
    {
        uint32_t u4t_span = (uint32_t)(JOY_MAX_ADC - (u2t_center + JOY_DEADZONE_COUNTS));
        uint32_t u4t_delta = (uint32_t)(s4t_diff - (int32_t)JOY_DEADZONE_COUNTS);
        s4t_norm = (int32_t)((u4t_delta * (uint32_t)JOY_NORM_MAX) / u4t_span);
    }
    else if (s4t_diff < -(int32_t)JOY_DEADZONE_COUNTS)
    {
        uint32_t u4t_span = (uint32_t)(u2t_center - JOY_DEADZONE_COUNTS);
        uint32_t u4t_delta = (uint32_t)(-s4t_diff - (int32_t)JOY_DEADZONE_COUNTS);
        s4t_norm = -(int32_t)((u4t_delta * (uint32_t)JOY_NORM_MAX) / u4t_span);
    }
    else
    {
        s4t_norm = 0;
    }

    if (s4t_norm > JOY_NORM_MAX)
    {
        s4t_norm = JOY_NORM_MAX;
    }
    else if (s4t_norm < -JOY_NORM_MAX)
    {
        s4t_norm = -JOY_NORM_MAX;
    }
    else
    {
        /* In valid range */
    }

    return s4t_norm;
}

/* Service Analog Axes */
static void joystick_service_axes(void)
{
    uint16_t u2t_raw_x = JOY_CENTER_VAL;
    uint16_t u2t_raw_y = JOY_CENTER_VAL;

    if (bsp_adc_get_joystick_raw(&u2t_raw_x, &u2t_raw_y) == true)
    {
        if (g_b_ema_init == false)
        {
            g_u2t_ema_x = u2t_raw_x;
            g_u2t_ema_y = u2t_raw_y;
            g_b_ema_init = true;
        }
        else
        {
            uint32_t u4t_fx = (((uint32_t)g_u2t_ema_x * 3U) + (uint32_t)u2t_raw_x) / 4U;
            uint32_t u4t_fy = (((uint32_t)g_u2t_ema_y * 3U) + (uint32_t)u2t_raw_y) / 4U;
            g_u2t_ema_x = (uint16_t)u4t_fx;
            g_u2t_ema_y = (uint16_t)u4t_fy;
        }

        g_s4t_norm_x = joystick_calc_norm(g_u2t_ema_x, JOY_CENTER_VAL);
        g_s4t_norm_y = joystick_calc_norm(g_u2t_ema_y, JOY_CENTER_VAL);
    }
    else
    {
        /* ADC busy */
    }
}

/* Service SW Center Button (Short / Long Press with Boot Guard) */
static void joystick_service_switch(uint32_t now_ms)
{
    bool b_sw_raw = bsp_gpio_read_joystick_switch();

    if (g_b_sw_boot_locked == true)
    {
        if (b_sw_raw == false)
        {
            g_b_sw_boot_locked = false;
        }
        else
        {
            /* Boot lock active */
        }
    }
    else
    {
        if (b_sw_raw != g_b_sw_raw_prev)
        {
            g_b_sw_raw_prev = b_sw_raw;
            g_u4t_sw_edge_time_ms = now_ms;
        }
        else if ((now_ms - g_u4t_sw_edge_time_ms) >= SW_DEBOUNCE_MS)
        {
            if (b_sw_raw != g_b_sw_debounced)
            {
                g_b_sw_debounced = b_sw_raw;
                if (g_b_sw_debounced == true)
                {
                    g_u4t_sw_press_start_ms = now_ms;
                    g_b_sw_long_fired = false;
                }
                else
                {
                    if (g_b_sw_long_fired == false)
                    {
                        g_joy_sw_event = JOY_SW_EVT_SHORT_PRESS;
                    }
                    else
                    {
                        /* Long press already fired */
                    }
                }
            }
            else
            {
                /* Debounced unchanged */
            }
        }
        else
        {
            /* Debouncing */
        }

        if ((g_b_sw_debounced == true) && (g_b_sw_long_fired == false))
        {
            if ((now_ms - g_u4t_sw_press_start_ms) >= SW_HOLD_MS)
            {
                g_b_sw_long_fired = true;
                g_joy_sw_event = JOY_SW_EVT_LONG_PRESS;
            }
            else
            {
                /* Holding */
            }
        }
        else
        {
            /* Idle */
        }
    }
}

void bsp_joystick_init(void)
{
    g_b_ema_init = false;
    g_u2t_ema_x = JOY_CENTER_VAL;
    g_u2t_ema_y = JOY_CENTER_VAL;
    g_s4t_norm_x = 0;
    g_s4t_norm_y = 0;
    g_b_sw_boot_locked = true;
    g_b_sw_raw_prev = false;
    g_b_sw_debounced = false;
    g_joy_sw_event = JOY_SW_EVT_NONE;
}

void bsp_joystick_service(uint32_t now_ms)
{
    joystick_service_axes();
    joystick_service_switch(now_ms);
}

int32_t bsp_joystick_get_norm_x(void)
{
    return g_s4t_norm_x;
}

int32_t bsp_joystick_get_norm_y(void)
{
    return g_s4t_norm_y;
}

joy_sw_event_t bsp_joystick_get_event(void)
{
    joy_sw_event_t evt = g_joy_sw_event;
    g_joy_sw_event = JOY_SW_EVT_NONE; /* Clear on read */
    return evt;
}

bool bsp_joystick_is_pressed(void)
{
    return g_b_sw_debounced;
}
