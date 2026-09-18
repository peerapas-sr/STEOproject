/*******************************************************************************
 * File Name   : bsp_oled.c
 * Description : Board Support Package - 1.30" I2C OLED (SH1106 / SSD1306) Driver
 * Target MCU  : STM32F411RET6 (Nucleo-F411RE)
 * Standard    : Toyota Embedded MISRA-C Compliant (22 Rules)
 *
 * Pin Connections:
 *   SCK  -> PB8 (I2C1_SCL, AF4, Open-Drain)
 *   SDA  -> PB9 (I2C1_SDA, AF4, Open-Drain)
 *   VCC  -> 3.3V
 *   GND  -> GND
 ******************************************************************************/

#include "bsp_oled.h"
#define STM32F411xE
#include "stm32f4xx.h"
#include "bsp_timer.h"
#include "bsp_buzzer.h"

/* Named Constants (Rule 5 & Rule 10) */
#define I2C_OLED_SLAVE_ADDR_WRITE   (0x78U)    /* 0x3C shifted left by 1 */
#define I2C_TIMEOUT_CYCLES          (10000U)
#define I2C_CTRL_BYTE_CMD           (0x00U)
#define I2C_CTRL_BYTE_DATA          (0x40U)

#define OLED_PAGE_SIZE_BYTES        (128U)
#define OLED_TOTAL_BUFFER_SIZE      (1024U)    /* 128 * 8 */
#define OLED_SERVICE_SLICE_MS       (5U)

#define SH1106_PAGE_CMD_BASE        (0xB0U)
#define SH1106_COL_LOW_OFFSET       (0x02U)    /* 1.3" SH1106 offset 2 columns */
#define SH1106_COL_HIGH_BASE        (0x10U)

#define FONT_CHAR_WIDTH_PX          (5U)
#define FONT_FIRST_ASCII            (32U)
#define FONT_LAST_ASCII             (95U)
#define FONT_TOTAL_CHARS            (64U)

#define PIANO_KEY_WIDTH_PX          (16U)
#define PIANO_BORDER_TOP_Y          (24U)
#define PIANO_BORDER_BOT_Y          (63U)
#define PIANO_BLACK_KEY_BOT_Y       (44U)
#define PIANO_BLACK_KEY_WIDTH_PX    (8U)

/* 5x7 ASCII Font Table (ASCII 32 to 95) */
static const uint8_t OLED_FONT5X7[FONT_TOTAL_CHARS][FONT_CHAR_WIDTH_PX] = {
    {0x00U, 0x00U, 0x00U, 0x00U, 0x00U}, /* 32: Space */
    {0x00U, 0x00U, 0x5FU, 0x00U, 0x00U}, /* 33: ! */
    {0x00U, 0x07U, 0x00U, 0x07U, 0x00U}, /* 34: " */
    {0x14U, 0x7FU, 0x14U, 0x7FU, 0x14U}, /* 35: # */
    {0x24U, 0x2AU, 0x7FU, 0x2AU, 0x12U}, /* 36: $ */
    {0x23U, 0x13U, 0x08U, 0x64U, 0x62U}, /* 37: % */
    {0x36U, 0x49U, 0x55U, 0x22U, 0x50U}, /* 38: & */
    {0x00U, 0x05U, 0x03U, 0x00U, 0x00U}, /* 39: ' */
    {0x00U, 0x1CU, 0x22U, 0x41U, 0x00U}, /* 40: ( */
    {0x00U, 0x41U, 0x22U, 0x1CU, 0x00U}, /* 41: ) */
    {0x14U, 0x08U, 0x3EU, 0x08U, 0x14U}, /* 42: * */
    {0x08U, 0x08U, 0x3EU, 0x08U, 0x08U}, /* 43: + */
    {0x00U, 0x50U, 0x30U, 0x00U, 0x00U}, /* 44: , */
    {0x08U, 0x08U, 0x08U, 0x08U, 0x08U}, /* 45: - */
    {0x00U, 0x60U, 0x60U, 0x00U, 0x00U}, /* 46: . */
    {0x20U, 0x10U, 0x08U, 0x04U, 0x02U}, /* 47: / */
    {0x3EU, 0x51U, 0x49U, 0x45U, 0x3EU}, /* 48: 0 */
    {0x00U, 0x42U, 0x7FU, 0x40U, 0x00U}, /* 49: 1 */
    {0x42U, 0x61U, 0x51U, 0x49U, 0x46U}, /* 50: 2 */
    {0x21U, 0x41U, 0x45U, 0x4BU, 0x31U}, /* 51: 3 */
    {0x18U, 0x14U, 0x12U, 0x7FU, 0x10U}, /* 52: 4 */
    {0x27U, 0x45U, 0x45U, 0x45U, 0x39U}, /* 53: 5 */
    {0x3CU, 0x4AU, 0x49U, 0x49U, 0x30U}, /* 54: 6 */
    {0x01U, 0x71U, 0x09U, 0x05U, 0x03U}, /* 55: 7 */
    {0x36U, 0x49U, 0x49U, 0x49U, 0x36U}, /* 56: 8 */
    {0x06U, 0x49U, 0x49U, 0x29U, 0x1EU}, /* 57: 9 */
    {0x00U, 0x36U, 0x36U, 0x00U, 0x00U}, /* 58: : */
    {0x00U, 0x56U, 0x36U, 0x00U, 0x00U}, /* 59: ; */
    {0x08U, 0x14U, 0x22U, 0x41U, 0x00U}, /* 60: < */
    {0x14U, 0x14U, 0x14U, 0x14U, 0x14U}, /* 61: = */
    {0x00U, 0x41U, 0x22U, 0x14U, 0x08U}, /* 62: > */
    {0x02U, 0x01U, 0x51U, 0x09U, 0x06U}, /* 63: ? */
    {0x32U, 0x49U, 0x79U, 0x41U, 0x3EU}, /* 64: @ */
    {0x7EU, 0x11U, 0x11U, 0x11U, 0x7EU}, /* 65: A */
    {0x7FU, 0x49U, 0x49U, 0x49U, 0x36U}, /* 66: B */
    {0x3EU, 0x41U, 0x41U, 0x41U, 0x22U}, /* 67: C */
    {0x7FU, 0x41U, 0x41U, 0x22U, 0x1CU}, /* 68: D */
    {0x7FU, 0x49U, 0x49U, 0x49U, 0x41U}, /* 69: E */
    {0x7FU, 0x09U, 0x09U, 0x09U, 0x01U}, /* 70: F */
    {0x3EU, 0x41U, 0x49U, 0x49U, 0x7AU}, /* 71: G */
    {0x7FU, 0x08U, 0x08U, 0x08U, 0x7FU}, /* 72: H */
    {0x00U, 0x41U, 0x7FU, 0x41U, 0x00U}, /* 73: I */
    {0x20U, 0x40U, 0x41U, 0x3FU, 0x01U}, /* 74: J */
    {0x7FU, 0x08U, 0x14U, 0x22U, 0x41U}, /* 75: K */
    {0x7FU, 0x40U, 0x40U, 0x40U, 0x40U}, /* 76: L */
    {0x7FU, 0x02U, 0x0CU, 0x02U, 0x7FU}, /* 77: M */
    {0x7FU, 0x04U, 0x08U, 0x10U, 0x7FU}, /* 78: N */
    {0x3EU, 0x41U, 0x41U, 0x41U, 0x3EU}, /* 79: O */
    {0x7FU, 0x09U, 0x09U, 0x09U, 0x06U}, /* 80: P */
    {0x3EU, 0x41U, 0x51U, 0x21U, 0x5EU}, /* 81: Q */
    {0x7FU, 0x09U, 0x19U, 0x29U, 0x46U}, /* 82: R */
    {0x46U, 0x49U, 0x49U, 0x49U, 0x31U}, /* 83: S */
    {0x01U, 0x01U, 0x7FU, 0x01U, 0x01U}, /* 84: T */
    {0x3FU, 0x40U, 0x40U, 0x40U, 0x3FU}, /* 85: U */
    {0x1FU, 0x20U, 0x40U, 0x20U, 0x1FU}, /* 86: V */
    {0x3FU, 0x40U, 0x38U, 0x40U, 0x3FU}, /* 87: W */
    {0x63U, 0x14U, 0x08U, 0x14U, 0x63U}, /* 88: X */
    {0x07U, 0x08U, 0x70U, 0x08U, 0x07U}, /* 89: Y */
    {0x61U, 0x51U, 0x49U, 0x45U, 0x43U}, /* 90: Z */
    {0x00U, 0x7FU, 0x41U, 0x41U, 0x00U}, /* 91: [ */
    {0x02U, 0x04U, 0x08U, 0x10U, 0x20U}, /* 92: \ */
    {0x00U, 0x41U, 0x41U, 0x7FU, 0x00U}, /* 93: ] */
    {0x04U, 0x02U, 0x01U, 0x02U, 0x04U}, /* 94: ^ */
    {0x40U, 0x40U, 0x40U, 0x40U, 0x40U}  /* 95: _ */
};

/* OLED Screen Framebuffer (1024 Bytes) */
static uint8_t  g_u1t_oled_buffer[OLED_TOTAL_BUFFER_SIZE];
static uint8_t  g_u1t_current_page = 0U;
static uint32_t g_u4t_last_service_ms = 0U;

/* Note Display Text Arrays */
static const char *NOTE_NAMES[OLED_PIANO_NUM_KEYS] = {
    "C7 (DO)", "D7 (RE)", "E7 (MI)", "F7 (FA)",
    "G7 (SO)", "A7 (LA)", "B7 (TI)", "C8 (DO)"
};

static const char *NOTE_FREQS[OLED_PIANO_NUM_KEYS] = {
    "2093H", "2349H", "2637H", "2794H",
    "3136H", "3520H", "3951H", "4186H"
};

static const char *KEY_LABELS[OLED_PIANO_NUM_KEYS] = {
    "DO", "RE", "MI", "FA", "SO", "LA", "TI", "C8"
};

/* Low-Level I2C Helper Functions */
static bool oled_i2c_start(uint8_t u1t_slave_addr)
{
    bool b_success = true;
    uint32_t u4t_timeout = I2C_TIMEOUT_CYCLES;

    /* Wait while bus is busy */
    while (((I2C1->SR2 & I2C_SR2_BUSY) != 0U) && (u4t_timeout > 0U))
    {
        u4t_timeout--;
    }

    if (u4t_timeout == 0U)
    {
        b_success = false;
    }
    else
    {
        /* Generate START condition */
        I2C1->CR1 |= I2C_CR1_START;
        u4t_timeout = I2C_TIMEOUT_CYCLES;

        while (((I2C1->SR1 & I2C_SR1_SB) == 0U) && (u4t_timeout > 0U))
        {
            u4t_timeout--;
        }

        if (u4t_timeout == 0U)
        {
            b_success = false;
        }
        else
        {
            /* Send Slave Address */
            (void)I2C1->SR1;
            I2C1->DR = u1t_slave_addr;
            u4t_timeout = I2C_TIMEOUT_CYCLES;

            while (((I2C1->SR1 & I2C_SR1_ADDR) == 0U) && (u4t_timeout > 0U))
            {
                if ((I2C1->SR1 & I2C_SR1_AF) != 0U)
                {
                    I2C1->SR1 &= ~I2C_SR1_AF;
                    u4t_timeout = 0U;
                }
                else
                {
                    u4t_timeout--;
                }
            }

            if (u4t_timeout == 0U)
            {
                I2C1->CR1 |= I2C_CR1_STOP;
                b_success = false;
            }
            else
            {
                /* Clear ADDR flag by reading SR1 and SR2 */
                (void)I2C1->SR1;
                (void)I2C1->SR2;
                b_success = true;
            }
        }
    }

    return b_success;
}

static bool oled_i2c_write_byte(uint8_t u1t_data)
{
    bool b_success = true;
    uint32_t u4t_timeout = I2C_TIMEOUT_CYCLES;

    while (((I2C1->SR1 & I2C_SR1_TXE) == 0U) && (u4t_timeout > 0U))
    {
        u4t_timeout--;
    }

    if (u4t_timeout == 0U)
    {
        b_success = false;
    }
    else
    {
        I2C1->DR = u1t_data;
        b_success = true;
    }

    return b_success;
}

static void oled_i2c_stop(void)
{
    uint32_t u4t_timeout = I2C_TIMEOUT_CYCLES;

    while (((I2C1->SR1 & I2C_SR1_BTF) == 0U) && (u4t_timeout > 0U))
    {
        u4t_timeout--;
    }

    I2C1->CR1 |= I2C_CR1_STOP;
}

static void oled_send_command(uint8_t u1t_cmd)
{
    if (oled_i2c_start(I2C_OLED_SLAVE_ADDR_WRITE) == true)
    {
        (void)oled_i2c_write_byte(I2C_CTRL_BYTE_CMD);
        (void)oled_i2c_write_byte(u1t_cmd);
        oled_i2c_stop();
    }
    else
    {
        /* I2C failure handled gracefully */
    }
}

static void oled_send_command_list(const uint8_t *p_cmds, uint8_t u1t_len)
{
    if (oled_i2c_start(I2C_OLED_SLAVE_ADDR_WRITE) == true)
    {
        (void)oled_i2c_write_byte(I2C_CTRL_BYTE_CMD);
        for (uint8_t u1t_i = 0U; u1t_i < u1t_len; u1t_i++)
        {
            (void)oled_i2c_write_byte(p_cmds[u1t_i]);
        }
        oled_i2c_stop();
    }
    else
    {
        /* I2C failure handled gracefully */
    }
}

/* Graphics Drawing Primitives */
void bsp_oled_clear_buffer(void)
{
    for (uint16_t u2t_i = 0U; u2t_i < OLED_TOTAL_BUFFER_SIZE; u2t_i++)
    {
        g_u1t_oled_buffer[u2t_i] = 0x00U;
    }
}

void bsp_oled_set_pixel(uint8_t u1t_x, uint8_t u1t_y, bool b_color)
{
    if ((u1t_x < OLED_WIDTH_PX) && (u1t_y < OLED_HEIGHT_PX))
    {
        uint8_t u1t_page = u1t_y / 8U;
        uint8_t u1t_bit = u1t_y % 8U;
        uint16_t u2t_index = ((uint16_t)u1t_page * OLED_WIDTH_PX) + (uint16_t)u1t_x;

        if (b_color == true)
        {
            g_u1t_oled_buffer[u2t_index] |= (uint8_t)(1U << u1t_bit);
        }
        else
        {
            g_u1t_oled_buffer[u2t_index] &= (uint8_t)(~(1U << u1t_bit));
        }
    }
    else
    {
        /* Coordinate out of bounds */
    }
}

void bsp_oled_draw_hline(uint8_t u1t_x0, uint8_t u1t_x1, uint8_t u1t_y, bool b_color)
{
    uint8_t u1t_start = u1t_x0;
    uint8_t u1t_end = u1t_x1;

    if (u1t_start > u1t_end)
    {
        u1t_start = u1t_x1;
        u1t_end = u1t_x0;
    }
    else
    {
        /* In correct order */
    }

    for (uint8_t u1t_x = u1t_start; u1t_x <= u1t_end; u1t_x++)
    {
        bsp_oled_set_pixel(u1t_x, u1t_y, b_color);
    }
}

void bsp_oled_draw_vline(uint8_t u1t_x, uint8_t u1t_y0, uint8_t u1t_y1, bool b_color)
{
    uint8_t u1t_start = u1t_y0;
    uint8_t u1t_end = u1t_y1;

    if (u1t_start > u1t_end)
    {
        u1t_start = u1t_y1;
        u1t_end = u1t_y0;
    }
    else
    {
        /* In correct order */
    }

    for (uint8_t u1t_y = u1t_start; u1t_y <= u1t_end; u1t_y++)
    {
        bsp_oled_set_pixel(u1t_x, u1t_y, b_color);
    }
}

void bsp_oled_fill_rect(uint8_t u1t_x0, uint8_t u1t_y0, uint8_t u1t_x1, uint8_t u1t_y1, bool b_color)
{
    for (uint8_t u1t_y = u1t_y0; u1t_y <= u1t_y1; u1t_y++)
    {
        bsp_oled_draw_hline(u1t_x0, u1t_x1, u1t_y, b_color);
    }
}

void bsp_oled_draw_string(uint8_t u1t_x, uint8_t u1t_page, const char *p_str, bool b_invert)
{
    uint8_t u1t_curr_x = u1t_x;

    if (p_str != (const char *)0)
    {
        while ((*p_str != '\0') && (u1t_curr_x < (OLED_WIDTH_PX - FONT_CHAR_WIDTH_PX)))
        {
            char c = *p_str;
            uint8_t u1t_ascii = (uint8_t)c;

            /* Convert lowercase to uppercase */
            if ((u1t_ascii >= 97U) && (u1t_ascii <= 122U))
            {
                u1t_ascii = u1t_ascii - 32U;
            }
            else
            {
                /* Keep as is */
            }

            if ((u1t_ascii >= FONT_FIRST_ASCII) && (u1t_ascii <= FONT_LAST_ASCII))
            {
                uint8_t u1t_font_idx = u1t_ascii - FONT_FIRST_ASCII;
                uint16_t u2t_base_idx = ((uint16_t)u1t_page * OLED_WIDTH_PX) + (uint16_t)u1t_curr_x;

                for (uint8_t u1t_col = 0U; u1t_col < FONT_CHAR_WIDTH_PX; u1t_col++)
                {
                    uint8_t u1t_bits = OLED_FONT5X7[u1t_font_idx][u1t_col];
                    if (b_invert == true)
                    {
                        u1t_bits = ~u1t_bits;
                    }
                    else
                    {
                        /* Normal */
                    }
                    g_u1t_oled_buffer[u2t_base_idx + u1t_col] = u1t_bits;
                }

                /* 1-pixel gap after character */
                if (b_invert == true)
                {
                    g_u1t_oled_buffer[u2t_base_idx + FONT_CHAR_WIDTH_PX] = 0xFFU;
                }
                else
                {
                    g_u1t_oled_buffer[u2t_base_idx + FONT_CHAR_WIDTH_PX] = 0x00U;
                }

                u1t_curr_x = u1t_curr_x + (FONT_CHAR_WIDTH_PX + 1U);
            }
            else
            {
                /* Skip unprintable character */
                u1t_curr_x = u1t_curr_x + (FONT_CHAR_WIDTH_PX + 1U);
            }

            p_str++;
        }
    }
    else
    {
        /* Null pointer guard */
    }
}

/* Theme 2: Virtual Piano UI Renderers */
void bsp_oled_render_header(const char *p_mode, bool b_high_bank, uint8_t u1t_vol_pct, int8_t s1t_note, int32_t s4t_cents)
{
    /* Line 0: Mode Badge, Bank Status, Volume Level */
    bsp_oled_draw_string(0U, 0U, p_mode, false);

    if (b_high_bank == true)
    {
        bsp_oled_draw_string(44U, 0U, "[HI]", false);
    }
    else
    {
        bsp_oled_draw_string(44U, 0U, "[LO]", false);
    }

    bsp_oled_draw_string(72U, 0U, "VOL:", false);

    /* Mini Volume Bar at X: 96..126, Y: 1..6 */
    bsp_oled_draw_hline(96U, 126U, 1U, true);
    bsp_oled_draw_hline(96U, 126U, 6U, true);
    bsp_oled_draw_vline(96U, 1U, 6U, true);
    bsp_oled_draw_vline(126U, 1U, 6U, true);

    uint8_t u1t_fill_len = (uint8_t)(((uint32_t)u1t_vol_pct * 28U) / 100U);
    if (u1t_fill_len > 0U)
    {
        bsp_oled_fill_rect(97U, 2U, (uint8_t)(97U + u1t_fill_len), 5U, true);
    }
    else
    {
        /* Zero volume fill */
    }

    /* Line 1: Active Note, Frequency, and Pitch Bend Cents */
    if ((s1t_note >= 0) && (s1t_note < (int8_t)OLED_PIANO_NUM_KEYS))
    {
        bsp_oled_draw_string(0U, 1U, NOTE_NAMES[s1t_note], false);
        bsp_oled_draw_string(50U, 1U, NOTE_FREQS[s1t_note], false);
    }
    else
    {
        bsp_oled_draw_string(0U, 1U, "-- SILENT --", false);
    }

    if (s4t_cents > 0)
    {
        bsp_oled_draw_string(88U, 1U, "P:+", false);
        bsp_oled_set_pixel(108U, 10U, true);
    }
    else if (s4t_cents < 0)
    {
        bsp_oled_draw_string(88U, 1U, "P:-", false);
        bsp_oled_set_pixel(108U, 10U, true);
    }
    else
    {
        bsp_oled_draw_string(88U, 1U, "P: 0", false);
    }
}

void bsp_oled_render_pitch_gauge(int32_t s4t_norm_x)
{
    /* Page 2: Separator line and Pitch Roll gauge */
    bsp_oled_draw_hline(0U, 127U, 16U, true);

    bsp_oled_draw_string(2U, 2U, "PITCH", false);
    bsp_oled_draw_string(96U, 2U, "ROLL", false);

    /* Center Pitch Box at X: 40..88, Y: 18..22 */
    bsp_oled_draw_hline(40U, 88U, 18U, true);
    bsp_oled_draw_hline(40U, 88U, 22U, true);
    bsp_oled_draw_vline(40U, 18U, 22U, true);
    bsp_oled_draw_vline(88U, 18U, 22U, true);

    /* Center Marker tick at X = 64 */
    bsp_oled_draw_vline(64U, 19U, 21U, true);

    /* Moving indicator dot */
    int32_t s4t_offset = (s4t_norm_x * 20) / 1000;
    int32_t s4t_marker_x = 64 + s4t_offset;

    if (s4t_marker_x < 42)
    {
        s4t_marker_x = 42;
    }
    else if (s4t_marker_x > 86)
    {
        s4t_marker_x = 86;
    }
    else
    {
        /* Within gauge bounds */
    }

    bsp_oled_fill_rect((uint8_t)(s4t_marker_x - 1), 19U, (uint8_t)(s4t_marker_x + 1), 21U, true);
}

void bsp_oled_render_piano_keyboard(int8_t s1t_active_key)
{
    /* 1. Top and Bottom keyboard borders */
    bsp_oled_draw_hline(0U, 127U, PIANO_BORDER_TOP_Y, true);
    bsp_oled_draw_hline(0U, 127U, PIANO_BORDER_BOT_Y, true);

    /* 2. White Key Borders and Active Invert Fill */
    for (uint8_t u1t_k = 0U; u1t_k < OLED_PIANO_NUM_KEYS; u1t_k++)
    {
        uint8_t u1t_x0 = u1t_k * PIANO_KEY_WIDTH_PX;
        uint8_t u1t_x1 = (u1t_x0 + PIANO_KEY_WIDTH_PX) - 1U;
        bool b_is_active = (s1t_active_key == (int8_t)u1t_k);

        /* Vertical dividing line between keys */
        bsp_oled_draw_vline(u1t_x0, PIANO_BORDER_TOP_Y, PIANO_BORDER_BOT_Y, true);
        bsp_oled_draw_vline(u1t_x1, PIANO_BORDER_TOP_Y, PIANO_BORDER_BOT_Y, true);

        if (b_is_active == true)
        {
            /* Highlight active key by filling white */
            bsp_oled_fill_rect((uint8_t)(u1t_x0 + 1U), (uint8_t)(PIANO_BORDER_TOP_Y + 1U), (uint8_t)(u1t_x1 - 1U), (uint8_t)(PIANO_BORDER_BOT_Y - 1U), true);
            /* Inverted text label inside active key (Page 7) */
            bsp_oled_draw_string((uint8_t)(u1t_x0 + 3U), 7U, KEY_LABELS[u1t_k], true);
        }
        else
        {
            /* Normal non-inverted text label inside key (Page 7) */
            bsp_oled_draw_string((uint8_t)(u1t_x0 + 3U), 7U, KEY_LABELS[u1t_k], false);
        }
    }

    /* 3. Black Keys (C#, D#, F#, G#, A#) */
    /* Black Key 0 (between C & D): X = 12..19 */
    bsp_oled_fill_rect(12U, (uint8_t)(PIANO_BORDER_TOP_Y + 1U), 19U, PIANO_BLACK_KEY_BOT_Y, true);
    bsp_oled_draw_vline(12U, (uint8_t)(PIANO_BORDER_TOP_Y + 1U), PIANO_BLACK_KEY_BOT_Y, false);
    bsp_oled_draw_vline(19U, (uint8_t)(PIANO_BORDER_TOP_Y + 1U), PIANO_BLACK_KEY_BOT_Y, false);

    /* Black Key 1 (between D & E): X = 28..35 */
    bsp_oled_fill_rect(28U, (uint8_t)(PIANO_BORDER_TOP_Y + 1U), 35U, PIANO_BLACK_KEY_BOT_Y, true);
    bsp_oled_draw_vline(28U, (uint8_t)(PIANO_BORDER_TOP_Y + 1U), PIANO_BLACK_KEY_BOT_Y, false);
    bsp_oled_draw_vline(35U, (uint8_t)(PIANO_BORDER_TOP_Y + 1U), PIANO_BLACK_KEY_BOT_Y, false);

    /* Note: No black key between E & F (Key 2 and Key 3) */

    /* Black Key 2 (between F & G): X = 60..67 */
    bsp_oled_fill_rect(60U, (uint8_t)(PIANO_BORDER_TOP_Y + 1U), 67U, PIANO_BLACK_KEY_BOT_Y, true);
    bsp_oled_draw_vline(60U, (uint8_t)(PIANO_BORDER_TOP_Y + 1U), PIANO_BLACK_KEY_BOT_Y, false);
    bsp_oled_draw_vline(67U, (uint8_t)(PIANO_BORDER_TOP_Y + 1U), PIANO_BLACK_KEY_BOT_Y, false);

    /* Black Key 3 (between G & A): X = 76..83 */
    bsp_oled_fill_rect(76U, (uint8_t)(PIANO_BORDER_TOP_Y + 1U), 83U, PIANO_BLACK_KEY_BOT_Y, true);
    bsp_oled_draw_vline(76U, (uint8_t)(PIANO_BORDER_TOP_Y + 1U), PIANO_BLACK_KEY_BOT_Y, false);
    bsp_oled_draw_vline(83U, (uint8_t)(PIANO_BORDER_TOP_Y + 1U), PIANO_BLACK_KEY_BOT_Y, false);

    /* Black Key 4 (between A & B): X = 92..99 */
    bsp_oled_fill_rect(92U, (uint8_t)(PIANO_BORDER_TOP_Y + 1U), 99U, PIANO_BLACK_KEY_BOT_Y, true);
    bsp_oled_draw_vline(92U, (uint8_t)(PIANO_BORDER_TOP_Y + 1U), PIANO_BLACK_KEY_BOT_Y, false);
    bsp_oled_draw_vline(99U, (uint8_t)(PIANO_BORDER_TOP_Y + 1U), PIANO_BLACK_KEY_BOT_Y, false);
}

/* Page-by-Page Refresh Service (Non-blocking: ~2.8 ms per page slice) */
void bsp_oled_service(uint32_t u4t_now)
{
    if ((u4t_now - g_u4t_last_service_ms) >= OLED_SERVICE_SLICE_MS)
    {
        g_u4t_last_service_ms = u4t_now;

        uint8_t u1t_page = g_u1t_current_page;
        uint16_t u2t_page_offset = (uint16_t)u1t_page * OLED_PAGE_SIZE_BYTES;

        /* Set Page and Column Addresses for SH1106 / SSD1306 */
        if (oled_i2c_start(I2C_OLED_SLAVE_ADDR_WRITE) == true)
        {
            (void)oled_i2c_write_byte(I2C_CTRL_BYTE_CMD);
            (void)oled_i2c_write_byte((uint8_t)(SH1106_PAGE_CMD_BASE | u1t_page));
            (void)oled_i2c_write_byte(SH1106_COL_LOW_OFFSET);
            (void)oled_i2c_write_byte(SH1106_COL_HIGH_BASE);
            oled_i2c_stop();

            /* Stream 128 bytes of data for this page */
            if (oled_i2c_start(I2C_OLED_SLAVE_ADDR_WRITE) == true)
            {
                (void)oled_i2c_write_byte(I2C_CTRL_BYTE_DATA);
                for (uint16_t u2t_col = 0U; u2t_col < OLED_PAGE_SIZE_BYTES; u2t_col++)
                {
                    (void)oled_i2c_write_byte(g_u1t_oled_buffer[u2t_page_offset + u2t_col]);
                }
                oled_i2c_stop();
            }
            else
            {
                /* Data write skipped on I2C error */
            }
        }
        else
        {
            /* Page command skipped on I2C error */
        }

        g_u1t_current_page = (g_u1t_current_page + 1U) % OLED_NUM_PAGES;
    }
    else
    {
        /* Waiting for next slice interval */
    }
}

/* Initialization Sequence */
void bsp_oled_init(void)
{
    /* 1. Enable GPIOB and I2C1 Clocks */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    /* 2. Configure PB8 (SCL) and PB9 (SDA) */
    /* Alternate Function Mode (0b10) */
    GPIOB->MODER &= ~((3UL << (8U * 2U)) | (3UL << (9U * 2U)));
    GPIOB->MODER |=  ((2UL << (8U * 2U)) | (2UL << (9U * 2U)));

    /* Open Drain (0b1) */
    GPIOB->OTYPER |= ((1UL << 8U) | (1UL << 9U));

    /* Very High Speed (0b11) */
    GPIOB->OSPEEDR |= ((3UL << (8U * 2U)) | (3UL << (9U * 2U)));

    /* Pull-Up Enabled (0b01) */
    GPIOB->PUPDR &= ~((3UL << (8U * 2U)) | (3UL << (9U * 2U)));
    GPIOB->PUPDR |=  ((1UL << (8U * 2U)) | (1UL << (9U * 2U)));

    /* Alternate Function 4 (AF4) for I2C1 on PB8 and PB9 */
    GPIOB->AFR[1] &= ~((15UL << 0U) | (15UL << 4U));
    GPIOB->AFR[1] |=  ((4UL  << 0U) | (4UL  << 4U));

    /* 3. Reset and Configure I2C1 Peripheral */
    I2C1->CR1 |= I2C_CR1_SWRST;
    bsp_delay_us(100U);
    I2C1->CR1 &= ~I2C_CR1_SWRST;

    /* Peripheral Clock = 16 MHz (APB1 default) */
    I2C1->CR2 = 16U;

    /* 400 kHz Fast Mode: CCR = 14, TRISE = 5 */
    I2C1->CCR = (uint16_t)(I2C_CCR_FS | 14U);
    I2C1->TRISE = 5U;

    /* Enable I2C1 */
    I2C1->CR1 |= I2C_CR1_PE;

    /* 4. Send OLED Initialization Command Table */
    static const uint8_t INIT_CMDS[] = {
        0xAEU,         /* Display OFF */
        0xD5U, 0x80U,   /* Set Display Clock Divide Ratio */
        0xA8U, 0x3FU,   /* Multiplex Ratio 64 (128x64) */
        0xD3U, 0x00U,   /* Display Offset 0 */
        0x40U,         /* Start Line 0 */
        0x8DU, 0x14U,   /* SSD1306 Charge Pump Enable */
        0xADU, 0x8BU,   /* SH1106 DC-DC Enable */
        0xA1U,         /* Segment Re-map: column 127 is SEG0 */
        0xC8U,         /* COM Output Scan Direction: remapped */
        0xDAU, 0x12U,   /* COM Pins Configuration */
        0x81U, 0xCFU,   /* Contrast Control */
        0xD9U, 0xF1U,   /* Pre-charge Period */
        0xDBU, 0x40U,   /* VCOMH Deselect Level */
        0x20U, 0x02U,   /* Memory Addressing: Page Mode */
        0xA4U,         /* Entire Display Resume */
        0xA6U,         /* Normal Display */
        0xAFU          /* Display ON */
    };

    bsp_delay_ms(50U);
    oled_send_command_list(INIT_CMDS, (uint8_t)sizeof(INIT_CMDS));
    bsp_delay_ms(50U);

    /* 5. Clear Initial Framebuffer */
    bsp_oled_clear_buffer();

    /* 6. Render Initial Virtual Piano Interface */
    bsp_oled_render_header("[LIVE]", false, 80U, -1, 0);
    bsp_oled_render_pitch_gauge(0);
    bsp_oled_render_piano_keyboard(-1);

    /* 7. Flush Entire Buffer once at startup so screen lights up immediately */
    for (uint8_t u1t_p = 0U; u1t_p < OLED_NUM_PAGES; u1t_p++)
    {
        uint16_t u2t_p_off = (uint16_t)u1t_p * OLED_PAGE_SIZE_BYTES;
        oled_send_command((uint8_t)(SH1106_PAGE_CMD_BASE | u1t_p));
        oled_send_command(SH1106_COL_LOW_OFFSET);
        oled_send_command(SH1106_COL_HIGH_BASE);

        if (oled_i2c_start(I2C_OLED_SLAVE_ADDR_WRITE) == true)
        {
            (void)oled_i2c_write_byte(I2C_CTRL_BYTE_DATA);
            for (uint16_t u2t_c = 0U; u2t_c < OLED_PAGE_SIZE_BYTES; u2t_c++)
            {
                (void)oled_i2c_write_byte(g_u1t_oled_buffer[u2t_p_off + u2t_c]);
            }
            oled_i2c_stop();
        }
        else
        {
            /* Error handled */
        }
    }
}
