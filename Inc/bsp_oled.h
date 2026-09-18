/*******************************************************************************
 * File Name   : bsp_oled.h
 * Description : Board Support Package - 1.30" I2C OLED (SH1106 / SSD1306) Driver
 * Target MCU  : STM32F411RET6 (Nucleo-F411RE)
 * Standard    : Toyota Embedded MISRA-C Compliant (22 Rules)
 *
 * Pin Connections:
 *   SCK  -> PB8 (I2C1_SCL)
 *   SDA  -> PB9 (I2C1_SDA)
 *   VCC  -> 3.3V
 *   GND  -> GND
 ******************************************************************************/

#ifndef BSP_OLED_H
#define BSP_OLED_H

#include <stdint.h>
#include <stdbool.h>

/* Display Dimensions */
#define OLED_WIDTH_PX               (128U)
#define OLED_HEIGHT_PX              (64U)
#define OLED_NUM_PAGES              (8U)
#define OLED_PIANO_NUM_KEYS         (8U)

/* Public API Functions */
void bsp_oled_init(void);
void bsp_oled_service(uint32_t u4t_now);
void bsp_oled_clear_buffer(void);
void bsp_oled_set_pixel(uint8_t u1t_x, uint8_t u1t_y, bool b_color);
void bsp_oled_draw_string(uint8_t u1t_x, uint8_t u1t_page, const char *p_str, bool b_invert);
void bsp_oled_draw_hline(uint8_t u1t_x0, uint8_t u1t_x1, uint8_t u1t_y, bool b_color);
void bsp_oled_draw_vline(uint8_t u1t_x, uint8_t u1t_y0, uint8_t u1t_y1, bool b_color);
void bsp_oled_fill_rect(uint8_t u1t_x0, uint8_t u1t_y0, uint8_t u1t_x1, uint8_t u1t_y1, bool b_color);

/* Theme 2: Virtual Piano UI Renderers */
void bsp_oled_render_piano_keyboard(int8_t s1t_active_key);
void bsp_oled_render_header(const char *p_mode, bool b_high_bank, uint8_t u1t_vol_pct, int8_t s1t_note, int32_t s4t_cents);
void bsp_oled_render_pitch_gauge(int32_t s4t_norm_x);

#endif /* BSP_OLED_H */
