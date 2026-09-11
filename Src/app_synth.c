/*******************************************************************************
 * File Name   : app_synth.c
 * Description : Application Layer - Synthesizer with Sequence Recorder
 *               Uses bsp_joystick for Pitch Bend, Vibrato, and Center Switch
 * Target MCU  : STM32F411RET6
 * Standard    : Toyota Embedded MISRA-C Compliant (22 Rules)
 ******************************************************************************/

#include "app_synth.h"
#include "bsp_gpio.h"
#include "bsp_adc.h"
#include "bsp_joystick.h"
#include "bsp_uart.h"
#include "bsp_buzzer.h"
#include "bsp_timer.h"

/* Named Constants (Rule 5 & Rule 10) */
#define SYNTH_MAX_SEQUENCE_STEPS    (64U)
#define SYNTH_NUM_NOTES             (4U)

#define SYNTH_NOTE_INDEX_DO         (0)
#define SYNTH_NOTE_INDEX_RE         (1)
#define SYNTH_NOTE_INDEX_MI         (2)
#define SYNTH_NOTE_INDEX_SOL        (3)

#define CHIME_NOTE_C4_FREQ          (262U)
#define CHIME_NOTE_C6_FREQ          (1047U)
#define CHIME_NOTE_G6_FREQ          (1568U)
#define CHIME_NOTE_VOL_RAW          (2500U)
#define CHIME_NOTE_VOL_FINAL        (3000U)
#define CHIME_DELAY_SHORT_MS        (50U)
#define CHIME_DELAY_LONG_MS         (100U)
#define CHIRP_DELAY_MS              (40U)
#define STATUS_TONE_NONE_FREQ       (0U)

#define COMBO_HOLD_MS               (600U)
#define REC_LED_BLINK_PERIOD_MS     (200U)

#define PLAYBACK_MIN_GAP_MS         (40U)
#define MIN_NOTE_DUR_RECORD_MS      (50U)
#define MAX_REST_GAP_RECORD_MS      (3000U)
#define DEF_REST_GAP_MS             (80U)
#define TRANSITION_REST_MS          (30U)
#define VOLUME_RAW_MAX              (4095U)
#define PERCENT_SCALE               (100U)

#define BEND_MAX_CENTS              (200)
#define VIBRATO_MAX_CENTS           (50)
#define TOTAL_CENTS_CLAMP           (250)
#define LFO_PERIOD_MS               (200U)
#define BEND_POLY_SCALE_Q16         (65536)
#define BEND_POLY_ROUND_Q16         (32768)
#define BEND_POLY_COEFF_B           (38)
#define BEND_POLY_COEFF_A           (11)
#define BEND_POLY_DIVISOR           (1024)

/* Recorder State Machine */
typedef enum {
    RECORDER_IDLE = 0,
    RECORDER_RECORDING,
    RECORDER_PLAYING
} recorder_mode_t;

/* Recorded Note Step Structure */
typedef struct {
    int8_t   note_index;   /* 0: C, 1: D, 2: E, 3: G */
    uint16_t duration_ms;  /* Key held duration */
    uint16_t rest_ms;      /* Gap between this note and next note */
} synth_step_t;

typedef struct {
    bool b_k1;
    bool b_k2;
    bool b_k3;
    bool b_k4;
} synth_keys_t;

/* Frequency Table for 4 Notes (Octave 7: C7, D7, E7, G7) - Shifted +2 Octaves */
static const uint32_t NOTE_FREQ[SYNTH_NUM_NOTES] = {
    2093U, 2349U, 2637U, 3136U
};

static const char *NOTE_NAMES[SYNTH_NUM_NOTES] = {
    "C7 (DO)", "D7 (RE)", "E7 (MI)", "G7 (SOL)"
};

/* Sequencer Memory and State */
static synth_step_t     g_sequence[SYNTH_MAX_SEQUENCE_STEPS];
static uint16_t         g_u2t_seq_count = 0U;
static recorder_mode_t  g_recorder_mode = RECORDER_IDLE;
static int8_t           g_s1t_last_played = -1;

/* Recording Trackers */
static uint32_t         g_u4t_rec_note_start_ms = 0U;
static uint32_t         g_u4t_rec_last_release_ms = 0U;
static bool             g_b_rec_is_note_active = false;
static int8_t           g_s1t_rec_current_note = -1;
static uint32_t         g_u4t_rec_blink_timer_ms = 0U;
static bool             g_b_rec_blink_led_state = false;

/* Playback Trackers */
static uint16_t         g_u2t_play_current_step = 0U;
static uint32_t         g_u4t_play_step_start_ms = 0U;
static bool             g_b_play_in_note_phase = false;

/* Sound Modulation Trackers */
static uint32_t         g_u4t_lfo_start_ms = 0U;

/* Command Arbitration & Combos */
static bool             g_b_cmd_suppress = false;
static uint32_t         g_u4t_combo_rec_start_ms = 0U;
static bool             g_b_combo_rec_taken = false;
static uint32_t         g_u4t_combo_play_start_ms = 0U;
static bool             g_b_combo_play_taken = false;

/* Helper: Sound feedback chimes */
static void synth_play_status_tone(uint32_t u4t_first_freq, uint32_t u4t_second_freq)
{
    bsp_buzzer_play_chunk(u4t_first_freq, CHIME_NOTE_VOL_RAW);
    bsp_delay_ms(CHIRP_DELAY_MS);
    if (u4t_second_freq != STATUS_TONE_NONE_FREQ)
    {
        bsp_buzzer_play_chunk(u4t_second_freq, CHIME_NOTE_VOL_RAW);
        bsp_delay_ms(CHIRP_DELAY_MS);
    }
    else
    {
        /* Single tone */
    }
    bsp_buzzer_off();
}

/* Sequencer State Transitions */
static void synth_seq_start_recording(void)
{
    g_u2t_seq_count = 0U;
    g_recorder_mode = RECORDER_RECORDING;
    g_b_rec_is_note_active = false;
    g_s1t_rec_current_note = -1;
    g_u4t_rec_blink_timer_ms = bsp_timer_get_ms();
    g_b_rec_blink_led_state = true;
    bsp_gpio_led_red_set(true);
    synth_play_status_tone(CHIME_NOTE_C6_FREQ, CHIME_NOTE_G6_FREQ);
    bsp_uart_send_string("[RECORDER] Recording Started!\r\n");
}

static void synth_seq_append_step(uint32_t u4t_now, uint16_t u2t_rest_ms)
{
    if (g_b_rec_is_note_active == true)
    {
        uint32_t u4t_dur = u4t_now - g_u4t_rec_note_start_ms;
        if (u4t_dur < MIN_NOTE_DUR_RECORD_MS)
        {
            u4t_dur = MIN_NOTE_DUR_RECORD_MS;
        }
        else if (u4t_dur > 65535U)
        {
            u4t_dur = 65535U;
        }
        else
        {
            /* Valid duration */
        }

        if (g_u2t_seq_count < SYNTH_MAX_SEQUENCE_STEPS)
        {
            g_sequence[g_u2t_seq_count].note_index = g_s1t_rec_current_note;
            g_sequence[g_u2t_seq_count].duration_ms = (uint16_t)u4t_dur;
            g_sequence[g_u2t_seq_count].rest_ms = u2t_rest_ms;
            g_u2t_seq_count++;
        }
        else
        {
            /* Sequence full */
        }

        g_b_rec_is_note_active = false;
        g_s1t_rec_current_note = -1;
    }
    else
    {
        /* No active note sounding */
    }
}

static void synth_seq_stop_recording(uint32_t u4t_now)
{
    synth_seq_append_step(u4t_now, DEF_REST_GAP_MS);
    g_recorder_mode = RECORDER_IDLE;
    bsp_buzzer_off();
    bsp_gpio_led_red_set(false);
    synth_play_status_tone(CHIME_NOTE_G6_FREQ, CHIME_NOTE_C6_FREQ);
    bsp_uart_send_string("[RECORDER] Stopped & Saved!\r\n");
}

static void synth_seq_start_playback(void)
{
    if (g_u2t_seq_count == 0U)
    {
        synth_play_status_tone(CHIME_NOTE_C4_FREQ, STATUS_TONE_NONE_FREQ);
        bsp_uart_send_string("[PLAYBACK] Memory empty!\r\n");
    }
    else
    {
        synth_play_status_tone(NOTE_FREQ[SYNTH_NOTE_INDEX_SOL], STATUS_TONE_NONE_FREQ);
        g_recorder_mode = RECORDER_PLAYING;
        g_u2t_play_current_step = 0U;
        g_b_play_in_note_phase = true;
        g_u4t_play_step_start_ms = bsp_timer_get_ms();
        bsp_uart_send_string("[PLAYBACK] Playing...\r\n");
    }
}

static void synth_seq_stop_playback(void)
{
    g_recorder_mode = RECORDER_IDLE;
    bsp_buzzer_off();
    bsp_gpio_led_red_set(false);
    bsp_uart_send_string("[PLAYBACK] Stopped!\r\n");
}

/* Consolidated Commands */
static void synth_cmd_toggle_playback(void)
{
    if (g_recorder_mode == RECORDER_PLAYING)
    {
        synth_seq_stop_playback();
    }
    else
    {
        synth_seq_start_playback();
    }
}

static void synth_cmd_toggle_recording(uint32_t u4t_now)
{
    if (g_recorder_mode == RECORDER_RECORDING)
    {
        synth_seq_stop_recording(u4t_now);
    }
    else
    {
        if (g_recorder_mode == RECORDER_PLAYING)
        {
            synth_seq_stop_playback();
        }
        else
        {
            /* IDLE mode */
        }
        synth_seq_start_recording();
    }
}

static void synth_cmd_short_press(uint32_t u4t_now)
{
    if (g_recorder_mode == RECORDER_RECORDING)
    {
        synth_seq_stop_recording(u4t_now);
    }
    else
    {
        synth_cmd_toggle_playback();
    }
}

/* Helper: Check hold duration for breadboard combos */
static bool synth_check_combo_hold(bool b_active, uint32_t *p_start_ms, bool *p_taken, uint32_t u4t_now)
{
    bool b_triggered = false;

    if (b_active == true)
    {
        if (*p_start_ms == 0U)
        {
            *p_start_ms = u4t_now;
            *p_taken = false;
        }
        else if (((u4t_now - *p_start_ms) >= COMBO_HOLD_MS) && (*p_taken == false))
        {
            *p_taken = true;
            b_triggered = true;
        }
        else
        {
            /* Holding */
        }
    }
    else
    {
        *p_start_ms = 0U;
        *p_taken = false;
    }

    return b_triggered;
}

/* Breadboard Combos & Command Arbitration */
static void synth_handle_combos(uint32_t u4t_now, const synth_keys_t *p_keys)
{
    bool b_rec_combo = ((p_keys->b_k1 == true) && (p_keys->b_k4 == true));
    bool b_play_combo = ((p_keys->b_k2 == true) && (p_keys->b_k3 == true));

    if (g_b_cmd_suppress == true)
    {
        if ((bsp_joystick_is_pressed() == false) && (b_rec_combo == false) && (b_play_combo == false))
        {
            g_b_cmd_suppress = false;
        }
        else
        {
            /* Wait for release */
        }
    }
    else
    {
        if (synth_check_combo_hold(b_rec_combo, &g_u4t_combo_rec_start_ms, &g_b_combo_rec_taken, u4t_now) == true)
        {
            synth_cmd_toggle_recording(u4t_now);
            g_b_cmd_suppress = true;
        }
        else if ((b_rec_combo == false) &&
                 (synth_check_combo_hold(b_play_combo, &g_u4t_combo_play_start_ms, &g_b_combo_play_taken, u4t_now) == true))
        {
            synth_cmd_toggle_playback();
            g_b_cmd_suppress = true;
        }
        else
        {
            /* No combo */
        }
    }
}

static int8_t synth_select_active_note(const synth_keys_t *p_keys)
{
    int8_t s1t_active_note = -1;

    if (((p_keys->b_k1 == true) && (p_keys->b_k4 == true)) ||
        ((p_keys->b_k2 == true) && (p_keys->b_k3 == true)))
    {
        s1t_active_note = -1;
    }
    else if (p_keys->b_k1 == true)
    {
        s1t_active_note = SYNTH_NOTE_INDEX_DO;
    }
    else if (p_keys->b_k2 == true)
    {
        s1t_active_note = SYNTH_NOTE_INDEX_RE;
    }
    else if (p_keys->b_k3 == true)
    {
        s1t_active_note = SYNTH_NOTE_INDEX_MI;
    }
    else if (p_keys->b_k4 == true)
    {
        s1t_active_note = SYNTH_NOTE_INDEX_SOL;
    }
    else
    {
        s1t_active_note = -1;
    }

    return s1t_active_note;
}

/* Sound Output Generator: Unified note playback with optional modulation */
static void synth_play_note(int8_t s1t_note_index, uint8_t u1t_volume_pct, bool b_modulate, uint32_t u4t_now)
{
    uint32_t u4t_freq = NOTE_FREQ[s1t_note_index];

    if (b_modulate == true)
    {
        int32_t s4t_norm_x = bsp_joystick_get_norm_x();
        int32_t s4t_norm_y = bsp_joystick_get_norm_y();
        int32_t s4t_cents_bend = (s4t_norm_x * BEND_MAX_CENTS) / 1000;
        int32_t s4t_vib_depth = 0;

        if (s4t_norm_y > 0)
        {
            s4t_vib_depth = (s4t_norm_y * VIBRATO_MAX_CENTS) / 1000;
        }
        else
        {
            s4t_vib_depth = 0;
        }

        int32_t s4t_lfo_cents = 0;
        if (s4t_vib_depth > 0)
        {
            uint32_t u4t_phase = (u4t_now - g_u4t_lfo_start_ms) % LFO_PERIOD_MS;
            if (u4t_phase < 50U)
            {
                s4t_lfo_cents = ((int32_t)u4t_phase * s4t_vib_depth) / 50;
            }
            else if (u4t_phase < 150U)
            {
                s4t_lfo_cents = ((100 - (int32_t)u4t_phase) * s4t_vib_depth) / 50;
            }
            else
            {
                s4t_lfo_cents = (((int32_t)u4t_phase - 200) * s4t_vib_depth) / 50;
            }
        }
        else
        {
            s4t_lfo_cents = 0;
        }

        int32_t s4t_total_cents = s4t_cents_bend + s4t_lfo_cents;
        if (s4t_total_cents > TOTAL_CENTS_CLAMP)
        {
            s4t_total_cents = TOTAL_CENTS_CLAMP;
        }
        else if (s4t_total_cents < -TOTAL_CENTS_CLAMP)
        {
            s4t_total_cents = -TOTAL_CENTS_CLAMP;
        }
        else
        {
            /* In range */
        }

        if (s4t_total_cents != 0)
        {
            /* Fast deterministic fixed-point pitch calculation (Q16 format) */
            int32_t s4t_c2_term = (BEND_POLY_COEFF_A * s4t_total_cents * s4t_total_cents) / BEND_POLY_DIVISOR;
            int32_t s4t_scaled = BEND_POLY_SCALE_Q16 + (BEND_POLY_COEFF_B * s4t_total_cents) + s4t_c2_term;
            int32_t s4t_calc_freq = (((int32_t)u4t_freq * s4t_scaled) + BEND_POLY_ROUND_Q16) / BEND_POLY_SCALE_Q16;

            if (s4t_calc_freq < 1)
            {
                u4t_freq = 1U;
            }
            else
            {
                u4t_freq = (uint32_t)s4t_calc_freq;
            }
        }
        else
        {
            /* Base frequency unchanged */
        }
    }
    else
    {
        /* Base note without modulation */
    }

    uint16_t u2t_volume_raw = (uint16_t)(((uint32_t)u1t_volume_pct * VOLUME_RAW_MAX) / PERCENT_SCALE);
    bsp_buzzer_play_chunk(u4t_freq, u2t_volume_raw);
}

/* State Machine Updates */
static void synth_update_recording(uint32_t u4t_now, int8_t s1t_active_note)
{
    if (g_recorder_mode == RECORDER_RECORDING)
    {
        if (s1t_active_note >= 0)
        {
            if (s1t_active_note != g_s1t_rec_current_note)
            {
                if (g_b_rec_is_note_active == true)
                {
                    synth_seq_append_step(u4t_now, TRANSITION_REST_MS);
                }
                else if (g_u2t_seq_count > 0U)
                {
                    uint32_t u4t_rest = u4t_now - g_u4t_rec_last_release_ms;
                    if (u4t_rest > MAX_REST_GAP_RECORD_MS)
                    {
                        u4t_rest = MAX_REST_GAP_RECORD_MS;
                    }
                    else
                    {
                        /* In range */
                    }
                    g_sequence[g_u2t_seq_count - 1U].rest_ms = (uint16_t)u4t_rest;
                }
                else
                {
                    /* First note */
                }

                g_b_rec_is_note_active = true;
                g_s1t_rec_current_note = s1t_active_note;
                g_u4t_rec_note_start_ms = u4t_now;
                g_u4t_lfo_start_ms = u4t_now;
            }
            else
            {
                /* Note held */
            }
        }
        else
        {
            if ((u4t_now - g_u4t_rec_blink_timer_ms) >= REC_LED_BLINK_PERIOD_MS)
            {
                g_u4t_rec_blink_timer_ms = u4t_now;
                g_b_rec_blink_led_state = !g_b_rec_blink_led_state;
                bsp_gpio_led_red_set(g_b_rec_blink_led_state);
            }
            else
            {
                /* Blink timer */
            }

            if (g_b_rec_is_note_active == true)
            {
                synth_seq_append_step(u4t_now, DEF_REST_GAP_MS);
                g_u4t_rec_last_release_ms = u4t_now;
            }
            else
            {
                /* No note active */
            }
        }

        if (g_u2t_seq_count >= SYNTH_MAX_SEQUENCE_STEPS)
        {
            synth_seq_stop_recording(u4t_now);
        }
        else
        {
            /* Sequence space available */
        }
    }
    else
    {
        /* Not recording */
    }
}

static void synth_update_playback(uint32_t u4t_now, int8_t s1t_active_note, uint8_t u1t_volume_pct)
{
    if (s1t_active_note >= 0)
    {
        synth_seq_stop_playback();
    }
    else
    {
        synth_step_t *step = &g_sequence[g_u2t_play_current_step];
        uint32_t u4t_elapsed = u4t_now - g_u4t_play_step_start_ms;

        if (g_b_play_in_note_phase == true)
        {
            bsp_gpio_led_red_set(true);
            synth_play_note(step->note_index, u1t_volume_pct, false, 0U);

            if (u4t_elapsed >= (uint32_t)step->duration_ms)
            {
                g_b_play_in_note_phase = false;
                g_u4t_play_step_start_ms = u4t_now;
                bsp_buzzer_off();
                bsp_gpio_led_red_set(false);
            }
            else
            {
                /* Sounding */
            }
        }
        else
        {
            uint32_t u4t_rest_target = (uint32_t)step->rest_ms;
            if (u4t_rest_target < PLAYBACK_MIN_GAP_MS)
            {
                u4t_rest_target = PLAYBACK_MIN_GAP_MS;
            }
            else
            {
                /* Adequate gap */
            }

            bsp_buzzer_off();
            bsp_gpio_led_red_set(false);
            bsp_delay_us(1000U);

            if (u4t_elapsed >= u4t_rest_target)
            {
                g_u2t_play_current_step++;
                if (g_u2t_play_current_step >= g_u2t_seq_count)
                {
                    synth_seq_stop_playback();
                    bsp_uart_send_string("[PLAYBACK] Done!\r\n");
                }
                else
                {
                    g_b_play_in_note_phase = true;
                    g_u4t_play_step_start_ms = u4t_now;
                }
            }
            else
            {
                /* Resting */
            }
        }
    }
}

static void synth_update_live_sound(int8_t s1t_active_note, uint8_t u1t_volume_pct, uint32_t u4t_now)
{
    if (s1t_active_note >= 0)
    {
        bsp_gpio_led_red_set(true);
        if (s1t_active_note != g_s1t_last_played)
        {
            g_u4t_lfo_start_ms = u4t_now;
            bsp_uart_send_string("[KEY] Playing: ");
            bsp_uart_send_string(NOTE_NAMES[s1t_active_note]);
            bsp_uart_send_string("\r\n");
            g_s1t_last_played = s1t_active_note;
        }
        else
        {
            /* Continues */
        }
        synth_play_note(s1t_active_note, u1t_volume_pct, true, u4t_now);
    }
    else
    {
        bsp_buzzer_off();
        if (g_recorder_mode != RECORDER_RECORDING)
        {
            bsp_gpio_led_red_set(false);
        }
        else
        {
            /* Handled by recording blinker */
        }

        g_s1t_last_played = -1;
        bsp_delay_us(1000U);
    }
}

void app_synth_init(void)
{
    bsp_uart_send_string("\r\n===============================================\r\n");
    bsp_uart_send_string("  STM32 Synthesizer & Sequence Recorder\r\n");
    bsp_uart_send_string("  HW-504: PC0=Vx, PC1=Vy, PC2=SW, PA4=Volume\r\n");
    bsp_uart_send_string("===============================================\r\n");

    /* Startup Welcome Chime: C7 -> E7 -> G7 */
    bsp_buzzer_play_chunk(NOTE_FREQ[SYNTH_NOTE_INDEX_DO], CHIME_NOTE_VOL_RAW);
    bsp_delay_ms(CHIME_DELAY_SHORT_MS);
    bsp_buzzer_play_chunk(NOTE_FREQ[SYNTH_NOTE_INDEX_MI], CHIME_NOTE_VOL_RAW);
    bsp_delay_ms(CHIME_DELAY_SHORT_MS);
    bsp_buzzer_play_chunk(NOTE_FREQ[SYNTH_NOTE_INDEX_SOL], CHIME_NOTE_VOL_FINAL);
    bsp_delay_ms(CHIME_DELAY_LONG_MS);
}

void app_synth_run(void)
{
    while (true)
    {
        uint32_t u4t_now = bsp_timer_get_ms();
        synth_keys_t keys;
        int8_t s1t_active_note;
        uint8_t u1t_volume_pct;
        joy_sw_event_t joy_evt;

        if (bsp_gpio_get_exti_flag() == true)
        {
            bsp_gpio_clear_exti_flag();
        }
        else
        {
            /* No EXTI event */
        }

        /* Periodic Services */
        bsp_adc_service(u4t_now);
        bsp_joystick_service(u4t_now);

        /* Read Inputs */
        keys.b_k1 = bsp_gpio_read_key1_do();
        keys.b_k2 = bsp_gpio_read_key2_re();
        keys.b_k3 = bsp_gpio_read_key3_mi();
        keys.b_k4 = bsp_gpio_read_key4_sol();

        /* Process Joystick Switch Events */
        joy_evt = bsp_joystick_get_event();
        if (joy_evt == JOY_SW_EVT_SHORT_PRESS)
        {
            if (g_b_cmd_suppress == false)
            {
                synth_cmd_short_press(u4t_now);
                g_b_cmd_suppress = true;
            }
            else
            {
                /* Suppressed */
            }
        }
        else if (joy_evt == JOY_SW_EVT_LONG_PRESS)
        {
            if (g_b_cmd_suppress == false)
            {
                synth_cmd_toggle_recording(u4t_now);
                g_b_cmd_suppress = true;
            }
            else
            {
                /* Suppressed */
            }
        }
        else
        {
            /* No switch event */
        }

        synth_handle_combos(u4t_now, &keys);

        s1t_active_note = synth_select_active_note(&keys);
        u1t_volume_pct = bsp_adc_get_volume_percent();

        synth_update_recording(u4t_now, s1t_active_note);

        if (g_recorder_mode == RECORDER_PLAYING)
        {
            synth_update_playback(u4t_now, s1t_active_note, u1t_volume_pct);
        }
        else
        {
            synth_update_live_sound(s1t_active_note, u1t_volume_pct, u4t_now);
        }
    }
}
