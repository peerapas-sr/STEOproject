/*******************************************************************************
 * File Name   : app_synth.c
 * Description : Application Layer - Synthesizer with Sequence Recorder
 *               Uses bsp_joystick for Pitch Bend, Vibrato, and Bank Switch
 * Target MCU  : STM32F411RET6 (Nucleo-F411RE)
 * Standard    : Toyota Embedded MISRA-C Compliant (22 Rules)
 ******************************************************************************/

#include "app_synth.h"
#include "bsp_gpio.h"
#include "bsp_adc.h"
#include "bsp_joystick.h"
#include "bsp_uart.h"
#include "bsp_buzzer.h"
#include "bsp_timer.h"
#include "bsp_oled.h"

/* Named Constants (Rule 5 & Rule 10) */
#define SYNTH_MAX_SEQUENCE_STEPS    (64U)
#define SYNTH_NUM_NOTES             (8U)
#define JOY_HIGH_BANK_THRESHOLD     (-350)
#define OLED_RENDER_INTERVAL_MS     (30U)

#define SYNTH_NOTE_INDEX_DO         (0)
#define SYNTH_NOTE_INDEX_MI         (2)
#define SYNTH_NOTE_INDEX_SOL        (4)

#define CHIME_NOTE_C4_FREQ          (262U)
#define CHIME_NOTE_C6_FREQ          (1047U)
#define CHIME_NOTE_G6_FREQ          (1568U)
#define CHIME_NOTE_VOL_RAW          (2500U)
#define CHIME_NOTE_VOL_FINAL        (3000U)
#define CHIME_DELAY_SHORT_MS        (50U)
#define CHIME_DELAY_LONG_MS         (100U)
#define CHIRP_DELAY_MS              (40U)
#define STATUS_TONE_NONE_FREQ       (0U)

#define REC_LED_BLINK_PERIOD_MS     (200U)
#define KEY_LOCKOUT_MS              (20U)
#define COMBO_HOLD_MS               (600U)
#define UART_NOTE_TRIGGER_DUR_MS    (300U)

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

/* State Machine Types */
typedef enum {
    RECORDER_IDLE = 0,
    RECORDER_RECORDING,
    RECORDER_PLAYING
} recorder_mode_t;

typedef struct {
    int8_t   note_index;
    uint16_t duration_ms;
    uint16_t rest_ms;
} synth_step_t;

/* Frequency and Note Name Tables */
static const uint32_t NOTE_FREQ[SYNTH_NUM_NOTES] = {
    2093U, 2349U, 2637U, 2794U, 3136U, 3520U, 3951U, 4186U
};

static const char *NOTE_NAMES[SYNTH_NUM_NOTES] = {
    "C7 (DO)", "D7 (RE)", "E7 (MI)", "F7 (FA)",
    "G7 (SO)", "A7 (LA)", "B7 (TI)", "C8 (DO+)"
};

/* Sequencer Memory and Application State */
static synth_step_t     g_sequence[SYNTH_MAX_SEQUENCE_STEPS];
static uint16_t         g_u2t_seq_count = 0U;
static recorder_mode_t  g_recorder_mode = RECORDER_IDLE;
static int8_t           g_s1t_last_played = -1;

/* UART State Trackers */
static int8_t           g_s1t_uart_note = -1;
static uint32_t         g_u4t_uart_note_start_ms = 0U;
static uint16_t         g_u2t_last_logged_play_step = 0xFFFFU;

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

/* Sound Modulation Tracker */
static uint32_t         g_u4t_lfo_start_ms = 0U;

/* Key Debounce Trackers (Leading-edge zero-latency with 20ms bounce lockout) */
static bool             g_b_key_state[4] = {false, false, false, false};
static uint32_t         g_u4t_key_lockout_ms[4] = {0U, 0U, 0U, 0U};

/* Breadboard Combos (K1+K4: Rec, K2+K3: Play) */
static uint32_t         g_u4t_combo_rec_start_ms = 0U;
static bool             g_b_combo_rec_taken = false;
static uint32_t         g_u4t_combo_play_start_ms = 0U;
static bool             g_b_combo_play_taken = false;

/* Helper: Send unsigned decimal integer over UART */
static void synth_uart_send_dec(uint32_t u4t_val)
{
    char buf[12];
    int32_t s4t_idx = 0;
    uint32_t u4t_rem = u4t_val;

    if (u4t_rem == 0U)
    {
        bsp_uart_send_char('0');
    }
    else
    {
        while ((u4t_rem > 0U) && (s4t_idx < 11))
        {
            buf[s4t_idx] = (char)('0' + (u4t_rem % 10U));
            s4t_idx++;
            u4t_rem = u4t_rem / 10U;
        }

        for (int32_t s4t_i = s4t_idx - 1; s4t_i >= 0; s4t_i--)
        {
            bsp_uart_send_char(buf[s4t_i]);
        }
    }
}

/* Helper: Unified Note Telemetry Output */
static void synth_uart_log_note(const char *p_prefix, int8_t s1t_note, uint32_t u4t_val, const char *p_unit)
{
    bsp_uart_send_string(p_prefix);
    if ((s1t_note >= 0) && (s1t_note < (int8_t)SYNTH_NUM_NOTES))
    {
        bsp_uart_send_string(NOTE_NAMES[s1t_note]);
    }
    else
    {
        bsp_uart_send_string("UNKNOWN");
    }
    bsp_uart_send_string(" (");
    synth_uart_send_dec(u4t_val);
    bsp_uart_send_string(p_unit);
    bsp_uart_send_string(")\r\n");
}

/* Helper: Chime and Chirp Sounds */
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

/* Sequencer Core State Machine */
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
            /* Duration within valid bounds */
        }

        if (g_u2t_seq_count < SYNTH_MAX_SEQUENCE_STEPS)
        {
            g_sequence[g_u2t_seq_count].note_index = g_s1t_rec_current_note;
            g_sequence[g_u2t_seq_count].duration_ms = (uint16_t)u4t_dur;
            g_sequence[g_u2t_seq_count].rest_ms = u2t_rest_ms;
            g_u2t_seq_count++;

            synth_uart_log_note("[RECORDER] Step Recorded: ", g_s1t_rec_current_note, u4t_dur, " ms");
        }
        else
        {
            /* Sequence memory full */
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
    bsp_uart_send_string("[RECORDER] Stopped & Saved! Total steps: ");
    synth_uart_send_dec((uint32_t)g_u2t_seq_count);
    bsp_uart_send_string("\r\n");
}

static void synth_seq_stop_playback(void)
{
    g_recorder_mode = RECORDER_IDLE;
    g_u2t_last_logged_play_step = 0xFFFFU;
    bsp_buzzer_off();
    bsp_gpio_led_red_set(false);
    bsp_uart_send_string("[PLAYBACK] Stopped!\r\n");
}

static void synth_seq_start_playback(void)
{
    if (g_u2t_seq_count == 0U)
    {
        synth_play_status_tone(CHIME_NOTE_C4_FREQ, STATUS_TONE_NONE_FREQ);
        bsp_uart_send_string("[PLAYBACK] Memory empty! Record first.\r\n");
    }
    else
    {
        synth_play_status_tone(NOTE_FREQ[SYNTH_NOTE_INDEX_SOL], STATUS_TONE_NONE_FREQ);
        g_recorder_mode = RECORDER_PLAYING;
        g_u2t_play_current_step = 0U;
        g_u2t_last_logged_play_step = 0xFFFFU;
        g_b_play_in_note_phase = true;
        g_u4t_play_step_start_ms = bsp_timer_get_ms();
        bsp_uart_send_string("[PLAYBACK] Playing ");
        synth_uart_send_dec((uint32_t)g_u2t_seq_count);
        bsp_uart_send_string(" notes...\r\n");
    }
}

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

/* Helper: Debounce 4 Piano Keys with asymmetric leading-edge and bounce lockout */
static void synth_debounce_keys(uint32_t u4t_now, bool *p_k1, bool *p_k2, bool *p_k3, bool *p_k4)
{
    bool b_raw[4];
    b_raw[0] = *p_k1;
    b_raw[1] = *p_k2;
    b_raw[2] = *p_k3;
    b_raw[3] = *p_k4;

    for (uint8_t u1t_i = 0U; u1t_i < 4U; u1t_i++)
    {
        if (b_raw[u1t_i] != g_b_key_state[u1t_i])
        {
            if ((u4t_now - g_u4t_key_lockout_ms[u1t_i]) >= KEY_LOCKOUT_MS)
            {
                g_b_key_state[u1t_i] = b_raw[u1t_i];
                g_u4t_key_lockout_ms[u1t_i] = u4t_now;
            }
            else
            {
                /* In lockout period: suppress contact bounce glitch */
            }
        }
        else
        {
            /* Key state steady */
        }
    }

    *p_k1 = g_b_key_state[0];
    *p_k2 = g_b_key_state[1];
    *p_k3 = g_b_key_state[2];
    *p_k4 = g_b_key_state[3];
}

/* Helper: Compact combo hold timer updater */
static void synth_check_combo(uint32_t u4t_now, bool b_k1, bool b_k2, bool b_k3, bool b_k4)
{
    bool b_rec_combo = ((b_k1 == true) && (b_k4 == true));
    bool b_play_combo = ((b_k2 == true) && (b_k3 == true));

    if (b_rec_combo == true)
    {
        if (g_u4t_combo_rec_start_ms == 0U)
        {
            g_u4t_combo_rec_start_ms = u4t_now;
            g_b_combo_rec_taken = false;
        }
        else if (((u4t_now - g_u4t_combo_rec_start_ms) >= COMBO_HOLD_MS) && (g_b_combo_rec_taken == false))
        {
            g_b_combo_rec_taken = true;
            synth_cmd_toggle_recording(u4t_now);
        }
        else
        {
            /* Holding record combo */
        }
    }
    else
    {
        g_u4t_combo_rec_start_ms = 0U;
        g_b_combo_rec_taken = false;
    }

    if (b_play_combo == true)
    {
        if (g_u4t_combo_play_start_ms == 0U)
        {
            g_u4t_combo_play_start_ms = u4t_now;
            g_b_combo_play_taken = false;
        }
        else if (((u4t_now - g_u4t_combo_play_start_ms) >= COMBO_HOLD_MS) && (g_b_combo_play_taken == false))
        {
            g_b_combo_play_taken = true;
            synth_cmd_toggle_playback();
        }
        else
        {
            /* Holding playback combo */
        }
    }
    else
    {
        g_u4t_combo_play_start_ms = 0U;
        g_b_combo_play_taken = false;
    }
}

/* Sample Active Key Note from Breadboard and Joystick Bank */
static int8_t synth_read_active_note(bool b_k1, bool b_k2, bool b_k3, bool b_k4)
{
    int8_t s1t_note = -1;

    if (((b_k1 == true) && (b_k4 == true)) || ((b_k2 == true) && (b_k3 == true)))
    {
        s1t_note = -1; /* Combo held: suppress single-note sound */
    }
    else
    {
        int8_t s1t_bank = 0;
        if (bsp_joystick_get_norm_y() <= JOY_HIGH_BANK_THRESHOLD)
        {
            s1t_bank = 4;
        }
        else
        {
            s1t_bank = 0;
        }

        const bool b_keys[4] = {b_k1, b_k2, b_k3, b_k4};
        for (int8_t s1t_i = 0; s1t_i < 4; s1t_i++)
        {
            if (b_keys[s1t_i] == true)
            {
                s1t_note = (int8_t)(s1t_i + s1t_bank);
                break;
            }
            else
            {
                /* Check next button */
            }
        }

        if (s1t_note < 0)
        {
            if (g_s1t_uart_note >= 0)
            {
                uint32_t u4t_now = bsp_timer_get_ms();
                if ((u4t_now - g_u4t_uart_note_start_ms) < UART_NOTE_TRIGGER_DUR_MS)
                {
                    s1t_note = g_s1t_uart_note;
                }
                else
                {
                    g_s1t_uart_note = -1;
                }
            }
            else
            {
                /* No UART trigger */
            }
        }
        else
        {
            /* Key pressed */
        }
    }

    return s1t_note;
}

/* Sound Output Generator: Unified note calculation with pitch bend and vibrato */
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
            /* In clamp limits */
        }

        if (s4t_total_cents != 0)
        {
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
            /* No modulation */
        }
    }
    else
    {
        /* Base note */
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
                        /* Rest in range */
                    }
                    g_sequence[g_u2t_seq_count - 1U].rest_ms = (uint16_t)u4t_rest;
                }
                else
                {
                    /* First step */
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
                g_b_rec_blink_led_state = (g_b_rec_blink_led_state == false);
                bsp_gpio_led_red_set(g_b_rec_blink_led_state);
            }
            else
            {
                /* Blink timer running */
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

            if (g_u2t_play_current_step != g_u2t_last_logged_play_step)
            {
                g_u2t_last_logged_play_step = g_u2t_play_current_step;
                synth_uart_log_note("[PLAYBACK] Step: ", step->note_index, (uint32_t)step->duration_ms, " ms");
            }
            else
            {
                /* Step already logged */
            }

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
            g_s1t_last_played = s1t_active_note;
            synth_uart_log_note("[KEY] Playing: ", s1t_active_note, NOTE_FREQ[s1t_active_note], " Hz");
        }
        else
        {
            /* Note continues */
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
            /* Keep blinking during recording */
        }
        g_s1t_last_played = -1;
    }
}

/* OLED Virtual Piano UI Service */
static void synth_update_display(uint32_t u4t_now, int8_t s1t_active_note, uint8_t u1t_volume_pct)
{
    static uint32_t u4t_last_render_ms = 0U;

    if ((u4t_now - u4t_last_render_ms) >= OLED_RENDER_INTERVAL_MS)
    {
        u4t_last_render_ms = u4t_now;

        int32_t s4t_norm_x = bsp_joystick_get_norm_x();
        int32_t s4t_norm_y = bsp_joystick_get_norm_y();
        bool b_high_bank = (s4t_norm_y <= JOY_HIGH_BANK_THRESHOLD);
        int32_t s4t_cents = (s4t_norm_x * BEND_MAX_CENTS) / 1000;
        int8_t s1t_display_key = -1;
        const char *p_mode_str = "[LIVE]";

        if (g_recorder_mode == RECORDER_RECORDING)
        {
            if (g_b_rec_blink_led_state == true)
            {
                p_mode_str = "[REC]";
            }
            else
            {
                p_mode_str = "[   ]";
            }
            s1t_display_key = s1t_active_note;
        }
        else if (g_recorder_mode == RECORDER_PLAYING)
        {
            p_mode_str = "[PLAY]";
            if (g_b_play_in_note_phase == true)
            {
                s1t_display_key = g_sequence[g_u2t_play_current_step].note_index;
            }
            else
            {
                s1t_display_key = -1;
            }
        }
        else
        {
            p_mode_str = "[LIVE]";
            s1t_display_key = s1t_active_note;
        }

        bsp_oled_clear_buffer();
        bsp_oled_render_header(p_mode_str, b_high_bank, u1t_volume_pct, s1t_display_key, s4t_cents);
        bsp_oled_render_pitch_gauge(s4t_norm_x);
        bsp_oled_render_piano_keyboard(s1t_display_key);
    }
    else
    {
        /* Refresh interval not elapsed */
    }

    bsp_oled_service(u4t_now);
}

/* Process Incoming UART Serial Commands */
static void synth_handle_uart_rx(uint32_t u4t_now)
{
    while (bsp_uart_has_rx_char() == true)
    {
        char c = bsp_uart_get_rx_char();

        if ((c >= '1') && (c <= '8'))
        {
            int8_t s1t_note = (int8_t)(c - '1');
            g_s1t_uart_note = s1t_note;
            g_u4t_uart_note_start_ms = u4t_now;

            bsp_uart_send_string("[CMD] Trigger: ");
            bsp_uart_send_string(NOTE_NAMES[s1t_note]);
            bsp_uart_send_string("\r\n");
        }
        else if ((c == 'r') || (c == 'R'))
        {
            synth_cmd_toggle_recording(u4t_now);
        }
        else if ((c == 'p') || (c == 'P'))
        {
            synth_cmd_toggle_playback();
        }
        else if ((c == 'c') || (c == 'C'))
        {
            g_u2t_seq_count = 0U;
            bsp_uart_send_string("[RECORDER] Memory cleared!\r\n");
        }
        else if (c == '?')
        {
            bsp_uart_send_string("\r\n=== STM32 Synthesizer Serial Commands ===\r\n");
            bsp_uart_send_string("  '1'..'8' : Play Note (Do..C8)\r\n");
            bsp_uart_send_string("  'r'/'R'  : Toggle Recording\r\n");
            bsp_uart_send_string("  'p'/'P'  : Toggle Playback\r\n");
            bsp_uart_send_string("  'c'/'C'  : Clear Sequence Memory\r\n");
            bsp_uart_send_string("  '?'      : Show Help Menu\r\n\r\n");
        }
        else
        {
            /* Ignore unrecognized characters */
        }
    }
}

/* Public Application Lifecycle */
void app_synth_init(void)
{
    bsp_uart_send_string("\r\n===================================================\r\n");
    bsp_uart_send_string("  STM32 Synthesizer (Octave 7-8 Diatonic + OLED)\r\n");
    bsp_uart_send_string("  Type '?' in Serial Monitor for Help | 115200 bps\r\n");
    bsp_uart_send_string("===================================================\r\n");

    bsp_buzzer_play_chunk(NOTE_FREQ[SYNTH_NOTE_INDEX_DO], CHIME_NOTE_VOL_RAW);
    bsp_delay_ms(CHIME_DELAY_SHORT_MS);
    bsp_buzzer_play_chunk(NOTE_FREQ[SYNTH_NOTE_INDEX_MI], CHIME_NOTE_VOL_RAW);
    bsp_delay_ms(CHIME_DELAY_SHORT_MS);
    bsp_buzzer_play_chunk(NOTE_FREQ[SYNTH_NOTE_INDEX_SOL], CHIME_NOTE_VOL_FINAL);
    bsp_delay_ms(CHIME_DELAY_LONG_MS);
    bsp_buzzer_off();
}

void app_synth_run(void)
{
    while (true)
    {
        uint32_t u4t_now = bsp_timer_get_ms();
        joy_sw_event_t joy_evt;
        int8_t s1t_active_note;
        uint8_t u1t_volume_pct;

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
        synth_handle_uart_rx(u4t_now);

        /* Process Joystick Switch Events */
        joy_evt = bsp_joystick_get_event();
        if (joy_evt == JOY_SW_EVT_SHORT_PRESS)
        {
            synth_cmd_short_press(u4t_now);
        }
        else if (joy_evt == JOY_SW_EVT_LONG_PRESS)
        {
            synth_cmd_toggle_recording(u4t_now);
        }
        else
        {
            /* No switch event */
        }

        /* Read 4 Piano Keys with Zero-Latency Debounce Filter */
        bool b_k1 = bsp_gpio_read_key1();
        bool b_k2 = bsp_gpio_read_key2();
        bool b_k3 = bsp_gpio_read_key3();
        bool b_k4 = bsp_gpio_read_key4();

        synth_debounce_keys(u4t_now, &b_k1, &b_k2, &b_k3, &b_k4);

        synth_check_combo(u4t_now, b_k1, b_k2, b_k3, b_k4);

        s1t_active_note = synth_read_active_note(b_k1, b_k2, b_k3, b_k4);
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

        synth_update_display(u4t_now, s1t_active_note, u1t_volume_pct);

        bsp_delay_us(1000U);
    }
}
