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
#include "bsp_oled.h"

/* Named Constants (Rule 5 & Rule 10) */
#define SYNTH_MAX_SEQUENCE_STEPS    (64U)
#define SYNTH_NUM_NOTES             (8U)
#define JOY_HIGH_BANK_THRESHOLD     (-350)
#define OLED_RENDER_INTERVAL_MS     (30U)

#define SYNTH_NOTE_INDEX_DO         (0)
#define SYNTH_NOTE_INDEX_RE         (1)
#define SYNTH_NOTE_INDEX_MI         (2)
#define SYNTH_NOTE_INDEX_FA         (3)
#define SYNTH_NOTE_INDEX_SOL        (4)
#define SYNTH_NOTE_INDEX_LA         (5)
#define SYNTH_NOTE_INDEX_TI         (6)
#define SYNTH_NOTE_INDEX_HIGH_DO    (7)

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
#define NOTE_RELEASE_TIME_MS        (250U)
#define COMBO_HOLD_MS               (600U)

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
    int8_t   note_index;   /* 0..7 (Do, Re, Mi, Fa, Sol, La, Ti, High Do) */
    uint16_t duration_ms;  /* Key held duration */
    uint16_t rest_ms;      /* Gap between this note and next note */
} synth_step_t;

/* Frequency Table for 8 Notes (C7 to C8: Do, Re, Mi, Fa, Sol, La, Ti, High Do) */
static const uint32_t NOTE_FREQ[SYNTH_NUM_NOTES] = {
    2093U, 2349U, 2637U, 2794U, 3136U, 3520U, 3951U, 4186U
};

/* Note Names for UART Logging & Communication */
static const char *NOTE_NAMES[SYNTH_NUM_NOTES] = {
    "C7 (DO)", "D7 (RE)", "E7 (MI)", "F7 (FA)",
    "G7 (SO)", "A7 (LA)", "B7 (TI)", "C8 (DO+)"
};

/* Sequencer Memory and State */
static synth_step_t     g_sequence[SYNTH_MAX_SEQUENCE_STEPS];
static uint16_t         g_u2t_seq_count = 0U;
static recorder_mode_t  g_recorder_mode = RECORDER_IDLE;
static int8_t           g_s1t_last_played = -1;

/* UART RX Command State & Playback Tracker */
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

/* Sound Modulation Trackers */
static uint32_t         g_u4t_lfo_start_ms = 0U;

/* Release Envelope Trackers */
static uint32_t         g_u4t_release_start_ms = 0U;
static int8_t           g_s1t_release_note = -1;
static bool             g_b_in_release = false;

/* Breadboard Combos (K1+K4: Rec, K2+K3: Play) */
static uint32_t         g_u4t_combo_rec_start_ms = 0U;
static bool             g_b_combo_rec_taken = false;
static uint32_t         g_u4t_combo_play_start_ms = 0U;
static bool             g_b_combo_play_taken = false;

/* Helper: Send unsigned decimal number over UART */
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

            bsp_uart_send_string("[RECORDER] Step ");
            synth_uart_send_dec((uint32_t)g_u2t_seq_count);
            bsp_uart_send_string(": ");
            if ((g_s1t_rec_current_note >= 0) && (g_s1t_rec_current_note < (int8_t)SYNTH_NUM_NOTES))
            {
                bsp_uart_send_string(NOTE_NAMES[g_s1t_rec_current_note]);
            }
            else
            {
                bsp_uart_send_string("UNKNOWN");
            }
            bsp_uart_send_string(" (");
            synth_uart_send_dec(u4t_dur);
            bsp_uart_send_string(" ms)\r\n");
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
    bsp_uart_send_string("[RECORDER] Stopped & Saved! Total steps: ");
    synth_uart_send_dec((uint32_t)g_u2t_seq_count);
    bsp_uart_send_string("\r\n");
}

static void synth_seq_start_playback(void)
{
    if (g_u2t_seq_count == 0U)
    {
        synth_play_status_tone(CHIME_NOTE_C4_FREQ, STATUS_TONE_NONE_FREQ);
        bsp_uart_send_string("[PLAYBACK] Memory empty! Hold K1+K4 to record first.\r\n");
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
        bsp_uart_send_string(" recorded notes...\r\n");
    }
}

static void synth_seq_stop_playback(void)
{
    g_recorder_mode = RECORDER_IDLE;
    g_u2t_last_logged_play_step = 0xFFFFU;
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

/* Helper: Process breadboard 2-key combos (K1+K4: Record, K2+K3: Play) */
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
            /* Holding combo */
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
            /* Holding combo */
        }
    }
    else
    {
        g_u4t_combo_play_start_ms = 0U;
        g_b_combo_play_taken = false;
    }
}

static int8_t synth_read_active_note(bool b_k1, bool b_k2, bool b_k3, bool b_k4)
{
    int8_t s1t_active_note = -1;

    if (((b_k1 == true) && (b_k4 == true)) || ((b_k2 == true) && (b_k3 == true)))
    {
        s1t_active_note = -1; /* Combo held: suppress single-note audio */
    }
    else
    {
        int8_t s1t_bank_offset = 0;
        int32_t s4t_norm_y = bsp_joystick_get_norm_y();

        if (s4t_norm_y <= JOY_HIGH_BANK_THRESHOLD)
        {
            s1t_bank_offset = 4; /* High Note Bank: Sol, La, Ti, High Do */
        }
        else
        {
            s1t_bank_offset = 0; /* Normal Note Bank: Do, Re, Mi, Fa */
        }

        if (b_k1 == true)
        {
            s1t_active_note = (int8_t)(SYNTH_NOTE_INDEX_DO + s1t_bank_offset);
        }
        else if (b_k2 == true)
        {
            s1t_active_note = (int8_t)(SYNTH_NOTE_INDEX_RE + s1t_bank_offset);
        }
        else if (b_k3 == true)
        {
            s1t_active_note = (int8_t)(SYNTH_NOTE_INDEX_MI + s1t_bank_offset);
        }
        else if (b_k4 == true)
        {
            s1t_active_note = (int8_t)(SYNTH_NOTE_INDEX_FA + s1t_bank_offset);
        }
        else if (g_s1t_uart_note >= 0)
        {
            uint32_t u4t_now = bsp_timer_get_ms();
            if ((u4t_now - g_u4t_uart_note_start_ms) < 300U)
            {
                s1t_active_note = g_s1t_uart_note;
            }
            else
            {
                g_s1t_uart_note = -1;
                s1t_active_note = -1;
            }
        }
        else
        {
            s1t_active_note = -1;
        }
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

            if (g_u2t_play_current_step != g_u2t_last_logged_play_step)
            {
                g_u2t_last_logged_play_step = g_u2t_play_current_step;
                bsp_uart_send_string("[PLAYBACK] Step ");
                synth_uart_send_dec((uint32_t)(g_u2t_play_current_step + 1U));
                bsp_uart_send_string(": ");
                if ((step->note_index >= 0) && (step->note_index < (int8_t)SYNTH_NUM_NOTES))
                {
                    bsp_uart_send_string(NOTE_NAMES[step->note_index]);
                }
                else
                {
                    bsp_uart_send_string("UNKNOWN");
                }
                bsp_uart_send_string(" (");
                synth_uart_send_dec((uint32_t)step->duration_ms);
                bsp_uart_send_string(" ms)\r\n");
            }
            else
            {
                /* Already logged */
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
        /* Active note pressed: cancel any release tail and play immediately */
        g_b_in_release = false;
        g_s1t_release_note = s1t_active_note;
        bsp_gpio_led_red_set(true);

        if (s1t_active_note != g_s1t_last_played)
        {
            g_u4t_lfo_start_ms = u4t_now;
            g_s1t_last_played = s1t_active_note;

            /* Transmit note to Computer via UART (115200 bps) */
            bsp_uart_send_string("[KEY] Playing: ");
            if ((s1t_active_note >= 0) && (s1t_active_note < (int8_t)SYNTH_NUM_NOTES))
            {
                bsp_uart_send_string(NOTE_NAMES[s1t_active_note]);
                bsp_uart_send_string(" (");
                synth_uart_send_dec(NOTE_FREQ[s1t_active_note]);
                bsp_uart_send_string(" Hz)");
            }
            else
            {
                bsp_uart_send_string("UNKNOWN");
            }
            bsp_uart_send_string(" | Vol: ");
            synth_uart_send_dec((uint32_t)u1t_volume_pct);
            bsp_uart_send_string("%\r\n");
        }
        else
        {
            /* Continues playing same note */
        }
        synth_play_note(s1t_active_note, u1t_volume_pct, true, u4t_now);
    }
    else
    {
        /* Key released: handle release decay tail */
        if ((g_b_in_release == false) && (g_s1t_release_note >= 0))
        {
            g_b_in_release = true;
            g_u4t_release_start_ms = u4t_now;
        }
        else
        {
            /* Release already initiated or no note was played */
        }

        if (g_b_in_release == true)
        {
            uint32_t u4t_rel_elapsed = u4t_now - g_u4t_release_start_ms;

            if (u4t_rel_elapsed < NOTE_RELEASE_TIME_MS)
            {
                uint32_t u4t_rem_time = NOTE_RELEASE_TIME_MS - u4t_rel_elapsed;
                uint32_t u4t_fade_vol = ((uint32_t)u1t_volume_pct * u4t_rem_time) / NOTE_RELEASE_TIME_MS;

                bsp_gpio_led_red_set(true);
                synth_play_note(g_s1t_release_note, (uint8_t)u4t_fade_vol, true, u4t_now);
            }
            else
            {
                /* Release tail expired */
                g_b_in_release = false;
                g_s1t_release_note = -1;
                g_s1t_last_played = -1;
                bsp_buzzer_off();

                if (g_recorder_mode != RECORDER_RECORDING)
                {
                    bsp_gpio_led_red_set(false);
                }
                else
                {
                    /* Handled by recording blinker */
                }
            }
        }
        else
        {
            /* Idle: no active note and no release tail */
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
}

/* Update OLED Virtual Piano Interface and refresh screen slices */
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
            if (s1t_active_note >= 0)
            {
                s1t_display_key = s1t_active_note;
            }
            else if (g_b_in_release == true)
            {
                s1t_display_key = g_s1t_release_note;
            }
            else
            {
                s1t_display_key = -1;
            }
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

    /* Stream 1 page slice (~2.8 ms) to OLED display */
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

            bsp_uart_send_string("[CMD] Trigger Note: ");
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
            bsp_uart_send_string("\r\n===================================================\r\n");
            bsp_uart_send_string("  STM32 Synthesizer Serial Commands (115200 bps)\r\n");
            bsp_uart_send_string("===================================================\r\n");
            bsp_uart_send_string("  '1'..'8' : Play Note (Do..C8)\r\n");
            bsp_uart_send_string("  'r'/'R'  : Toggle Recording (Start / Stop)\r\n");
            bsp_uart_send_string("  'p'/'P'  : Toggle Playback (Play / Stop)\r\n");
            bsp_uart_send_string("  'c'/'C'  : Clear Sequence Memory\r\n");
            bsp_uart_send_string("  '?'      : Show this Help Menu\r\n\r\n");
        }
        else
        {
            /* Ignore unrecognized characters */
        }
    }
}

void app_synth_init(void)
{
    bsp_uart_send_string("\r\n===================================================\r\n");
    bsp_uart_send_string("  STM32 Synthesizer (Octave 7-8 Diatonic + OLED)\r\n");
    bsp_uart_send_string("  Type '?' in Serial Monitor for Help | 115200 bps\r\n");
    bsp_uart_send_string("===================================================\r\n");

    /* Startup Welcome Chime: C7 -> E7 -> G7 */
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

        /* Read 4 Piano Keys */
        bool b_k1 = bsp_gpio_read_key1();
        bool b_k2 = bsp_gpio_read_key2();
        bool b_k3 = bsp_gpio_read_key3();
        bool b_k4 = bsp_gpio_read_key4();

        /* Process Breadboard Combos (K1+K4: Record, K2+K3: Play) */
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

        /* Update OLED Virtual Piano and Stream display slice */
        synth_update_display(u4t_now, s1t_active_note, u1t_volume_pct);

        bsp_delay_us(1000U);
    }
}
