/*******************************************************************************
 * File Name   : app_synth.c
 * Description : Application Layer - Synthesizer with On-Board Record & Playback
 * Target MCU  : STM32F411RET6
 * Standard    : Toyota Embedded MISRA-C Compliant (22 Rules)
 ******************************************************************************/

#include "app_synth.h"
#include "bsp_gpio.h"
#include "bsp_adc.h"
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

#define CHIME_NOTE_C5_FREQ          (523U)
#define CHIME_NOTE_E5_FREQ          (659U)
#define CHIME_NOTE_G5_FREQ          (784U)
#define CHIME_NOTE_C6_FREQ          (1047U)
#define CHIME_NOTE_G6_FREQ          (1568U)
#define CHIME_NOTE_C4_FREQ          (262U)
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

/* Frequency Table for 4 Notes (Octave 5: C5, D5, E5, G5) */
static const uint32_t NOTE_FREQ[SYNTH_NUM_NOTES] = {
    523U, 587U, 659U, 784U
};

static const char *NOTE_NAMES[SYNTH_NUM_NOTES] = {
    "C (DO)",
    "D (RE)",
    "E (MI)",
    "G (SOL)"
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

/* Breadboard Combo Trackers */
static uint32_t         g_u4t_combo_rec_start_ms = 0U;
static bool             g_b_combo_rec_action_taken = false;
static uint32_t         g_u4t_combo_play_start_ms = 0U;
static bool             g_b_combo_play_action_taken = false;

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
        /* Single-note status tone */
    }
    bsp_buzzer_off();
}

static void synth_chirp_rec_start(void)
{
    synth_play_status_tone(CHIME_NOTE_C6_FREQ, CHIME_NOTE_G6_FREQ);
}

static void synth_chirp_rec_stop(void)
{
    synth_play_status_tone(CHIME_NOTE_G6_FREQ, CHIME_NOTE_C6_FREQ);
}

static void synth_chirp_play_start(void)
{
    synth_play_status_tone(CHIME_NOTE_G5_FREQ, STATUS_TONE_NONE_FREQ);
}

/* Sequencer Controls */
static void synth_seq_stop_all(void)
{
    g_recorder_mode = RECORDER_IDLE;
    bsp_buzzer_off();
    bsp_gpio_led_red_set(false);
}

static void synth_seq_toggle_record(void)
{
    if (g_recorder_mode == RECORDER_RECORDING)
    {
        /* Stop Recording & Save */
        synth_seq_stop_all();
        synth_chirp_rec_stop();
        bsp_uart_send_string("[RECORDER] Stopped & Saved!\r\n");
    }
    else
    {
        /* Start Recording */
        g_u2t_seq_count = 0U;
        g_recorder_mode = RECORDER_RECORDING;
        g_b_rec_is_note_active = false;
        g_s1t_rec_current_note = -1;
        g_u4t_rec_blink_timer_ms = bsp_timer_get_ms();
        g_b_rec_blink_led_state = true;
        bsp_gpio_led_red_set(true);
        synth_chirp_rec_start();
        bsp_uart_send_string("[RECORDER] Recording Started!\r\n");
    }
}

static void synth_seq_toggle_playback(void)
{
    if (g_recorder_mode == RECORDER_PLAYING)
    {
        synth_seq_stop_all();
        bsp_uart_send_string("[PLAYBACK] Stopped!\r\n");
    }
    else
    {
        if (g_u2t_seq_count == 0U)
        {
            /* Memory empty warning beep */
            synth_play_status_tone(CHIME_NOTE_C4_FREQ, STATUS_TONE_NONE_FREQ);
            bsp_uart_send_string("[PLAYBACK] Memory empty!\r\n");
        }
        else
        {
            synth_chirp_play_start();
            g_recorder_mode = RECORDER_PLAYING;
            g_u2t_play_current_step = 0U;
            g_b_play_in_note_phase = true;
            g_u4t_play_step_start_ms = bsp_timer_get_ms();
            bsp_uart_send_string("[PLAYBACK] Playing...\r\n");
        }
    }
}

static void synth_seq_append_step(uint32_t u4t_now, uint16_t u2t_rest_ms)
{
    uint32_t u4t_dur = u4t_now - g_u4t_rec_note_start_ms;
    if (u4t_dur < MIN_NOTE_DUR_RECORD_MS)
    {
        u4t_dur = MIN_NOTE_DUR_RECORD_MS;
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

        if (g_u2t_seq_count >= SYNTH_MAX_SEQUENCE_STEPS)
        {
            synth_seq_toggle_record();
        }
        else
        {
            /* Memory has space */
        }
    }
    else
    {
        /* Memory full */
    }
}

/* Button & Combo Handlers */
static void synth_handle_combo(uint32_t u4t_current_time, const synth_keys_t *p_keys)
{
    /* Combo 1: Key 1 + Key 4 (Outer buttons) held for 0.6s -> Toggle Record */
    if ((p_keys->b_k1 == true) && (p_keys->b_k4 == true))
    {
        if (g_u4t_combo_rec_start_ms == 0U)
        {
            g_u4t_combo_rec_start_ms = u4t_current_time;
            g_b_combo_rec_action_taken = false;
        }
        else
        {
            uint32_t u4t_combo_dur = u4t_current_time - g_u4t_combo_rec_start_ms;
            if ((u4t_combo_dur >= COMBO_HOLD_MS) && (g_b_combo_rec_action_taken == false))
            {
                g_b_combo_rec_action_taken = true;
                synth_seq_toggle_record();
            }
            else
            {
                /* Holding record combo */
            }
        }
    }
    else
    {
        g_u4t_combo_rec_start_ms = 0U;
        g_b_combo_rec_action_taken = false;
    }

    /* Combo 2: Key 2 + Key 3 (Inner buttons) held for 0.6s -> Toggle Playback */
    if ((p_keys->b_k2 == true) && (p_keys->b_k3 == true))
    {
        if (g_u4t_combo_play_start_ms == 0U)
        {
            g_u4t_combo_play_start_ms = u4t_current_time;
            g_b_combo_play_action_taken = false;
        }
        else
        {
            uint32_t u4t_combo_dur = u4t_current_time - g_u4t_combo_play_start_ms;
            if ((u4t_combo_dur >= COMBO_HOLD_MS) && (g_b_combo_play_action_taken == false))
            {
                g_b_combo_play_action_taken = true;
                synth_seq_toggle_playback();
            }
            else
            {
                /* Holding playback combo */
            }
        }
    }
    else
    {
        g_u4t_combo_play_start_ms = 0U;
        g_b_combo_play_action_taken = false;
    }
}

static int8_t synth_select_active_note(const synth_keys_t *p_keys)
{
    int8_t s1t_active_note = -1;

    if (((p_keys->b_k1 == true) && (p_keys->b_k4 == true)) ||
        ((p_keys->b_k2 == true) && (p_keys->b_k3 == true)))
    {
        /* Combo mode active: ignore single notes */
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

static void synth_play_note(int8_t s1t_note_index, uint8_t u1t_volume_pct)
{
    uint32_t u4t_base_freq = NOTE_FREQ[s1t_note_index];
    uint16_t u2t_volume_raw = (uint16_t)(((uint32_t)u1t_volume_pct * VOLUME_RAW_MAX) / PERCENT_SCALE);

    bsp_buzzer_play_chunk(u4t_base_freq, u2t_volume_raw);
}

/* State Machine Updates */
static void synth_update_recording(uint32_t u4t_current_time, int8_t s1t_active_note)
{
    if (g_recorder_mode == RECORDER_RECORDING)
    {
        if (s1t_active_note >= 0)
        {
            if (g_b_rec_is_note_active == false)
            {
                /* Note Pressed */
                if (g_u2t_seq_count > 0U)
                {
                    uint32_t u4t_rest = u4t_current_time - g_u4t_rec_last_release_ms;
                    if (u4t_rest > MAX_REST_GAP_RECORD_MS)
                    {
                        u4t_rest = MAX_REST_GAP_RECORD_MS;
                    }
                    else
                    {
                        /* Rest within bounds */
                    }
                    g_sequence[g_u2t_seq_count - 1U].rest_ms = (uint16_t)u4t_rest;
                }
                else
                {
                    /* First note in sequence */
                }
                g_b_rec_is_note_active = true;
                g_s1t_rec_current_note = s1t_active_note;
                g_u4t_rec_note_start_ms = u4t_current_time;
            }
            else if (s1t_active_note != g_s1t_rec_current_note)
            {
                /* Note Changed without releasing */
                synth_seq_append_step(u4t_current_time, TRANSITION_REST_MS);
                g_s1t_rec_current_note = s1t_active_note;
                g_u4t_rec_note_start_ms = u4t_current_time;
            }
            else
            {
                /* Note is being held */
            }
        }
        else
        {
            /* LED blinks to signal RECORDING STANDBY */
            if ((u4t_current_time - g_u4t_rec_blink_timer_ms) >= REC_LED_BLINK_PERIOD_MS)
            {
                g_u4t_rec_blink_timer_ms = u4t_current_time;
                g_b_rec_blink_led_state = !g_b_rec_blink_led_state;
                bsp_gpio_led_red_set(g_b_rec_blink_led_state);
            }
            else
            {
                /* Blink timer running */
            }

            if (g_b_rec_is_note_active == true)
            {
                /* Note Released */
                synth_seq_append_step(u4t_current_time, DEF_REST_GAP_MS);
                g_b_rec_is_note_active = false;
                g_u4t_rec_last_release_ms = u4t_current_time;
            }
            else
            {
                /* No note was active */
            }
        }
    }
    else
    {
        /* Not recording */
    }
}

static void synth_update_playback(uint32_t u4t_current_time, int8_t s1t_active_note,
                                  uint8_t u1t_volume_pct)
{
    /* Any key pressed during playback immediately stops playback */
    if (s1t_active_note >= 0)
    {
        synth_seq_stop_all();
    }
    else
    {
        synth_step_t *step = &g_sequence[g_u2t_play_current_step];
        uint32_t u4t_elapsed = u4t_current_time - g_u4t_play_step_start_ms;

        if (g_b_play_in_note_phase == true)
        {
            bsp_gpio_led_red_set(true);
            synth_play_note(step->note_index, u1t_volume_pct);

            if (u4t_elapsed >= (uint32_t)step->duration_ms)
            {
                g_b_play_in_note_phase = false;
                g_u4t_play_step_start_ms = u4t_current_time;
                bsp_buzzer_off();
                bsp_gpio_led_red_set(false);
            }
            else
            {
                /* Still sounding */
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
                    synth_seq_stop_all();
                    bsp_uart_send_string("[PLAYBACK] Done!\r\n");
                }
                else
                {
                    g_b_play_in_note_phase = true;
                    g_u4t_play_step_start_ms = u4t_current_time;
                }
            }
            else
            {
                /* Still resting */
            }
        }
    }
}

static void synth_update_live_sound(int8_t s1t_active_note, uint8_t u1t_volume_pct)
{
    if (s1t_active_note >= 0)
    {
        bsp_gpio_led_red_set(true);
        synth_play_note(s1t_active_note, u1t_volume_pct);
        if (s1t_active_note != g_s1t_last_played)
        {
            bsp_uart_send_string("[KEY] Playing: ");
            bsp_uart_send_string(NOTE_NAMES[s1t_active_note]);
            bsp_uart_send_string("\r\n");
            g_s1t_last_played = s1t_active_note;
        }
        else
        {
            /* Already logged */
        }
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
            /* LED handled by recording blinker */
        }

        g_s1t_last_played = -1;
        bsp_delay_us(1000U);
    }
}

void app_synth_init(void)
{
    bsp_uart_send_string("\r\n===============================================\r\n");
    bsp_uart_send_string("  STM32 Synthesizer & Sequence Recorder\r\n");
    bsp_uart_send_string("  Controls: K1+K4=Rec, K2+K3=Play, PA4=Volume\r\n");
    bsp_uart_send_string("===============================================\r\n");

    /* Startup Welcome Chime: C5 -> E5 -> G5 */
    bsp_buzzer_play_chunk(CHIME_NOTE_C5_FREQ, CHIME_NOTE_VOL_RAW);
    bsp_delay_ms(CHIME_DELAY_SHORT_MS);
    bsp_buzzer_play_chunk(CHIME_NOTE_E5_FREQ, CHIME_NOTE_VOL_RAW);
    bsp_delay_ms(CHIME_DELAY_SHORT_MS);
    bsp_buzzer_play_chunk(CHIME_NOTE_G5_FREQ, CHIME_NOTE_VOL_FINAL);
    bsp_delay_ms(CHIME_DELAY_LONG_MS);
}

void app_synth_run(void)
{
    while (true)
    {
        uint32_t u4t_current_time = bsp_timer_get_ms();
        synth_keys_t keys;
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

        keys.b_k1 = bsp_gpio_read_key1_do();
        keys.b_k2 = bsp_gpio_read_key2_re();
        keys.b_k3 = bsp_gpio_read_key3_mi();
        keys.b_k4 = bsp_gpio_read_key4_sol();

        synth_handle_combo(u4t_current_time, &keys);
        s1t_active_note = synth_select_active_note(&keys);
        u1t_volume_pct = bsp_adc_get_volume_percent();

        synth_update_recording(u4t_current_time, s1t_active_note);

        if (g_recorder_mode == RECORDER_PLAYING)
        {
            synth_update_playback(u4t_current_time, s1t_active_note, u1t_volume_pct);
        }
        else
        {
            synth_update_live_sound(s1t_active_note, u1t_volume_pct);
        }
    }
}
