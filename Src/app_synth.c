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
#define SYNTH_NUM_OCTAVES           (3U)
#define SYNTH_NUM_NOTES             (4U)
#define SYNTH_OCTAVE_OFFSET         (4U)

#define SYNTH_NOTE_INDEX_DO         (0)
#define SYNTH_NOTE_INDEX_RE         (1)
#define SYNTH_NOTE_INDEX_MI         (2)
#define SYNTH_NOTE_INDEX_SOL        (3)

#define UART_NOTE_PLAY_MS           (250U)
#define UART_NOTE_TICK_MS           (5U)
#define CHIME_NOTE_C5_FREQ          (523U)
#define CHIME_NOTE_E5_FREQ          (659U)
#define CHIME_NOTE_G5_FREQ          (784U)
#define CHIME_NOTE_C6_FREQ          (1047U)
#define CHIME_NOTE_G6_FREQ          (1568U)
#define CHIME_NOTE_VOL_RAW          (2500U)
#define CHIME_NOTE_VOL_FINAL        (3000U)
#define CHIME_DELAY_SHORT_MS        (50U)
#define CHIME_DELAY_LONG_MS         (100U)
#define CHIRP_DELAY_MS              (40U)
#define STATUS_TONE_NONE_FREQ       (0U)

#define BTN_DEBOUNCE_MS             (25U)
#define BTN_LONG_PRESS_MS           (800U)
#define COMBO_HOLD_MS               (600U)
#define REC_LED_BLINK_PERIOD_MS     (200U)

#define PLAYBACK_MIN_GAP_MS         (40U)
#define MIN_NOTE_DUR_RECORD_MS      (50U)
#define MIN_NOTE_DUR_RELEASE_MS     (60U)
#define MAX_REST_GAP_RECORD_MS      (3000U)
#define DEF_REST_GAP_MS             (80U)
#define TRANSITION_REST_MS          (30U)
#define FINAL_NOTE_REST_MS          (100U)
#define VOLUME_RAW_MAX              (4095U)
#define PERCENT_SCALE               (100U)
#define VOLUME_REPORT_PERIOD_MS     (200U)
#define VOLUME_REPORT_DELTA_PCT     (5)

/* Octave Definitions */
typedef enum {
    OCTAVE_LOW = 4U,   /* Octave 4: Bass */
    OCTAVE_MID = 5U,   /* Octave 5: Normal */
    OCTAVE_HIGH = 6U   /* Octave 6: Treble */
} synth_octave_t;

/* Recorder State Machine */
typedef enum {
    RECORDER_IDLE = 0,
    RECORDER_RECORDING,
    RECORDER_PLAYING
} recorder_mode_t;

/* Recorded Note Step Structure */
typedef struct {
    int8_t   note_index;   /* 0: C, 1: D, 2: E, 3: G */
    uint8_t  octave;       /* Octave (4, 5, or 6) */
    uint16_t duration_ms;  /* Key held duration */
    uint16_t rest_ms;      /* Gap between this note and next note */
} synth_step_t;

typedef struct {
    bool b_k1;
    bool b_k2;
    bool b_k3;
    bool b_k4;
} synth_keys_t;

/* Frequency Table for 4 Notes across 3 Octaves */
static const uint32_t NOTE_FREQ[SYNTH_NUM_OCTAVES][SYNTH_NUM_NOTES] = {
    { 262U, 294U, 330U, 392U },    /* Octave 4: C4, D4, E4, G4 */
    { 523U, 587U, 659U, 784U },    /* Octave 5: C5, D5, E5, G5 */
    { 1047U, 1175U, 1319U, 1568U }  /* Octave 6: C6, D6, E6, G6 */
};

static const char *NOTE_NAMES[SYNTH_NUM_NOTES] = {
    "C (DO)",
    "D (RE)",
    "E (MI)",
    "G (SOL)"
};

static const char *UART_NOTE_MESSAGES[SYNTH_NUM_NOTES] = {
    "[UART CMD] Note 1: C (DO)\r\n",
    "[UART CMD] Note 2: D (RE)\r\n",
    "[UART CMD] Note 3: E (MI)\r\n",
    "[UART CMD] Note 4: G (SOL)\r\n"
};

/* Current Synthesizer Live States (Hungarian prefix) */
static synth_octave_t   g_current_octave = OCTAVE_MID;
static int8_t           g_s1t_last_played = -1;

/* UART Note Trigger duration in milliseconds */
static uint32_t         g_u4t_uart_note_duration_ms = 0U;
static int8_t           g_s1t_uart_note_index = -1;

/* Sequencer Memory and Control */
static synth_step_t     g_sequence[SYNTH_MAX_SEQUENCE_STEPS];
static uint16_t         g_u2t_seq_count = 0U;
static recorder_mode_t  g_recorder_mode = RECORDER_IDLE;

/* Recording Trackers */
static uint32_t         g_u4t_rec_note_start_ms = 0U;
static uint32_t         g_u4t_rec_last_release_ms = 0U;
static bool             g_b_rec_is_note_active = false;
static int8_t           g_s1t_rec_current_note = -1;
static uint8_t          g_u1t_rec_current_octave = 5U;
static uint32_t         g_u4t_rec_blink_timer_ms = 0U;
static bool             g_b_rec_blink_led_state = false;

/* Playback Trackers */
static uint16_t         g_u2t_play_current_step = 0U;
static uint32_t         g_u4t_play_step_start_ms = 0U;
static bool             g_b_play_in_note_phase = false;

/* Breadboard Combo Trackers (Directly on 4 buttons):
 * - Key 1 + Key 4 (Outer buttons): Toggle Record
 * - Key 2 + Key 3 (Inner buttons): Toggle Playback
 */
static uint32_t         g_u4t_combo_rec_start_ms = 0U;
static bool             g_b_combo_rec_action_taken = false;
static uint32_t         g_u4t_combo_play_start_ms = 0U;
static bool             g_b_combo_play_action_taken = false;

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

/* Helper: Send unsigned decimal number over UART without stdlib printf */
static void synth_uart_send_dec(uint32_t val)
{
    char buf[12] = { 0 };
    int32_t s4t_idx = 0;
    uint32_t u4t_rem = val;

    if (u4t_rem == 0U)
    {
        bsp_uart_send_string("0");
    }
    else
    {
        while (u4t_rem > 0U)
        {
            buf[s4t_idx] = (char)('0' + (u4t_rem % 10U));
            s4t_idx++;
            u4t_rem /= 10U;
        }
        for (int32_t s4t_i = s4t_idx - 1; s4t_i >= 0; s4t_i--)
        {
            bsp_uart_send_char(buf[s4t_i]);
        }
    }
}

/* Print Help Banner */
static void synth_print_help(void)
{
    bsp_uart_send_string("\r\n===================================================\r\n");
    bsp_uart_send_string("  STM32 Synthesizer with On-Board Sequence Recorder\r\n");
    bsp_uart_send_string("  Toyota MISRA-C Compliant | Zero-Polling\r\n");
    bsp_uart_send_string("===================================================\r\n");
    bsp_uart_send_string("[HARDWARE CONTROLS (DIRECTLY ON BREADBOARD)]:\r\n");
    bsp_uart_send_string("  - Key 1-4    : C (PA10), D (PB3), E (PB5), G (PB4)\r\n");
    bsp_uart_send_string("  - Key 1 + Key 4 (Hold 0.6s) : START / STOP RECORDING!\r\n");
    bsp_uart_send_string("  - Key 2 + Key 3 (Hold 0.6s) : PLAY / STOP PLAYBACK!\r\n");
    bsp_uart_send_string("  - Press Any Key while Playing: STOP PLAYBACK!\r\n");
    bsp_uart_send_string("  - Potentiometer (PA4)       : Volume Control (0-100%)\r\n");
    bsp_uart_send_string("[UART COMMANDS (OPTIONAL)] (115200 bps):\r\n");
    bsp_uart_send_string("  - '1' - '4'  : Play note directly\r\n");
    bsp_uart_send_string("  - 'r' / 'R'  : Toggle Record (Start / Stop)\r\n");
    bsp_uart_send_string("  - 'p' / 'P'  : Toggle Playback (Play / Stop)\r\n");
    bsp_uart_send_string("  - 'c' / 'C'  : Clear recorded sequence\r\n");
    bsp_uart_send_string("  - 'l' / 'L'  : List recorded sequence table\r\n");
    bsp_uart_send_string("  - 'o' / 'O'  : Cycle Octave (4, 5, 6)\r\n");
    bsp_uart_send_string("  - '?'        : Show this Help Menu\r\n\r\n");
}

/* List recorded sequence */
static void synth_seq_list(void)
{
    bsp_uart_send_string("\r\n--- Recorded Sequence List ---\r\n");
    if (g_u2t_seq_count == 0U)
    {
        bsp_uart_send_string("[EMPTY] No notes recorded. Hold Key 1 + Key 4 to start recording!\r\n");
    }
    else
    {
        for (uint16_t u2t_i = 0U; u2t_i < g_u2t_seq_count; u2t_i++)
        {
            bsp_uart_send_string("Step ");
            synth_uart_send_dec((uint32_t)(u2t_i + 1U));
            bsp_uart_send_string(": ");
            if ((g_sequence[u2t_i].note_index >= 0) && (g_sequence[u2t_i].note_index < 4))
            {
                bsp_uart_send_string(NOTE_NAMES[g_sequence[u2t_i].note_index]);
            }
            else
            {
                bsp_uart_send_string("UNKNOWN");
            }
            bsp_uart_send_string(" [Oct ");
            synth_uart_send_dec((uint32_t)g_sequence[u2t_i].octave);
            bsp_uart_send_string("] | Note: ");
            synth_uart_send_dec((uint32_t)g_sequence[u2t_i].duration_ms);
            bsp_uart_send_string(" ms | Rest: ");
            synth_uart_send_dec((uint32_t)g_sequence[u2t_i].rest_ms);
            bsp_uart_send_string(" ms\r\n");
        }
        bsp_uart_send_string("Total: ");
        synth_uart_send_dec((uint32_t)g_u2t_seq_count);
        bsp_uart_send_string(" / 64 notes\r\n------------------------------\r\n");
    }
}

/* Start Recording (On-Board) */
static void synth_seq_start_record(void)
{
    g_u2t_seq_count = 0U;
    g_recorder_mode = RECORDER_RECORDING;
    g_b_rec_is_note_active = false;
    g_s1t_rec_current_note = -1;
    g_u4t_rec_last_release_ms = bsp_timer_get_ms();
    g_u4t_rec_blink_timer_ms = bsp_timer_get_ms();
    g_b_rec_blink_led_state = true;

    synth_play_status_tone(CHIME_NOTE_C6_FREQ, CHIME_NOTE_G6_FREQ);
    bsp_uart_send_string("\r\n[RECORDER] >>> RECORDING STARTED! Play notes on Key 1-4 <<<\r\n");
    bsp_uart_send_string("[RECORDER] Click Blue Button when finished to save.\r\n");
}

static uint32_t synth_seq_append_note(uint32_t u4t_now, uint32_t u4t_min_duration,
                                      uint16_t u2t_rest_ms)
{
    uint32_t u4t_duration = 0U;

    if (g_u2t_seq_count < SYNTH_MAX_SEQUENCE_STEPS)
    {
        u4t_duration = u4t_now - g_u4t_rec_note_start_ms;
        if (u4t_duration < u4t_min_duration)
        {
            u4t_duration = u4t_min_duration;
        }
        else
        {
            /* Duration is sufficient */
        }
        g_sequence[g_u2t_seq_count].note_index = g_s1t_rec_current_note;
        g_sequence[g_u2t_seq_count].octave = g_u1t_rec_current_octave;
        g_sequence[g_u2t_seq_count].duration_ms = (uint16_t)u4t_duration;
        g_sequence[g_u2t_seq_count].rest_ms = u2t_rest_ms;
        g_u2t_seq_count++;
    }
    else
    {
        /* Sequence memory is full */
    }

    return u4t_duration;
}

/* Stop Recording (On-Board) */
static void synth_seq_stop_record(void)
{
    if ((g_b_rec_is_note_active == true) && (g_u2t_seq_count < SYNTH_MAX_SEQUENCE_STEPS))
    {
        uint32_t u4t_now = bsp_timer_get_ms();
        (void)synth_seq_append_note(u4t_now, MIN_NOTE_DUR_RELEASE_MS, FINAL_NOTE_REST_MS);
        g_b_rec_is_note_active = false;
    }
    else
    {
        /* No active note hanging */
    }
    g_recorder_mode = RECORDER_IDLE;
    bsp_gpio_led_red_set(false);

    synth_play_status_tone(CHIME_NOTE_G6_FREQ, CHIME_NOTE_C6_FREQ);
    bsp_uart_send_string("\r\n[RECORDER] === RECORDING STOPPED & SAVED ===\r\n");
    bsp_uart_send_string("[RECORDER] Total notes saved: ");
    synth_uart_send_dec((uint32_t)g_u2t_seq_count);
    bsp_uart_send_string("\r\n[RECORDER] Click Blue Button to playback!\r\n");
}

/* Toggle Record */
static void synth_seq_toggle_record(void)
{
    if (g_recorder_mode == RECORDER_RECORDING)
    {
        synth_seq_stop_record();
    }
    else
    {
        if (g_recorder_mode == RECORDER_PLAYING)
        {
            g_recorder_mode = RECORDER_IDLE;
            bsp_buzzer_off();
            bsp_gpio_led_red_set(false);
        }
        else
        {
            /* Idle state */
        }
        synth_seq_start_record();
    }
}

/* Start Playback (On-Board) */
static void synth_seq_start_playback(void)
{
    if (g_u2t_seq_count == 0U)
    {
        bsp_uart_send_string("\r\n[PLAYBACK] Memory is empty! Hold Key 1 + Key 4 to record first.\r\n");
        /* Short low beep to signal empty memory */
        bsp_buzzer_play_chunk(262U, 2000U);
        bsp_delay_ms(80U);
        bsp_buzzer_off();
    }
    else
    {
        synth_play_status_tone(CHIME_NOTE_E5_FREQ, STATUS_TONE_NONE_FREQ);
        g_recorder_mode = RECORDER_PLAYING;
        g_u2t_play_current_step = 0U;
        g_b_play_in_note_phase = true;
        g_u4t_play_step_start_ms = bsp_timer_get_ms();
        bsp_uart_send_string("\r\n[PLAYBACK] >>> PLAYBACK STARTED! (Playing ");
        synth_uart_send_dec((uint32_t)g_u2t_seq_count);
        bsp_uart_send_string(" notes) <<<\r\n");
    }
}

/* Stop Playback */
static void synth_seq_stop_playback(void)
{
    g_recorder_mode = RECORDER_IDLE;
    bsp_buzzer_off();
    bsp_gpio_led_red_set(false);
    bsp_uart_send_string("[PLAYBACK] Playback Stopped.\r\n");
}

/* Toggle Playback */
static void synth_seq_toggle_playback(void)
{
    if (g_recorder_mode == RECORDER_PLAYING)
    {
        synth_seq_stop_playback();
    }
    else
    {
        if (g_recorder_mode == RECORDER_RECORDING)
        {
            synth_seq_stop_record();
        }
        else
        {
            /* Idle state */
        }
        synth_seq_start_playback();
    }
}

void app_synth_init(void)
{
    synth_print_help();

    /* Startup Welcome Chime */
    bsp_buzzer_play_chunk(CHIME_NOTE_C5_FREQ, CHIME_NOTE_VOL_RAW);
    bsp_delay_ms(CHIME_DELAY_SHORT_MS);
    bsp_buzzer_play_chunk(CHIME_NOTE_E5_FREQ, CHIME_NOTE_VOL_RAW);
    bsp_delay_ms(CHIME_DELAY_SHORT_MS);
    bsp_buzzer_play_chunk(CHIME_NOTE_G5_FREQ, CHIME_NOTE_VOL_FINAL);
    bsp_delay_ms(CHIME_DELAY_LONG_MS);
}

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

static void synth_handle_uart(void)
{
    if (bsp_uart_has_rx_char() == true)
    {
        char u1t_cmd = bsp_uart_get_rx_char();
        if ((u1t_cmd >= '1') && (u1t_cmd <= '4'))
        {
            uint8_t u1t_note_index = (uint8_t)(u1t_cmd - '1');
            g_s1t_uart_note_index = (int8_t)u1t_note_index;
            g_u4t_uart_note_duration_ms = UART_NOTE_PLAY_MS;
            bsp_uart_send_string(UART_NOTE_MESSAGES[u1t_note_index]);
        }
        else
        {
            switch (u1t_cmd)
            {
                case 'r':
                case 'R':
                    synth_seq_toggle_record();
                    break;
                case 'p':
                case 'P':
                    synth_seq_toggle_playback();
                    break;
                case 'c':
                case 'C':
                    g_u2t_seq_count = 0U;
                    bsp_uart_send_string("[RECORDER] Sequence memory cleared!\r\n");
                    break;
                case 'l':
                case 'L':
                    synth_seq_list();
                    break;
                case '?':
                    synth_print_help();
                    break;
                case 'o':
                case 'O':
                    if (g_current_octave == OCTAVE_LOW)
                    {
                        g_current_octave = OCTAVE_MID;
                        bsp_uart_send_string("[OCTAVE] Switched to OCTAVE 5 (Normal)\r\n");
                    }
                    else if (g_current_octave == OCTAVE_MID)
                    {
                        g_current_octave = OCTAVE_HIGH;
                        bsp_uart_send_string("[OCTAVE] Switched to OCTAVE 6 (Treble)\r\n");
                    }
                    else
                    {
                        g_current_octave = OCTAVE_LOW;
                        bsp_uart_send_string("[OCTAVE] Switched to OCTAVE 4 (Bass)\r\n");
                    }
                    break;
                default:
                    /* Ignore other characters */
                    break;
            }
        }
    }
    else
    {
        /* No incoming UART data */
    }
}

static int8_t synth_select_active_note(const synth_keys_t *p_keys)
{
    int8_t s1t_active_note = -1;

    if (((p_keys->b_k1 == true) && (p_keys->b_k4 == true)) ||
        ((p_keys->b_k2 == true) && (p_keys->b_k3 == true)))
    {
        /* In Combo mode: mute note to allow clean trigger */
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
    else if (g_u4t_uart_note_duration_ms > 0U)
    {
        s1t_active_note = g_s1t_uart_note_index;
        if (g_u4t_uart_note_duration_ms >= UART_NOTE_TICK_MS)
        {
            g_u4t_uart_note_duration_ms -= UART_NOTE_TICK_MS;
        }
        else
        {
            g_u4t_uart_note_duration_ms = 0U;
        }
    }
    else
    {
        s1t_active_note = -1;
    }

    return s1t_active_note;
}

static void synth_report_volume(uint32_t u4t_current_time, uint8_t u1t_volume_pct)
{
    static uint8_t s_u1t_prev_vol = 255U;
    static uint32_t s_u4t_vol_timer = 0U;

    if ((u4t_current_time - s_u4t_vol_timer) >= VOLUME_REPORT_PERIOD_MS)
    {
        int32_t s4t_vdiff = (int32_t)u1t_volume_pct - (int32_t)s_u1t_prev_vol;
        if ((s4t_vdiff >= VOLUME_REPORT_DELTA_PCT) ||
            (s4t_vdiff <= -VOLUME_REPORT_DELTA_PCT))
        {
            s_u4t_vol_timer = u4t_current_time;
            s_u1t_prev_vol = u1t_volume_pct;
            bsp_uart_send_string("[VOL] ");
            synth_uart_send_dec((uint32_t)u1t_volume_pct);
            bsp_uart_send_string("%\r\n");
        }
        else
        {
            /* Small change */
        }
    }
    else
    {
        /* Throttle period */
    }
}

static void synth_play_note(uint8_t u1t_octave, int8_t s1t_note_index,
                            uint8_t u1t_volume_pct)
{
    uint8_t u1t_oct_idx = 1U;
    uint32_t u4t_frequency;
    uint16_t u2t_volume_raw;

    bsp_gpio_led_red_set(true);
    if ((u1t_octave >= (uint8_t)OCTAVE_LOW) && (u1t_octave <= (uint8_t)OCTAVE_HIGH))
    {
        u1t_oct_idx = u1t_octave - SYNTH_OCTAVE_OFFSET;
    }
    else
    {
        u1t_oct_idx = 1U;
    }
    u4t_frequency = NOTE_FREQ[u1t_oct_idx][(uint8_t)s1t_note_index];
    u2t_volume_raw = (uint16_t)(((uint32_t)u1t_volume_pct * VOLUME_RAW_MAX) /
                                PERCENT_SCALE);
    bsp_buzzer_play_chunk(u4t_frequency, u2t_volume_raw);
}

static void synth_update_recording(uint32_t u4t_current_time, int8_t s1t_active_note)
{
    if (g_recorder_mode == RECORDER_RECORDING)
    {
        if (s1t_active_note >= 0)
        {
            if (g_b_rec_is_note_active == false)
            {
                /* Note Just Pressed */
                if (g_u2t_seq_count > 0U)
                {
                    uint32_t u4t_rest = u4t_current_time - g_u4t_rec_last_release_ms;
                    if (u4t_rest > MAX_REST_GAP_RECORD_MS)
                    {
                        u4t_rest = MAX_REST_GAP_RECORD_MS;
                    }
                    else
                    {
                        /* Gap within bounds */
                    }
                    g_sequence[g_u2t_seq_count - 1U].rest_ms = (uint16_t)u4t_rest;
                }
                else
                {
                    /* First note */
                }
                g_b_rec_is_note_active = true;
                g_s1t_rec_current_note = s1t_active_note;
                g_u1t_rec_current_octave = (uint8_t)g_current_octave;
                g_u4t_rec_note_start_ms = u4t_current_time;
            }
            else if (s1t_active_note != g_s1t_rec_current_note)
            {
                /* Note Changed without releasing */
                (void)synth_seq_append_note(u4t_current_time, MIN_NOTE_DUR_RECORD_MS,
                                            TRANSITION_REST_MS);
                g_s1t_rec_current_note = s1t_active_note;
                g_u1t_rec_current_octave = (uint8_t)g_current_octave;
                g_u4t_rec_note_start_ms = u4t_current_time;
            }
            else
            {
                /* Note is being held */
            }
        }
        else
        {
            /* No note pressed -> LED blinks to visually signal RECORDING STANDBY */
            if ((u4t_current_time - g_u4t_rec_blink_timer_ms) >= REC_LED_BLINK_PERIOD_MS)
            {
                g_u4t_rec_blink_timer_ms = u4t_current_time;
                g_b_rec_blink_led_state = !g_b_rec_blink_led_state;
                bsp_gpio_led_red_set(g_b_rec_blink_led_state);
            }
            else
            {
                /* Waiting for blink period */
            }

            if (g_b_rec_is_note_active == true)
            {
                uint32_t u4t_duration;

                /* Note Released */
                u4t_duration = synth_seq_append_note(u4t_current_time, MIN_NOTE_DUR_RECORD_MS,
                                                     DEF_REST_GAP_MS);
                if (u4t_duration > 0U)
                {
                    bsp_uart_send_string("[RECORDER] Step ");
                    synth_uart_send_dec((uint32_t)g_u2t_seq_count);
                    bsp_uart_send_string(": ");
                    bsp_uart_send_string(NOTE_NAMES[g_s1t_rec_current_note]);
                    bsp_uart_send_string(" (");
                    synth_uart_send_dec(u4t_duration);
                    bsp_uart_send_string(" ms)\r\n");

                    if (g_u2t_seq_count >= SYNTH_MAX_SEQUENCE_STEPS)
                    {
                        bsp_uart_send_string("[RECORDER] Sequence memory full!\r\n");
                        synth_seq_stop_record();
                    }
                    else
                    {
                        /* Memory still available */
                    }
                }
                else
                {
                    /* Memory full */
                }
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
        /* Not in recording mode */
    }
}

static void synth_update_playback(uint32_t u4t_current_time, int8_t s1t_active_note,
                                  uint8_t u1t_volume_pct)
{
    if (s1t_active_note >= 0)
    {
        /* If user presses any live key during playback, immediately stop playback */
        synth_seq_stop_playback();
    }
    else
    {
        synth_step_t *p_step = &g_sequence[g_u2t_play_current_step];
        uint32_t u4t_elapsed = u4t_current_time - g_u4t_play_step_start_ms;

        if (g_b_play_in_note_phase == true)
        {
            /* Note Sounding Phase */
            synth_play_note(p_step->octave, p_step->note_index, u1t_volume_pct);
            if (u4t_elapsed >= (uint32_t)p_step->duration_ms)
            {
                /* Note duration reached -> Transition to Rest Phase */
                g_b_play_in_note_phase = false;
                g_u4t_play_step_start_ms = u4t_current_time;
                bsp_buzzer_off();
                bsp_gpio_led_red_set(false);
            }
            else
            {
                /* Still in note sounding duration */
            }
        }
        else
        {
            uint32_t u4t_rest_target = (uint32_t)p_step->rest_ms;

            /* Rest/Silence Phase between notes */
            bsp_buzzer_off();
            bsp_gpio_led_red_set(false);
            bsp_delay_us(1000U);
            if (u4t_rest_target < PLAYBACK_MIN_GAP_MS)
            {
                u4t_rest_target = PLAYBACK_MIN_GAP_MS;
            }
            else
            {
                /* Target rest is adequate */
            }

            if (u4t_elapsed >= u4t_rest_target)
            {
                /* Rest finished -> Advance to next note */
                g_u2t_play_current_step++;
                if (g_u2t_play_current_step >= g_u2t_seq_count)
                {
                    synth_seq_stop_playback();
                    bsp_uart_send_string("[PLAYBACK] Finished sequence playback!\r\n");
                }
                else
                {
                    g_b_play_in_note_phase = true;
                    g_u4t_play_step_start_ms = u4t_current_time;
                }
            }
            else
            {
                /* Still in rest duration */
            }
        }
    }
}

static void synth_update_live_sound(int8_t s1t_active_note, uint8_t u1t_volume_pct)
{
    if (s1t_active_note >= 0)
    {
        synth_play_note((uint8_t)g_current_octave, s1t_active_note, u1t_volume_pct);
        if (s1t_active_note != g_s1t_last_played)
        {
            bsp_uart_send_string("[KEY] Playing: ");
            bsp_uart_send_string(NOTE_NAMES[s1t_active_note]);
            bsp_uart_send_string(" | Vol: ");
            synth_uart_send_dec((uint32_t)u1t_volume_pct);
            bsp_uart_send_string("%\r\n");
            g_s1t_last_played = s1t_active_note;
        }
        else
        {
            /* Note already logged */
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
            /* In recording mode, LED is handled by blinking logic above */
        }

        g_s1t_last_played = -1;
        bsp_delay_us(1000U);
    }
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
            /* Hardware EXTI10 flag cleared without altering pitch/octave */
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
        synth_handle_uart();
        s1t_active_note = synth_select_active_note(&keys);
        u1t_volume_pct = bsp_adc_get_volume_percent();
        synth_report_volume(u4t_current_time, u1t_volume_pct);
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
