/**
 * tx_control.cpp - see tx_control.h.  (2026-11-10, uncompiled)
 */

#include "tx_control.h"
#include "config.h"
#include "envelope_floor.h"
#include "dsp_state.h"
#include "adc_capture.h"
#include <Arduino.h>
#include <stdlib.h>
#include <math.h>
#include "soc/gpio_reg.h"

#ifndef TX_LEVEL_SLEW_MS
#define TX_LEVEL_SLEW_MS          5.0f   // ms for a full-scale level change ({L})
#endif
#ifndef TX_LEVEL_FLOOR_FADE
#define TX_LEVEL_FLOOR_FADE       0.10f  // below this scale the envelope floor fades out
#endif
#ifndef TX_SEQ_RELAY_MS
#define TX_SEQ_RELAY_MS           5u     // TXout on -> RF allowed (Nano: 5 ms); also the CW keying delay
#endif
#ifndef TX_SEQ_RF_OFF_MARGIN_MS
#define TX_SEQ_RF_OFF_MARGIN_MS   2u     // after the RF edge has finished, before TXout drops
#endif
#ifndef TX_PTT_DEBOUNCE_MS
#define TX_PTT_DEBOUNCE_MS        20u
#endif
#ifndef TX_KEY_DEBOUNCE_MS
#define TX_KEY_DEBOUNCE_MS        1u     // CW key: must be stable 1 ms (adds the same 1 ms to both edges)
#endif
#ifndef TX_HOLD_DEFAULT_MS
#define TX_HOLD_DEFAULT_MS        200
#endif
#ifndef TX_EDGE_DEFAULT_MS
#define TX_EDGE_DEFAULT_MS        5.0f
#endif

#define MS_TO_TICKS(ms)  ((uint32_t)((ms) * (SAMPLE_RATE_HZ / 1000.0f) + 0.5f))

// ---- command side -> tick ----
static volatile bool  s_requested = false;
static volatile int   s_level = 0;
static volatile bool  s_local_pa = true;
static volatile int   s_mode = TX_MODE_BOOT;
static volatile float s_level_target = 0.0f;
static volatile int   s_hold_ms = TX_HOLD_DEFAULT_MS;
static volatile float s_edge_ms = TX_EDGE_DEFAULT_MS;

// ---- tick-owned state (read elsewhere for status only) ----
static volatile tx_seq_state_t s_state = TX_SEQ_RX;
static volatile bool  s_tx_on = false;
static volatile bool  s_ptt = false;
static volatile bool  s_key = false;
static uint32_t       s_tick = 0;
static uint32_t       s_state_tick = 0;
static bool           s_ptt_raw = false;  static uint32_t s_ptt_raw_tick = 0;
static bool           s_key_raw = false;  static uint32_t s_key_raw_tick = 0;
// CW delay line: key edges (tick + new state) waiting TX_SEQ_RELAY_MS before they reach the RF gate
#define KEY_FIFO_N 32
static uint32_t       s_kf_tick[KEY_FIFO_N];
static bool           s_kf_state[KEY_FIFO_N];
static uint8_t        s_kf_head = 0, s_kf_tail = 0;
static bool           s_key_rf = false;   // delayed key
static uint32_t       s_keyup_tick = 0;
static bool           s_gate = false;     // RF gate target
static float          s_edge_pos = 0.0f;  // 0..1 along the edge curve
static float          s_level_scale = 0.0f;
static float          s_scale = 0.0f;
static bool           s_cw_carrier = false;

// ---- tick -> loop messages ----
static volatile uint32_t s_msg_tx_on = 0, s_msg_tx_off = 0, s_msg_ptt = 0, s_msg_key = 0, s_msg_kf_ovf = 0;

// Gaussian edge: shape(p) = (erf(a(2p-1))/erf(a) + 1)/2, a = 2 (+/-2 sigma), 257-point table
#define EDGE_LUT_N 256
static float s_edge_lut[EDGE_LUT_N + 1];

static inline bool gpio_in(int pin)
{
    if (pin < 0) return true;                                   // not fitted: reads "released" (high)
    if (pin < 32) return (REG_READ(GPIO_IN_REG) >> pin) & 1u;
    return (REG_READ(GPIO_IN1_REG) >> (pin - 32)) & 1u;
}

static void write_txout(bool on)
{
    if (on == s_tx_on) return;
    digitalWrite(TXOUT_GPIO, on ? HIGH : LOW);                  // active high: HIGH = TX on
    s_tx_on = on;
}

static void set_state(tx_seq_state_t st)
{
    tx_seq_state_t old = s_state;
    s_state = st;
    s_state_tick = s_tick;
    if (old == TX_SEQ_RX && st != TX_SEQ_RX) s_msg_tx_on = s_msg_tx_on + 1;
    if (old != TX_SEQ_RX && st == TX_SEQ_RX) s_msg_tx_off = s_msg_tx_off + 1;
}

void tx_control_init(void)
{
    digitalWrite(TXOUT_GPIO, LOW);                              // latch low before it becomes an output
    pinMode(TXOUT_GPIO, OUTPUT);
    digitalWrite(TXOUT_GPIO, LOW);
#if MIC_PTT_IN_GPIO >= 0
    pinMode(MIC_PTT_IN_GPIO, INPUT_PULLUP);
#endif
#if KEYIN_GPIO >= 0
    pinMode(KEYIN_GPIO, INPUT_PULLUP);
#endif
    const float a = 2.0f, ea = erff(a);
    for (int i = 0; i <= EDGE_LUT_N; i++) {
        float p = (float)i / EDGE_LUT_N;
        s_edge_lut[i] = 0.5f * (erff(a * (2.0f * p - 1.0f)) / ea + 1.0f);
    }
    s_edge_lut[0] = 0.0f;
    s_edge_lut[EDGE_LUT_N] = 1.0f;
    s_tx_on = false;
    s_state = TX_SEQ_RX;
}

bool IRAM_ATTR tx_control_tick(void)
{
    s_tick++;
    const uint32_t now = s_tick;
    const int mode = s_mode;
    const bool cw = (mode == TX_MODE_CW);
    const uint32_t relay = MS_TO_TICKS(TX_SEQ_RELAY_MS);
    const uint32_t edge_ticks = MS_TO_TICKS(s_edge_ms);
    const uint32_t rf_off_wait = edge_ticks + MS_TO_TICKS(TX_SEQ_RF_OFF_MARGIN_MS);

    // ---- inputs (active low), debounced by "stable for N ticks" ----
    bool p = !gpio_in(MIC_PTT_IN_GPIO);
    if (p != s_ptt_raw) { s_ptt_raw = p; s_ptt_raw_tick = now; }
    else if (p != s_ptt && now - s_ptt_raw_tick >= MS_TO_TICKS(TX_PTT_DEBOUNCE_MS)) { s_ptt = p; s_msg_ptt = s_msg_ptt + 1; }

    bool k = !gpio_in(KEYIN_GPIO);
    if (k != s_key_raw) { s_key_raw = k; s_key_raw_tick = now; }
    else if (k != s_key && now - s_key_raw_tick >= MS_TO_TICKS(TX_KEY_DEBOUNCE_MS)) {
        s_key = k;
        if (!k) s_keyup_tick = now;
        uint8_t nh = (uint8_t)((s_kf_head + 1) % KEY_FIFO_N);
        if (nh != s_kf_tail) { s_kf_tick[s_kf_head] = now; s_kf_state[s_kf_head] = k; s_kf_head = nh; }
        else s_msg_kf_ovf = s_msg_kf_ovf + 1;                   // >32 edges inside 5 ms: cannot happen with debounce
    }
    while (s_kf_tail != s_kf_head && now - s_kf_tick[s_kf_tail] >= relay) {
        s_key_rf = s_kf_state[s_kf_tail];
        s_kf_tail = (uint8_t)((s_kf_tail + 1) % KEY_FIFO_N);
    }

    // ---- what is wanted ----
    bool want, rf_want;
    if (cw) {
        uint32_t hold = MS_TO_TICKS(s_hold_ms);
        if (hold < relay + rf_off_wait) hold = relay + rf_off_wait;   // RF must be down before TXout drops
        bool holding = s_key || s_key_rf || (s_keyup_tick != 0 && now - s_keyup_tick < hold);
        want = s_requested && holding;
        rf_want = s_requested && s_key_rf;
    } else {
        want = s_requested && (tx_mode_is_voice(mode) ? s_ptt : true);
        rf_want = want;
    }

    // ---- sequencer ----
    uint32_t in_state = now - s_state_tick;
    switch (s_state) {
    case TX_SEQ_RX:
        if (want) set_state(TX_SEQ_KEYING);
        break;
    case TX_SEQ_KEYING:
        if (!want) set_state(TX_SEQ_RX);
        else if (in_state >= relay) set_state(TX_SEQ_TX);
        break;
    case TX_SEQ_TX:
        if (!want) set_state(TX_SEQ_UNKEYING);
        break;
    case TX_SEQ_UNKEYING:
        if (want) set_state(TX_SEQ_TX);
        else if (in_state >= rf_off_wait && s_edge_pos <= 0.0f) set_state(TX_SEQ_RX);
        break;
    }
    write_txout(s_state != TX_SEQ_RX && s_local_pa);
    s_gate = (s_state == TX_SEQ_TX) && rf_want;
    s_cw_carrier = cw;
    return cw;
}

void tx_control_service(void)
{
    static uint32_t on_seen = 0, off_seen = 0, ptt_seen = 0, ovf_seen = 0;
    // {M} before {N} if both happened since the last loop (a very short TX)
    while (on_seen != s_msg_tx_on || off_seen != s_msg_tx_off) {
        if (on_seen != s_msg_tx_on) { on_seen++; Serial.print("{M}\r\n"); }       // mute RX (Nano)
        if (off_seen != s_msg_tx_off) { off_seen++; Serial.print("{N}\r\n"); }    // unmute RX
    }
    if (ptt_seen != s_msg_ptt) { ptt_seen = s_msg_ptt; Serial.printf("-> mic PTT %s\r\n", s_ptt ? "pressed" : "released"); }
    if (ovf_seen != s_msg_kf_ovf) { ovf_seen = s_msg_kf_ovf; Serial.printf("-> CW key edge queue overflow\r\n"); }
}

void tx_request_set(bool on) { s_requested = on; }
bool tx_request_get(void)    { return s_requested; }
bool tx_out_get(void)        { return s_tx_on; }

bool tx_mode_is_voice(int mode)
{
    return mode == TX_MODE_USB || mode == TX_MODE_LSB || mode == TX_MODE_TUNE;
}

const char *tx_mode_name(int mode)
{
    switch (mode) {
    case TX_MODE_RTTY: return "RTTY";
    case TX_MODE_FT2:  return "FT2";
    case TX_MODE_FT4:  return "FT4";
    case TX_MODE_FT8:  return "FT8";
    case TX_MODE_JTTY: return "JTTY";
    case TX_MODE_USB:  return "USB";
    case TX_MODE_LSB:  return "LSB";
    case TX_MODE_TUNE: return "Tune (two-tone)";
    case TX_MODE_CW:   return "CW";
    default:           return "unknown";
    }
}

void tx_mode_set(int mode)
{
    if (mode == TX_MODE_USB || mode == TX_MODE_LSB) {
        dsp_state_set_sideband(mode == TX_MODE_LSB ? SSB_SIDEBAND_LSB : SSB_SIDEBAND_USB);
        if (dsp_state_get_audio_source() != AUDIO_SRC_MIC) {
            dsp_state_set_audio_source(AUDIO_SRC_MIC);
            adc_capture_resample_prime_calibration();          // same re-baseline as the 'm' key
        }
    } else if (mode == TX_MODE_TUNE) {
        dsp_state_set_audio_source(AUDIO_SRC_TWOTONE);
    }
    s_mode = mode;
}

int  tx_mode_get(void) { return s_mode; }

void tx_hold_set_ms(int ms) { if (ms < 0) ms = 0; if (ms > 5000) ms = 5000; s_hold_ms = ms; }
int  tx_hold_get_ms(void)   { return s_hold_ms; }
void  tx_edge_set_ms(float ms) { if (!(ms >= 0.5f)) ms = 0.5f; if (ms > 20.0f) ms = 20.0f; s_edge_ms = ms; }
float tx_edge_get_ms(void)     { return s_edge_ms; }

bool tx_ptt_pressed(void) { return s_ptt; }
bool tx_key_down(void)    { return s_key; }
tx_seq_state_t tx_seq_state(void) { return s_state; }

const char *tx_seq_state_name(void)
{
    switch (s_state) {
    case TX_SEQ_RX:       return "RX";
    case TX_SEQ_KEYING:   return "keying";
    case TX_SEQ_TX:       return "TX";
    case TX_SEQ_UNKEYING: return "unkeying";
    }
    return "?";
}

void tx_level_set(int level)
{
    bool local = (level >= 0);                                  // Nano: TXlevel < 0 -> LocalPAactive = false
    int mag = abs(level);
    if (mag > 100) mag = 100;
    s_level = local ? mag : -mag;
    s_local_pa = local;                                         // the tick drops/restores TXout within 50 us
    s_level_target = (float)mag / 100.0f;
}

int  tx_level_get(void)       { return s_level; }
bool tx_local_pa_active(void) { return s_local_pa; }
bool tx_report_idle(void)     { return !s_requested && s_level == 0; }

float IRAM_ATTR tx_level_apply(float envelope)
{
    // level: linear slew
    const float lstep = 1.0f / (TX_LEVEL_SLEW_MS * 0.001f * (float)SAMPLE_RATE_HZ);
    float t = s_level_target;
    if (s_level_scale < t)      { s_level_scale += lstep; if (s_level_scale > t) s_level_scale = t; }
    else if (s_level_scale > t) { s_level_scale -= lstep; if (s_level_scale < t) s_level_scale = t; }
    // keying edge: walk the Gaussian curve
    const float estep = 1.0f / (s_edge_ms * 0.001f * (float)SAMPLE_RATE_HZ);
    if (s_gate) { if (s_edge_pos < 1.0f) { s_edge_pos += estep; if (s_edge_pos > 1.0f) s_edge_pos = 1.0f; } }
    else        { if (s_edge_pos > 0.0f) { s_edge_pos -= estep; if (s_edge_pos < 0.0f) s_edge_pos = 0.0f; } }
    float x = s_edge_pos * EDGE_LUT_N;
    int i = (int)x;
    float g = (i >= EDGE_LUT_N) ? 1.0f : s_edge_lut[i] + (s_edge_lut[i + 1] - s_edge_lut[i]) * (x - (float)i);

    float s = s_level_scale * g;
    s_scale = s;
    if (s >= 1.0f) return envelope;                             // L100, edge complete: bit-identical to before
    if (s <= 0.0f) return 0.0f;                                 // gate closed / L0: no RF, floor included
    float fade = s / TX_LEVEL_FLOOR_FADE;
    if (fade > 1.0f) fade = 1.0f;
    float f = envelope_floor_get() * fade;
    return f + (envelope - f) * s;
}

float tx_level_current_scale(void) { return s_scale; }
