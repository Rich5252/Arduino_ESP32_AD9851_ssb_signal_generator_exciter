/**
 * tx_control.cpp - see tx_control.h.  (2026-11-06; mode / mic PTT / sequencing 2026-11-10, uncompiled)
 */

#include "tx_control.h"
#include "config.h"
#include "envelope_floor.h"
#include "dsp_state.h"
#include "adc_capture.h"
#include <Arduino.h>
#include <stdlib.h>

#ifndef TX_LEVEL_SLEW_MS
#define TX_LEVEL_SLEW_MS        5.0f     // ms for a full-scale (0 <-> 100 %) scale change
#endif
#ifndef TX_LEVEL_FLOOR_FADE
#define TX_LEVEL_FLOOR_FADE     0.10f    // below this scale the envelope floor fades out (0 = no output)
#endif
#ifndef TX_SEQ_RELAY_ON_MS
#define TX_SEQ_RELAY_ON_MS      15u      // TXout on -> RF allowed
#endif
#ifndef TX_SEQ_RELAY_OFF_MS
#define TX_SEQ_RELAY_OFF_MS     15u      // RF gate closed -> TXout off (must exceed the level ramp-down, 5 ms)
#endif
#ifndef TX_PTT_DEBOUNCE_MS
#define TX_PTT_DEBOUNCE_MS      20u      // PTT must read the same for this long before it counts
#endif

static volatile bool  s_requested = false;
static volatile bool  s_tx_on = false;
static volatile int   s_level = 0;
static volatile bool  s_local_pa = true;
static volatile int   s_mode = TX_MODE_BOOT;
static volatile bool  s_ptt = false;              // debounced, true = pressed
static volatile tx_seq_state_t s_state = TX_SEQ_RX;
static volatile bool  s_rf_gate = false;          // read by dsp_task
static volatile float s_level_target = 0.0f;      // |L|/100, written on Core 1, read by dsp_task (Core 0)
static float          s_scale = 0.0f;             // owned by dsp_task
static uint32_t       s_state_ms = 0;             // millis() at the last state change
static bool           s_ptt_raw_last = false;
static uint32_t       s_ptt_raw_since = 0;

static void apply_txout(void)
{
    bool on = (s_state != TX_SEQ_RX) && s_local_pa;
    digitalWrite(TXOUT_GPIO, on ? HIGH : LOW);    // active high: HIGH = TX on
    s_tx_on = on;
}

static bool want_tx(void)
{
    if (!s_requested) return false;
    if (tx_mode_is_voice(s_mode)) return s_ptt;
    return true;                                  // Nano digital modes: {U} keys directly
}

static void set_state(tx_seq_state_t st)
{
    s_state = st;
    s_state_ms = millis();
    s_rf_gate = (st == TX_SEQ_TX);
    apply_txout();
}

void tx_control_init(void)
{
    digitalWrite(TXOUT_GPIO, LOW);                // set the output latch low first, so the pin never glitches high
    pinMode(TXOUT_GPIO, OUTPUT);
#if MIC_PTT_IN_GPIO >= 0
    pinMode(MIC_PTT_IN_GPIO, INPUT_PULLUP);       // active low; external 4.7-10k pull-up + RC recommended as well
#endif
    s_requested = false;
    s_local_pa = true;
    s_level = 0;
    s_level_target = 0.0f;
    s_scale = 0.0f;
    s_mode = TX_MODE_BOOT;
    s_ptt = false;
    s_ptt_raw_last = false;
    s_ptt_raw_since = millis();
    set_state(TX_SEQ_RX);
}

void tx_control_service(void)
{
    uint32_t now = millis();
#if MIC_PTT_IN_GPIO >= 0
    bool raw = (digitalRead(MIC_PTT_IN_GPIO) == LOW);
    if (raw != s_ptt_raw_last) { s_ptt_raw_last = raw; s_ptt_raw_since = now; }
    else if (raw != s_ptt && (uint32_t)(now - s_ptt_raw_since) >= TX_PTT_DEBOUNCE_MS) {
        s_ptt = raw;
        Serial.printf("-> mic PTT %s\r\n", raw ? "pressed" : "released");
    }
#endif
    bool want = want_tx();
    uint32_t in_state = now - s_state_ms;
    switch (s_state) {
    case TX_SEQ_RX:
        if (want) set_state(TX_SEQ_KEYING);
        break;
    case TX_SEQ_KEYING:
        if (!want) set_state(TX_SEQ_RX);                      // RF never opened: drop straight back
        else if (in_state >= TX_SEQ_RELAY_ON_MS) set_state(TX_SEQ_TX);
        break;
    case TX_SEQ_TX:
        if (!want) set_state(TX_SEQ_UNKEYING);
        break;
    case TX_SEQ_UNKEYING:
        if (want) set_state(TX_SEQ_TX);                       // relay still made: reopen RF at once
        else if (in_state >= TX_SEQ_RELAY_OFF_MS) set_state(TX_SEQ_RX);
        break;
    }
}

void tx_request_set(bool on) { s_requested = on; tx_control_service(); }
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
    default:           return "unknown";
    }
}

void tx_mode_set(int mode)
{
    s_mode = mode;
    if (mode == TX_MODE_USB || mode == TX_MODE_LSB) {
        dsp_state_set_sideband(mode == TX_MODE_LSB ? SSB_SIDEBAND_LSB : SSB_SIDEBAND_USB);
        if (dsp_state_get_audio_source() != AUDIO_SRC_MIC) {
            dsp_state_set_audio_source(AUDIO_SRC_MIC);
            adc_capture_resample_prime_calibration();         // same re-baseline as the 'm' key
        }
    } else if (mode == TX_MODE_TUNE) {
        dsp_state_set_audio_source(AUDIO_SRC_TWOTONE);
    }
    tx_control_service();                                     // a mode change can start or stop TX
}

int  tx_mode_get(void)   { return s_mode; }
bool tx_ptt_pressed(void) { return s_ptt; }
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
    bool local = (level >= 0);                    // Nano: TXlevel < 0 -> LocalPAactive = false
    int mag = abs(level);
    if (mag > 100) mag = 100;
    s_level = local ? mag : -mag;
    s_local_pa = local;
    s_level_target = (float)mag / 100.0f;
    apply_txout();                                // a negative L drops TXout at once, a positive one restores it
}

int  tx_level_get(void)       { return s_level; }
bool tx_local_pa_active(void) { return s_local_pa; }

float IRAM_ATTR tx_level_apply(float envelope)
{
    const float step = 1.0f / (TX_LEVEL_SLEW_MS * 0.001f * (float)SAMPLE_RATE_HZ);
    float t = s_rf_gate ? s_level_target : 0.0f;
    if (s_scale < t)      { s_scale += step; if (s_scale > t) s_scale = t; }
    else if (s_scale > t) { s_scale -= step; if (s_scale < t) s_scale = t; }
    float s = s_scale;
    if (s >= 1.0f) return envelope;               // L100 with the gate open: bit-identical to before
    if (s <= 0.0f) return 0.0f;                   // gate closed / L0: no RF, floor included
    float fade = s / TX_LEVEL_FLOOR_FADE;
    if (fade > 1.0f) fade = 1.0f;
    float f = envelope_floor_get() * fade;
    return f + (envelope - f) * s;
}

float tx_level_current_scale(void) { return s_scale; }
