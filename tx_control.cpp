/**
 * tx_control.cpp - see tx_control.h.  (2026-11-10, uncompiled)
 */

#include "tx_control.h"
#include "config.h"
#include "envelope_floor.h"
#include "dsp_state.h"
#include "adc_capture.h"
#include "module_link.h"
#include "diagnostics.h"
#include "digi_gen.h"
#if AD9851_ATTACHED
#include "carrier_output.h"
#endif
#include <Arduino.h>
#include <stdlib.h>
#include <math.h>
#include "soc/gpio_reg.h"
#include "soc/gpio_sig_map.h"
#include "driver/gpio.h"
#include "esp_rom_gpio.h"
#include "esp_system.h"

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
static volatile float s_level_target = 0.0f;  // |effective level| / 100 (L, or P while the external PA is on)
static volatile int   s_l_mag = 0;              // |L|
static volatile bool  s_extpa_on = false;       // {Q1}
static volatile int   s_extpa_level = 0;        // {P} 0..100
static volatile bool  s_extpa_out = false;      // ExtPAout pin state
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
// 2026-11-11 digital modes (digi_gen.h): generator output of the last tick, TX-hold timer, carrier offset for dsp_task
static digi_out_t     s_digi_out = { 0.0f, false, false };
static uint32_t       s_digi_key_last = 0;
static bool           s_digi_key_seen = false;
static float          s_carrier_freq_hz = 0.0f;
static volatile bool  s_shutdown = false;  // set by the reboot shutdown handler: TX forced off for good

// ---- trip (slow_adc task -> loop message) ----
static volatile uint32_t s_trip_count = 0;
static const char *volatile s_trip_reason = "";
static volatile float s_trip_swr = 0, s_trip_fwd = 0, s_trip_rev = 0, s_trip_rev_mv = 0, s_trip_temp = 0;

#ifndef TEMP_TRIP_DEFAULT_C
#define TEMP_TRIP_DEFAULT_C 54
#endif
static volatile int s_temp_max = TEMP_TRIP_DEFAULT_C;

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

static void write_extpaout(bool on)
{
    if (on == s_extpa_out) return;
    gpio_set_level((gpio_num_t)EXTPAOUT_GPIO, on ? 1u : 0u);    // active high: 1 = external PA PTT on
    s_extpa_out = on;
}

static void update_level_target(void)
{
    // Nano setDDSlevelPWM(): level = ExtPAon ? ExtPAlevel : TXlevel (magnitude); LocalPAactive stays from L's sign
    int mag = s_extpa_on ? s_extpa_level : s_l_mag;
    s_level_target = (float)mag / 100.0f;
}

static void write_txout(bool on)
{
    if (on == s_tx_on) return;
    // IDF call, not digitalWrite(): the pin is set up with plain IDF GPIO calls in force_safe_outputs(), and the
    // Arduino core's digitalWrite() refuses pins its peripheral manager has not seen pinMode() on.
    gpio_set_level((gpio_num_t)TXOUT_GPIO, on ? 1u : 0u);       // active high: 1 = TX on
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

// Drive every TX-related output to its safe level with plain IDF GPIO calls (no Arduino peripheral manager), taking
// a pin back from LEDC / any peripheral first. Used at the top of setup() and in the reboot shutdown handler.
static void drive_low_as_gpio(int pin, int level)
{
    if (pin < 0) return;
    esp_rom_gpio_connect_out_signal((uint32_t)pin, SIG_GPIO_OUT_IDX, false, false);   // detach LEDC etc.
    gpio_set_level((gpio_num_t)pin, (uint32_t)level);
    gpio_set_direction((gpio_num_t)pin, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)pin, (uint32_t)level);
}

static void force_safe_outputs(void)
{
    drive_low_as_gpio(TXOUT_GPIO, 0);                           // relay to RX, local PA power off
    drive_low_as_gpio(EXTPAOUT_GPIO, 0);                        // external PA PTT off
    drive_low_as_gpio(FANPIN_GPIO, 0);                          // fan off until fan_control_init() takes the pin
#if AD9851_ATTACHED
    drive_low_as_gpio(AD9851_PIN_RESET, AD9851_RESET_ASSERT_LEVEL);   // DDS held in reset: 0 Hz, no RF
#endif
    drive_low_as_gpio(RSET_MOD_LEDC_GPIO, 0);                   // envelope PWM node to 0 (BS170 gate off)
    s_tx_on = false;
    s_extpa_out = false;
}

// esp_restart() (the 'b' command, OTA, ...) calls this before resetting. NOT called on a panic / watchdog reset or
// when the USB hardware resets the chip for flashing - the hardware pull resistors cover those.
static void tx_shutdown_handler(void)
{
    s_shutdown = true;
    s_requested = false;
    force_safe_outputs();
}

void tx_control_init(void)
{
    // 2026-11-10: very first thing in setup(): TXout off, external PA off, DDS held in reset, envelope node low - so
    // nothing left over from before the reset (the AD9851 keeps its last frequency on its own clock) can transmit
    // while setup() runs. carrier_output_init() later releases the DDS reset as part of its normal init.
    force_safe_outputs();
    esp_register_shutdown_handler(tx_shutdown_handler);
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
    s_state = TX_SEQ_RX;
    // 2026-11-11 digital modes: generator tables for the Nano's default FT8, then the boot mode (not digital)
    digi_cfg_t dc = { (uint32_t)SAMPLE_RATE_HZ, DIGI_GAUSSIAN_SHAPE, DIGI_BT_FT8, DIGI_BT_FT4, DIGI_BT_FT2, DIGI_BT_JTTY,
                      DIGI_BT_RTTY };
    digi_init(&dc);
    digi_set_mode(s_mode);
}

bool IRAM_ATTR tx_control_tick(void)
{
    if (s_shutdown) { s_gate = false; return false; }          // rebooting: outputs already forced safe, leave them
    s_tick++;
    const uint32_t now = s_tick;
    const int mode = s_mode;
    const bool cw = (mode == TX_MODE_CW);
    const bool digi = digi_mode_is_digital(mode);
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
    } else if (digi) {
        // 2026-11-11 Nano: the digital "key" is DigiTxOn ({X1}) plus the message in progress (digi_gen), held for
        // {H} ms after it drops (Nano TXholdTime, as CW). RF follows the generator (first..last symbol).
        uint32_t hold = MS_TO_TICKS(s_hold_ms);
        if (hold < rf_off_wait) hold = rf_off_wait;           // RF must be down before TXout drops
        if (s_digi_out.key) { s_digi_key_last = now; s_digi_key_seen = true; }
        bool holding = s_digi_out.key || (s_digi_key_seen && now - s_digi_key_last < hold);
        want = s_requested && holding;
        rf_want = false;                                       // set after the generator has run, below
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
    write_extpaout(s_state != TX_SEQ_RX && s_local_pa && s_extpa_on);   // Nano: ExtPAout with TXout when {Q1}
    if (digi) {
        // generator: starts once the relay is made (TX), {V} start delay only after a fresh RX -> TX
        digi_tick(s_requested, s_state == TX_SEQ_TX, s_msg_tx_on, &s_digi_out);
        rf_want = s_requested && s_digi_out.rf_on;
        s_carrier_freq_hz = s_digi_out.freq_hz;
    } else {
        s_carrier_freq_hz = 0.0f;
    }
    s_gate = (s_state == TX_SEQ_TX) && rf_want;
    s_cw_carrier = cw || digi;
    return cw || digi;
}

float IRAM_ATTR tx_carrier_freq_dev_hz(void) { return s_carrier_freq_hz; }

void tx_control_service(void)
{
    static uint32_t on_seen = 0, off_seen = 0, ptt_seen = 0, ovf_seen = 0, trip_seen = 0;
    if (trip_seen != s_trip_count) {
        trip_seen = s_trip_count;
        Serial.print("{D}{N}\r\n");                                                  // TX off in the UI (Nano)
        Serial.printf("-> TX TRIP: %s (SWR %.1f, fwd %.1f W, rev %.2f W, REV pin %.0f mV, PA %.1f degC, limit %d) - "
                      "send the U command to re-enable\r\n", (const char *)s_trip_reason, s_trip_swr, s_trip_fwd, s_trip_rev,
                      s_trip_rev_mv, s_trip_temp, s_temp_max);
    }
    // {M} before {N} if both happened since the last loop (a very short TX)
    while (on_seen != s_msg_tx_on || off_seen != s_msg_tx_off) {
        if (on_seen != s_msg_tx_on) {
            on_seen++;
            Serial.print("{M}\r\n");                                                // mute RX (Nano)
#if AD9851_ATTACHED
            module_link_send_freq_exact(carrier_output_get_freq_hz());               // Nano: exact {f} at key-down
#endif
        }
        if (off_seen != s_msg_tx_off) { off_seen++; Serial.print("{N}\r\n"); }    // unmute RX
    }
    if (ptt_seen != s_msg_ptt) {
        ptt_seen = s_msg_ptt;
        if (!diagnostics_get_muted()) Serial.printf("-> mic PTT %s\r\n", s_ptt ? "pressed" : "released");   // diagnostic
    }
    if (ovf_seen != s_msg_kf_ovf) { ovf_seen = s_msg_kf_ovf; Serial.printf("-> CW key edge queue overflow\r\n"); }
    // 2026-11-11 digital-mode start/end lines (diagnostic, only when unmuted)
    static uint32_t dstart_seen = 0, dend_seen = 0, dabort_seen = 0;
    digi_status_t ds;
    digi_get_status(&ds);
    if (ds.starts != dstart_seen || ds.ends != dend_seen || ds.aborts != dabort_seen) {
        bool started = ds.starts != dstart_seen, ended = ds.ends != dend_seen, aborted = ds.aborts != dabort_seen;
        dstart_seen = ds.starts; dend_seen = ds.ends; dabort_seen = ds.aborts;
        if (!diagnostics_get_muted()) {
            if (started && !ended && !aborted)
                Serial.printf("-> digital %s start: first symbol %d, offset %d Hz, %ld ms left, %s\r\n", digi_mode_name(ds.mode),
                              ds.first_symbol, ds.df_hz, ds.millis_to_end, ds.tune ? "tune" : "message");
            if (ended || aborted)
                Serial.printf("-> digital %s %s: %u symbols/codes, %.3f s, restarts %u\r\n", digi_mode_name(ds.mode),
                              aborted ? "aborted" : "end", (unsigned)ds.symbols_sent,
                              (double)ds.run_ticks / SAMPLE_RATE_HZ, (unsigned)ds.restarts);
        }
    }
}

void tx_request_set(bool on) { s_requested = on; }

void tx_trip(const char *reason, float swr, float fwd_w, float rev_w, float rev_pin_mv, float temp_c)
{
    s_requested = false;                       // the tick ramps RF off within {E} ms and drops TXout after it
    digi_abort();                              // Nano: DigiTxOn = false, the digital message stops
    s_trip_reason = reason; s_trip_swr = swr; s_trip_fwd = fwd_w; s_trip_rev = rev_w; s_trip_rev_mv = rev_pin_mv;
    s_trip_temp = temp_c;
    s_trip_count = s_trip_count + 1;
}

void tx_temp_max_set(int deg_c) { if (deg_c < 0) deg_c = 0; if (deg_c > 150) deg_c = 150; s_temp_max = deg_c; }
int  tx_temp_max_get(void)      { return s_temp_max; }
uint32_t tx_trip_count(void) { return s_trip_count; }
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
    if (mode != s_mode) digi_set_mode(mode);                   // Nano: setDigDefaults() on a mode change
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
    s_l_mag = mag;
    update_level_target();
}

int  tx_level_get(void)       { return s_level; }
bool tx_local_pa_active(void) { return s_local_pa; }
bool tx_report_idle(void)     { return !s_requested && s_level == 0; }

void tx_extpa_set_on(bool on)   { s_extpa_on = on; update_level_target(); }
bool tx_extpa_get_on(void)      { return s_extpa_on; }
void tx_extpa_set_level(int pct){ if (pct < 0) pct = -pct; if (pct > 100) pct = 100; s_extpa_level = pct; update_level_target(); }
int  tx_extpa_get_level(void)   { return s_extpa_level; }
bool tx_extpa_out_get(void)     { return s_extpa_out; }
uint32_t tx_start_count(void)   { return s_msg_tx_on; }

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
