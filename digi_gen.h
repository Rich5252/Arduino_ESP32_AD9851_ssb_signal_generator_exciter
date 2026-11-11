#pragma once

/**
 * digi_gen.h  (2026-11-11)
 *
 * TXlink Nano digital modes (TXlink4_Nano_Latest d3db0ad, 2026-10-21) for the ESP32 exciter: FT8, FT4, FT2, RTTY,
 * JTTY. The PROTOCOL and the SEQUENCING are the Nano's, kept as close to its code as the platform allows:
 *   {V<ms>} TX start delay ({X1} -> first symbol; FT4 half, FT2/JTTY none)   Nano FT8txDelay, default 250
 *   {W<ms>} ms left in the current T/R period ({W0} = stop at the end of the current symbol)
 *   {X1}/{X0} digital TX on/off ({X0} also empties the RTTY queue, aborts JTTY)
 *   {Y<Hz>} TX audio offset (FTxTxDF): tones start at carrier + Y
 *   {Z<...>} FT8/FT4/FT2 symbol list ("0".."7", 79 or 105 chars; all zeros = WSJT-X Tune) or, in RTTY, a text line
 *   {J<hex>} / {JT} JTTY data symbols (sync inserted here) / JTTY tune
 *   AdjustStart() (late-start truncation), restart on a changed message or offset during TX, the WSJT-X Tune
 *   detection, the RTTY Baudot/ITA2 table, shifts, preamble and trailer, the JTTY frame layout and chaining.
 *
 * What is NOT the Nano's: the tone-to-tone frequency change. The Nano stepped the DDS from a busy-wait loop with a
 * 32-point Gaussian table over ~10-13 % of a symbol. Here the frequency is computed once per DSP tick (20 kHz, from
 * the sample timer's crystal, so symbol timing has no jitter) as a sum of erf steps centred on the symbol boundaries:
 *   shape 1 (default): the WSJT-X GFSK pulse - FT8 BT 2, FT4 BT 1 (WSJT-X), FT2 BT 1, JTTY BT 2 (assumed; config.h)
 *   shape 0: the Nano curve exactly as it ran (its table, its integer interpolation, its 64 us steps, its timing).
 * Simulation (host_tests/digi_gaussian_sim) says shape 1 cuts the spread by about 30 dB (FT8) / 48 dB (FT4) at
 * +50..100 Hz; step time and 1 ms timing jitter made no difference. RTTY stays hard-keyed FSK, as on the Nano.
 *
 * Pure C++: no Arduino/ESP code, host-tested by host_tests/test_digi_gen.cpp. Threads: the digi_cmd_*(),
 * digi_set_mode() and digi_abort() calls are made from loop() / other tasks (Core 1); digi_tick() runs in dsp_task
 * (Core 0) from tx_control_tick(). Shared data is published with a release fence or read once per symbol.
 */

#include <stdint.h>
#include <stdbool.h>

#define DIGI_MODE_RTTY   1      // same numbers as the Nano's {M} / tx_control.h
#define DIGI_MODE_FT2    2
#define DIGI_MODE_FT4    4
#define DIGI_MODE_FT8    8
#define DIGI_MODE_JTTY   16

typedef struct {
    uint32_t tick_hz;           // DSP tick rate (SAMPLE_RATE_HZ, 20000)
    int      shape;             // 1 = WSJT-X GFSK pulse, 0 = Nano curve
    float    bt_ft8, bt_ft4, bt_ft2, bt_jtty;
} digi_cfg_t;

void digi_init(const digi_cfg_t *cfg);   // once, before anything else (builds the boot mode's tables)
bool digi_mode_is_digital(int mode);
void digi_set_mode(int mode);            // Nano setDigDefaults() on an {M} change; a non-digital mode just aborts

// ---- brace commands, Nano semantics (loop context) ----
void digi_cmd_V(long ms);
void digi_cmd_W(long ms);
void digi_cmd_X(long on);
void digi_cmd_Y(long hz);
void digi_cmd_Z(const char *msg);       // returns nothing, like the Nano; bad lengths fall back to its error message
void digi_cmd_J(const char *payload);   // everything after 'J' up to '}' (hex digits, or "T")
void digi_abort(void);                  // SWR/temp trip or {D}: DigiTxOn = false, the tick stops at once

// ---- per tick (dsp_task) ----
typedef struct {
    float freq_hz;              // offset from the carrier, Hz (held after the end so the RF ramp-down sees no jump)
    bool  rf_on;                // RF wanted (first symbol .. last symbol)
    bool  key;                  // TX wanted: DigiTxOn or a transmission in progress (the Nano's "DigiTxOn" key)
} digi_out_t;

// requested: TX enabled ({U}, no trip); tx_ready: sequencer in TX (relay made); keydown_count: increments on every
// RX -> TX start (tx_start_count()) - a change means a fresh key-down, which gets the {V} start delay (Nano: only
// when not already keyed).
void digi_tick(bool requested, bool tx_ready, uint32_t keydown_count, digi_out_t *out);

// ---- status (any context; values may be a tick old) ----
typedef struct {
    int      mode;
    bool     digi_tx_on;        // Nano DigiTxOn
    int      state;             // 0 idle, 1 waiting for TX, 2 start delay, 3 running
    uint32_t starts, ends, restarts, aborts;
    int      first_symbol;      // first symbol index of the last start (AdjustStart)
    uint32_t symbols_sent;      // symbols (RTTY: Baudot codes) since the last start
    uint32_t run_ticks;         // ticks since the last start
    int      tx_delay_ms, df_hz;
    long     millis_to_end;
    int      symbol_count;      // FT modes; JTTY: channel symbols held; RTTY: messages queued
    bool     tune;
    int      half_window_ticks; // transition table half width
} digi_status_t;
void digi_get_status(digi_status_t *st);
const char *digi_mode_name(int mode);
