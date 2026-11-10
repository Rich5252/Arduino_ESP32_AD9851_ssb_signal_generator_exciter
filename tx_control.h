#pragma once

/**
 * tx_control.h  (2026-11-06, mode / mic PTT / sequencing 2026-11-10)
 *
 * TX switching, mode, mic PTT and output level - the TXlink TX control port, step by step.
 *
 * ---- Modes {Mn} ----
 * Nano digital modes (RTTY 1, FT2 2, FT4 4, FT8 8, JTTY 16) are accepted and remembered; this firmware cannot
 * generate them, and in those modes {U} keys TX directly (PTT ignored) - the pre-2026-11-10 behaviour.
 * New voice modes (powers of two, continuing the Nano's pattern):
 *   {M32}  USB  - mic input, upper sideband
 *   {M64}  LSB  - mic input, lower sideband (NOTE: all the DSP/predistort/delay tuning was done on USB)
 *   {M128} Tune - two-tone test signal, sideband as last set by USB/LSB
 * In voice modes TX is active only while {U} is set AND the mic PTT (MIC_PTT_IN_GPIO, active low) is pressed.
 * Boot mode: USB. Boot sideband/source are NOT forced - they change only when an {M} arrives, so the single-key
 * bench commands ('t', 'm', ...) still work as before.
 *
 * ---- Sequencing (runs from loop(), about every 10 ms) ----
 *   want TX:   RX -> KEYING: TXout on (if LocalPAactive), RF still gated off
 *              after TX_SEQ_RELAY_ON_MS -> TX: RF gate opens (envelope ramps up through the level slew)
 *   drop TX:   TX -> UNKEYING: RF gate closes (ramps down), TXout still on
 *              after TX_SEQ_RELAY_OFF_MS -> RX: TXout off
 * So the T/R relay never switches with RF on. Timing has loop()'s 10 ms granularity (15 ms -> 20 ms in practice).
 * Fine for mic modes; CW will need something different (later).
 *
 * ---- TXout ----
 * Nano name and job (antenna relay + local 2 W PA power): TXOUT_GPIO (GPIO40), ACTIVE HIGH.
 * TXout = (state != RX) AND LocalPAactive. Driven low first thing in setup().
 *
 * ---- {Lxxx} output level (TXlink semantics) ----
 * |xxx| = % of envelope VOLTAGE, clamped 0..100, applied just before the predistort LUT so every upstream stage
 * (mic gain, compressor, EQ, ALC, soft-limit) keeps working relative to 100 %. L < 0 = diagnostic mode:
 * LocalPAactive = false, TXout held off, DDS still driven at |L| % while TX is active. Boot: L0.
 * Output scale = |L|/100 x RF gate, slewed in dsp_task (5 ms full scale). Floor-aware:
 *   out = f + (env - f) * s,  f = floor * min(1, s / TX_LEVEL_FLOOR_FADE)
 * With the gate closed s = 0, so the output is zero (no RF) whatever the floor.
 * The chirp ('w') measurement path is neither scaled nor gated.
 */

#include <stdbool.h>
#include <stdint.h>

#define TX_MODE_RTTY     1
#define TX_MODE_FT2      2
#define TX_MODE_FT4      4
#define TX_MODE_FT8      8
#define TX_MODE_JTTY     16
#define TX_MODE_USB      32
#define TX_MODE_LSB      64
#define TX_MODE_TUNE     128
#define TX_MODE_BOOT     TX_MODE_USB

typedef enum { TX_SEQ_RX = 0, TX_SEQ_KEYING, TX_SEQ_TX, TX_SEQ_UNKEYING } tx_seq_state_t;

void tx_control_init(void);          // TXout low, L0, mode USB, PTT input; call first in setup()
void tx_control_service(void);       // loop(): PTT debounce + sequencer

// {U}/{D}
void tx_request_set(bool on);
bool tx_request_get(void);
bool tx_out_get(void);               // actual pin state

// {M}
void tx_mode_set(int mode);          // any value accepted; voice modes also set sideband / audio source
int  tx_mode_get(void);
bool tx_mode_is_voice(int mode);
const char *tx_mode_name(int mode);

// mic PTT (debounced)
bool tx_ptt_pressed(void);
tx_seq_state_t tx_seq_state(void);
const char *tx_seq_state_name(void);

// {L}
void tx_level_set(int level);        // signed, as received; magnitude clamped to 0..100
int  tx_level_get(void);
bool tx_local_pa_active(void);

// DSP side (dsp_task, once per full tick)
float tx_level_apply(float envelope);
float tx_level_current_scale(void);
