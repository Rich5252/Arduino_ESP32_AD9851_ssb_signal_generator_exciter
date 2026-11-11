#pragma once

/**
 * tx_control.h  (2026-11-06; modes / mic PTT 2026-11-10; CW, sequencer in the DSP tick, edge shaping 2026-11-10)
 *
 * TX switching, mode, mic PTT, CW key, output level - the TXlink TX control port.
 *
 * ---- Safe states (2026-11-10) ----
 * tx_control_init() (first line of setup()) forces TXout and ExtPAout low, holds the AD9851 in reset and pulls the
 * envelope PWM pin low before anything else runs; a shutdown handler does the same on esp_restart() ('b' reboot).
 * Neither can act during a USB flash or a panic/watchdog reset - only external pull resistors cover those
 * (10k pull-downs on TXout 45 / ExtPAout 46, ~100k gate pull-down on the AD9851 RESET BS170).
 *
 * ---- Where it runs ----
 * The whole sequencer runs in dsp_task (Core 0) once per full tick (20 kHz, tx_control_tick()), so key/PTT timing is
 * sample-accurate and nothing extra runs periodically on Core 1. The command side ({U} {D} {M} {L} {H} {E}) only
 * writes volatile settings; loop() (tx_control_service()) only prints the messages the tick has flagged.
 * TXOUT is written only by the tick (on change).
 *
 * ---- Modes {Mn} ----
 *   Nano digital modes RTTY 1, FT2 2, FT4 4, FT8 8, JTTY 16 (2026-11-11, digi_gen.h): generated here as a carrier
 *       offset (envelope 100 %), keyed by {U} + {X1} (Nano DigiTxOn) while the message runs, held {H} ms after it.
 *   {M32} USB, {M64} LSB: mic input, sideband set; {M128} Tune: two-tone; keyed by {U} + mic PTT (GPIO7).
 *   {M256} CW: steady carrier at the carrier frequency, keyed by {U} + key (KEYIN_GPIO, GPIO1, active low).
 *   Boot: USB (sideband/source not forced until an {M} arrives).
 *
 * ---- Sequencing ----
 *   Voice / digital: want -> TXout on (if LocalPAactive) -> after TX_SEQ_RELAY_MS the RF gate opens.
 *                    drop -> RF gate closes (edge ramp) -> after edge time + TX_SEQ_RF_OFF_MARGIN_MS TXout off.
 *   CW: the debounced key goes to TXout at once and to the RF gate through a fixed TX_SEQ_RELAY_MS delay, so EVERY
 *       element keeps its exact length (constant 5 ms latency) and the relay is always made before RF. After the
 *       last key-up TXout is held for {H} ms (TX hold, min = relay delay + edge + margin) so the relays do not
 *       follow the keying.
 *   On every RX->TX transition "{M}" (mute RX) is sent to the plugin, on every TX->RX "{N}" - as the Nano did.
 *
 * ---- RF edges {E<ms>} ----
 * Every RF on/off (CW element, PTT, {U}/{D}) is shaped by a Gaussian edge (integral of a Gaussian, i.e. an erf curve,
 * +/-2 sigma over the edge time) of {E} ms, default 5 ms, range 0.5-20 ms, decimals allowed.
 *
 * ---- {Lxxx} output level ----
 * |xxx| = % of envelope VOLTAGE, clamped 0..100, applied just before the predistort LUT so every upstream stage keeps
 * working relative to 100 %. L < 0 = diagnostic: LocalPAactive = false, TXout held off, DDS still driven. Boot: L0.
 * Level changes slew linearly (5 ms full scale), independent of the keying edge. Output scale s = level x edge:
 *   out = f + (env - f) * s,  f = floor * min(1, s / TX_LEVEL_FLOOR_FADE);   s = 0 -> exactly no RF.
 * CW (and the digital modes) use envelope 1.0 (= 100 %) and a fixed (CW) or generated (digital) offset, injected after soft-limit (the mic/two-tone DSP keeps
 * running underneath so its filters stay warm). The chirp ('w') path is neither scaled nor gated.
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
#define TX_MODE_CW       256
#define TX_MODE_BOOT     TX_MODE_USB

typedef enum { TX_SEQ_RX = 0, TX_SEQ_KEYING, TX_SEQ_TX, TX_SEQ_UNKEYING } tx_seq_state_t;

void tx_control_init(void);          // TXout low, L0, mode USB, inputs; call first in setup()
void tx_control_service(void);       // loop(): prints {M}/{N} and PTT/key messages flagged by the tick

// dsp_task, once per full tick. Runs the sequencer (and the digital-mode generator); returns true when this tick
// should send a carrier (envelope 1.0, frequency offset tx_carrier_freq_dev_hz()) instead of the DSP output:
// CW (offset 0) and the digital modes (offset = the generator's tone, Hz).
bool tx_control_tick(void);
float tx_carrier_freq_dev_hz(void);

// {U}/{D}
void tx_request_set(bool on);
bool tx_request_get(void);
bool tx_out_get(void);

// {M}
void tx_mode_set(int mode);
int  tx_mode_get(void);
bool tx_mode_is_voice(int mode);
const char *tx_mode_name(int mode);

// {H} TX hold after the last CW key-up, ms (0..5000)
void tx_hold_set_ms(int ms);
int  tx_hold_get_ms(void);
// {E} RF edge time, ms (0.5..20)
void  tx_edge_set_ms(float ms);
float tx_edge_get_ms(void);

bool tx_ptt_pressed(void);
bool tx_key_down(void);
tx_seq_state_t tx_seq_state(void);
const char *tx_seq_state_name(void);

// {L}
void tx_level_set(int level);
int  tx_level_get(void);
bool tx_local_pa_active(void);

// SWR trip (called by slow_adc's task): clears the TX request (sequencer ramps RF off and drops TXout), sends
// "{D}{N}" to the plugin and a "-> TX TRIP" line from loop(). reason/values are only for the message.
void tx_trip(const char *reason, float swr, float fwd_w, float rev_w, float rev_pin_mv, float temp_c);

// {T<degC>} PA temperature trip limit (Nano TxMaxTemp)
void tx_temp_max_set(int deg_c);
int  tx_temp_max_get(void);
uint32_t tx_trip_count(void);

// {Q0/1} external PA on/off, {P<%>} external PA drive level (Nano ExtPAon / ExtPAlevel). While on, the RF level
// is |P| instead of |L| (LocalPAactive still follows L's sign) and ExtPAout (EXTPAOUT_GPIO, active high) switches
// with TXout - on at the start of TX if LocalPAactive, off when TX ends (same relay sequencing).
void tx_extpa_set_on(bool on);
bool tx_extpa_get_on(void);
void tx_extpa_set_level(int pct);
int  tx_extpa_get_level(void);
bool tx_extpa_out_get(void);

// Number of RX -> TX starts since boot (lets slow pollers such as the fan notice a TX shorter than their period).
uint32_t tx_start_count(void);

// true while TX is not requested ({D}) and the level is exactly {L0}: the slow-ADC {F}{R}{S}{T} serial report is
// suppressed then (instrumentation.cpp); the ADC readings and any checks on them carry on.
bool tx_report_idle(void);

// dsp_task, once per full tick, after tx_control_tick(): level x edge, floor-aware
float tx_level_apply(float envelope);
float tx_level_current_scale(void);  // level x edge, last tick
