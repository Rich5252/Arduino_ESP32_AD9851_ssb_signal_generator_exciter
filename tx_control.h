#pragma once

/**
 * tx_control.h  (2026-11-06)
 *
 * TX switching and output level, first steps of the TXlink TX sequencing port.
 *
 * ---- TXout ----
 * Same name and job as the Nano's TXout (antenna relay + local 2 W PA power): TXOUT_GPIO (ESP32_hardware.h, GPIO40),
 * ACTIVE HIGH. Driven low as the first thing in setup(); an external pull-down keeps it off before that.
 *   {U} = TX requested, {D} = TX not requested.
 *   TXout = requested AND LocalPAactive, re-evaluated on every {U}, {D} and {L} (user's rule for now, 2026-11-06;
 *   the full Nano key-down sequencing comes later).
 *
 * ---- {Lxxx} output level (TXlink semantics) ----
 * xxx is a signed integer. |xxx| = output level as a % of envelope VOLTAGE (not power: L50 is about a quarter of the
 * L100 power), clamped to 0..100. It sets an artificial "100 % envelope": every stage up to and including ALC and
 * soft-limit keeps working exactly as before relative to full scale, and the result is scaled to the L ceiling just
 * before the predistortion LUT, which then linearises the reduced level. L100 = previous behaviour.
 *   L >= 0: LocalPAactive = true.   L < 0: diagnostic mode, LocalPAactive = false (DDS still driven at |L| %, TXout
 *   stays off).   Boot: L = 0 (no RF) and LocalPAactive = true until the plugin sends {L} (it does on every TX change).
 * Scaling form, so a nonzero envelope floor ('x', envelope_floor.h) still protects the steep low-duty LUT region:
 *   out = f + (env - f) * s,  f = floor * min(1, s / TX_LEVEL_FLOOR_FADE)      (s = |L| / 100)
 * i.e. the floor stays where it is for normal levels and fades out below TX_LEVEL_FLOOR_FADE so that L0 really is
 * zero output. With floor 0 (default) this is just env * s.
 * s is slewed in the DSP task (TX_LEVEL_SLEW_MS for a full 0 -> 100 % change) so an {L} during TX does not click.
 * The chirp ('w') measurement path is not scaled.
 * Not ported: {P}/{Q} external PA level, the Nano's level-PWM curve (that was a hardware DDS level control).
 */

#include <stdbool.h>
#include <stdint.h>

void tx_control_init(void);          // drive TXout low (off), L = 0; call first in setup()

// TX request ({U}/{D}) and the resulting TXout pin
void tx_request_set(bool on);
bool tx_request_get(void);
bool tx_out_get(void);               // actual pin state

// {L}
void tx_level_set(int level);        // signed, as received; magnitude clamped to 0..100
int  tx_level_get(void);             // signed value as last set (after clamping)
bool tx_local_pa_active(void);

// DSP side (dsp_task, once per full tick): slew towards the target and apply to one envelope sample.
float tx_level_apply(float envelope);
float tx_level_current_scale(void);  // for status/diagnostics
