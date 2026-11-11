#pragma once

/**
 * mode_defaults.h  (2026-11-11)
 *
 * Default settings for each {M} TX mode. A preset (settings.h, '0'-'9') holds the whole audio/DSP/output tuning, but
 * not the few settings below, so selecting a mode loads its preset and then sets those directly:
 *
 *   mode group                  preset (config.h)          then also
 *   M32 USB, M64 LSB            MODE_PRESET_VOICE   (9)    sideband (tx_mode_set)
 *   M128 Tune                   MODE_PRESET_TUNE    (7)    two-tone phase generator MODE_TUNE_PHASE_GEN (LEGACY),
 *                                                          tone pair / amplitude ratio / dither back to boot values,
 *                                                          tone generator + DSP restart (as the 'O' key)
 *   M256 CW, Nano 1/2/4/8/16    MODE_PRESET_CARRIER (9)    -
 *   every mode                                             duty override ('d') off - a bench tool that bypasses the
 *                                                          envelope (and so the TX gating) while on
 *
 * Not touched (global, not per mode): {L} level, {H} hold, {E} edge, {T} temp limit, ExtPA, fan, carrier frequency
 * and calibration, diagnostics. Applied only when the mode CHANGES (and once at boot); the caller then calls
 * tx_mode_set(). Loop context (Core 1) only. A mode number tx_control does not know loads nothing.
 */

// Applies the defaults for `mode` (TX_MODE_*). Returns the preset loaded, or -1 when none.
int mode_defaults_apply(int mode);
