#pragma once

/**
 * carrier_output.h
 *
 * AD9851 DDS carrier: init, per-tick frequency update (carrier + delayed
 * frequency deviation), and RF output on/off. Only compiled in when
 * AD9851_ATTACHED (see config.h) - mirrors the original .ino's guarding
 * exactly, so this still builds standalone (mic->DSP->DAC, no AD9851
 * board yet) with AD9851_ATTACHED set to 0.
 */

#include "config.h"

#if AD9851_ATTACHED

#include <stdint.h>
#include <stdbool.h>
#include "ad9851.h"

#define AD9851_PIN_DATA   10
#define AD9851_PIN_WCLK   12
#define AD9851_PIN_FQUD   11
#define AD9851_PIN_RESET  9
#define REF_CLK_HZ        30000000u
// ---- Carrier frequency and calibration (2026-10-12) ----
// CARRIER_NOMINAL_HZ is the frequency wanted on the air (boot default; changed at run time with the serial
// command {F<Hz>}, see serial_commands.cpp / brace_input.h). The AD9851 is programmed with exactly that frequency;
// the calibration is NOT a Hz offset any more but a proportional correction of the assumed DDS clock, in parts per
// billion, exactly as in the Nano library (cjheath AD9851.h setClock(), used in TXlink4_Nano):
//     calibrated_clock = (6 * REF_CLK_HZ) * (1e9 - CARRIER_CAL_PPB) / 1e9
// (see ad9851_set_clock_cal_ppb(), AD9851.h). Positive CARRIER_CAL_PPB = the real clock is slower than assumed, so
// the output rises. Changed at run time with {C<ppb>}.
//
// History: the first calibration here was a fixed offset, +160Hz (confirmed against the user's calibrated
// receiver at 14.2MHz), changed to +162 on 2026-09-30 after an SDRuno I/Q capture put the carrier ~2.1Hz low (good
// to roughly +/-0.25Hz, the SDR's own reference not independently checked). A fixed Hz offset is only right at the
// frequency it was measured on (a reference error is proportional to frequency), and the user found it wrong when
// checking another frequency on 2026-10-12. The starting value below is just the old +162Hz at 14.2MHz expressed as
// a proportion: 162 / 14200000 = 11408 ppb. It is NOT a new measurement; re-trim with {C<ppb>} using
//     new_ppb = old_ppb + (f_set - f_measured) / f_set * 1e9.
#define CARRIER_NOMINAL_HZ  14200000u
#define CARRIER_CAL_PPB     11408

// Range accepted for the nominal frequency (before calibration). 1MHz..60MHz covers 160m..6m; the AD9851 with its
// 6x multiplier (180MHz clock) is specified for output up to about 70MHz. The envelope predistort LUT, gdeq,
// relative delay etc. were all measured at 14.2MHz and are not re-tuned per band.
#define CARRIER_MIN_HZ      1000000u
#define CARRIER_MAX_HZ      60000000u

// Inits the AD9851 driver, applies the clock calibration (CARRIER_CAL_PPB) and sets the initial carrier frequency
// (CARRIER_NOMINAL_HZ). Call once from setup().
void carrier_output_init(void);

// tx_freq = carrier + (int32_t)delayed_freq_dev_hz, sent to the chip via
// ad9851_set_frequency(). Returns the exact integer Hz value sent, so the
// caller can feed it straight to diagnostics_set_tx_info() - ground truth
// for what the chip is actually asked to produce, independent of any of
// the upstream DSP/delay-line reasoning. Call once per dsp_task tick.
uint32_t IRAM_ATTR carrier_output_set_freq_dev(float delayed_freq_dev_hz);

// 2026-09-19: split OUT of carrier_output_set_freq_dev() above, folded in
// from minimal_fs_test_step6_isr_write after real-hardware bench
// confirmation - see config.h's AD9851_ISR_WRITE_ENABLED comment and
// null_bias_investigation.md/moving_forward_notes.md's 2026-09-19 entries
// for the full write-time-jitter story. carrier_output_set_freq_dev() does
// `s_carrier_hz + (int32_t)delayed_freq_dev_hz` - a float-to-int cast,
// which uses Xtensa's floating-point coprocessor. Calling it directly from
// an ISR panics with a Coprocessor exception (confirmed on real hardware,
// EXCCAUSE 0x4, during this fix's own bench validation) - this project's
// history had already named this exact failure mode ("v2's ISR crash", see
// group_delay_fit_notes.md) before step6 first repeated it. The fix: do
// the float arithmetic in TASK context (safe - FPU use is normal there)
// via carrier_output_compute_tx_freq(), and pass the ALREADY-INTEGER
// result to carrier_output_set_freq_dev_raw() - pure register writes
// underneath, no float, no coprocessor - which IS safe to call from an
// ISR (dsp_task's own on_timer_alarm() now does exactly this when
// AD9851_ISR_WRITE_ENABLED). carrier_output_set_freq_dev() above is
// unchanged and still does both steps together, for every other caller.
uint32_t IRAM_ATTR carrier_output_compute_tx_freq(float delayed_freq_dev_hz);
void IRAM_ATTR carrier_output_set_freq_dev_raw(uint32_t tx_freq);

// Sets the NOMINAL carrier frequency in Hz (the calibration offset is added internally). Returns false and changes
// nothing if outside CARRIER_MIN_HZ..CARRIER_MAX_HZ. Safe to call from loop() while dsp_task runs: it only updates
// a 32-bit value that dsp_task reads fresh every tick (carrier_output_compute_tx_freq()), so the new frequency
// takes effect on the next tick; the chip is not written from here.
bool carrier_output_set_freq_hz(uint32_t nominal_hz);
// Clock calibration in ppb (see above). Returns false and changes nothing if out of range
// (+/-AD9851_CLOCK_CAL_PPB_MAX). Not saved across reboots.
bool carrier_output_set_cal_ppb(int32_t cal_ppb);
int32_t carrier_output_get_cal_ppb(void);
uint32_t carrier_output_get_freq_hz(void);          // nominal frequency currently set (without calibration)

bool carrier_output_get_rf_enabled(void);
void carrier_output_set_rf_enabled(bool enable);

// Thin wrapper over ad9851_get_profile() - see ad9851_profile_t (AD9851.h)
// for what max_prep_us/max_spi_us each cover. Exposed here so
// diagnostics.cpp doesn't need its own direct handle to the AD9851
// module, matching how everything else in [timing] is reached.
void carrier_output_get_profile(ad9851_profile_t *out);

// 2026-09-09: canary check for the random-TX-jump investigation (see
// moving_forward_notes.md). s_carrier_hz is one of the two persistent values that theory points at, alongside
// ad9851_canary_t's ftw_reciprocal. It used to be written once at init and compared against the compile-time
// CARRIER_HZ; now that it can legitimately change at run time ({F}), a second copy is kept as a bitwise
// complement, updated together with it, and the canary compares against that. This catches a stray write to either
// copy, but unlike the old compile-time comparison it cannot catch a stray write that hits both consistently.
// carrier_output_get_carrier_hz() returns the live value (the frequency handed to the AD9851 before deviation);
// carrier_output_get_carrier_hz_expected() returns what the shadow copy says it should be.
uint32_t carrier_output_get_carrier_hz(void);
uint32_t carrier_output_get_carrier_hz_expected(void);

// Thin wrapper over ad9851_get_canary() - see its doc comment (AD9851.h).
void carrier_output_get_canary(ad9851_canary_t *out);

#endif // AD9851_ATTACHED
