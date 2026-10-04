#pragma once

#include "esp_attr.h"

/**
 * instrumentation.h  (2026-10-04)
 *
 * Machine-readable status fields for the SDR plugin, in the same "{X<value>}" style the TXlink Nano already sends
 * ({F0.0}{R0.00}{S0.0}{T23.63}). One line per period, fields back to back, CR LF terminated, e.g.
 *     {A-12.3}
 * {A} = RF envelope level in dB, always written as a sign plus two integer digits and one decimal, so it is always
 * 5 characters between the letter and the closing brace: "-12.3" is 12.3 dB below the 0 dB mark, "-00.0" is at
 * (or within 0.05 dB of) it, "+02.3" is 2.3 dB OVER it, "-99.9" is the floor. The '+' sign only ever appears for
 * over-range readings (+99.9 max).
 *   Meter point: dsp_task, the envelope right before the predistortion step (after floor/gdeq/ampeq/ALC/soft-limit,
 *   before the 'D' LUT or linear PWM mapping). 0 dB = an envelope of 1.0 there, i.e. the top of the predistort
 *   LUT's input range ("full drive"); above 0 dB the predistort/PWM clamps flat-top the signal. It is a peak
 *   meter: instant attack, then falls at INSTRUMENTATION_LEVEL_DECAY_DB_PER_S; each report is the highest value
 *   since the previous one, so short RF peaks are not missed. With soft-limit on the envelope can never exceed
 *   1.0 (it approaches it), so over-range readings need soft-limit off. With 'D' off, the default linear mapping
 *   (duty = env*0.9+0.2) already clamps at an envelope of about 0.89 (-1 dB), so overdrive can start slightly
 *   below the 0 dB mark in that case.
 * The same {A} field carries whichever meter point is selected; there is no marker in the line saying which.
 *   ADC mode: the AC signal at the ADC output after DC removal, before the digital mic gain ('U'/'Y', which
 *   therefore cannot change it). 0 dB = the AC peak equals half the ADC code range, i.e. the ADC would clip at
 *   exactly that level if the DC level sat exactly at mid-scale. The DC level is deliberately rejected (it is not
 *   accurately known), so headroom lost to a bias offset is NOT shown: with an offset the ADC clips below 0 dB on
 *   the side nearer a rail. The ESP32 ADC front end may also saturate or go nonlinear before the code rails
 *   (not checked here). The DC blocker tracks slowly, so a sudden DC step reads as a brief transient.
 * More fields can be appended to the same line later.
 *
 * Off at boot unless INSTRUMENTATION_DEFAULT_MODE (config.h) says otherwise; '#' cycles off -> RF -> ADC -> off at run time. Called from
 * loop() on Core 1, never from the real-time path.
 */

// Meter point selected with '#': OFF -> RF -> ADC -> OFF (each press advances one step).
enum { INSTR_OFF = 0, INSTR_RF = 1, INSTR_ADC = 2 };

// Real-time side, called once per full dsp_task tick. Cheap: only the selected mode's meter does any work (one
// fabsf, one multiply, two compares); the other call returns after one compare. Never touches Serial.
//   RF : the envelope at the RF meter point (see above).
//   ADC: the ADC output AFTER DC removal (dsp_task's DC blocker) and before the digital mic gain, normalised so
//        +/-1.0 = half the ADC code range about the DC level; mic path only, with another audio source nothing is
//        fed and {A} reads the floor.
void IRAM_ATTR instrumentation_record_envelope(float envelope);
void IRAM_ATTR instrumentation_record_adc(float adc_norm);

// Call every loop() iteration; sends at most one line per INSTRUMENTATION_PERIOD_MS.
void instrumentation_service(void);

int instrumentation_get_mode(void);                 // INSTR_OFF / INSTR_RF / INSTR_ADC
const char* instrumentation_mode_name(int mode);
const char* instrumentation_cycle_mode(void);       // advance to the next mode, returns its name

// Formats "{A-12.3}" (no line ending) into buf (needs >= 9 bytes). Exposed so the format can be host-tested.
void instrumentation_format_level(char *buf, unsigned buf_len, float level_db);
