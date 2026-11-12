#pragma once

/**
 * slow_adc_conv.h  (2026-11-04)
 *
 * Pure maths for the slow ADC channels (forward power, reflected power, PA temperature), ported from the TXlink
 * Nano sketch (TXlink4_Nano_Latest/TXlink4_Nano.ino: SendSWR() and calcNTC()), plus the formatter for the
 * "{F..}{R..}{S..}{T..}" fields the SDR plugin already understands.
 *
 * Inputs are the voltage at the ESP32-S3 pin, in volts. The Nano measured 0-5 V with a 10-bit ADC; every constant
 * below was fitted on THAT hardware. To keep the Nano curves usable until the ESP32 front end is designed and
 * recalibrated, each detector input is first multiplied by a scale factor (pin volts -> "volts the Nano would have
 * seen"), e.g. 5.0/3.1 if a divider squeezes the Nano's 0-5 V range into 0-3.1 V, or 1.0 if the detector output
 * is wired straight in and never exceeds about 3.1 V. These are PLACEHOLDERS: the detector curves need re-measuring
 * on the new PA/coupler before the numbers mean anything.
 *
 * No Arduino/ESP dependencies: host-tested by host_tests/test_slow_adc_conv.cpp.
 */

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float fwd_scale;        // pin volts -> Nano-equivalent volts, forward detector
    float rev_scale;        // pin volts -> Nano-equivalent volts, reflected detector
    float ntc_supply_v;     // voltage the NTC pull-up resistor goes to (3.3 V on the ESP32 board)
    float ntc_pullup_ohm;   // pull-up resistor (Nano code: 51k)
    float ntc_r0_ohm;       // NTC resistance at 25 degC (Nano: 50k)
    float ntc_beta;         // NTC beta (Nano: 3950)
} slow_adc_conv_cfg_t;

typedef struct {
    float fwd_w;            // forward power, W      -> {F%.1f}
    float rev_w;            // reflected power, W    -> {R%.2f}
    float swr;              // 0 when not computable -> {S%.1f}
    float temp_c;           // PA temperature, degC  -> {T%.2f}
} slow_adc_reading_t;

// Forward power from the averaged forward-detector pin voltage. Same curve as the Nano (34.232*Vrms + 2.2884).
float slow_adc_fwd_watts(float pin_v, const slow_adc_conv_cfg_t *cfg);

// Reflected power from the averaged reflected-detector pin voltage. Same curve as the Nano, including its diode
// knee linearisation below 100 counts (Nano 10-bit counts) and the halving above 30 MHz.
float slow_adc_rev_watts(float pin_v, uint32_t carrier_hz, const slow_adc_conv_cfg_t *cfg);

// SWR from the two powers, exactly as the Nano: 0 when reflected >= forward (or both 0).
float slow_adc_swr(float fwd_w, float rev_w);

// NTC to degC (beta equation), pull-up to ntc_supply_v, NTC to ground. Returns -99.0 for an open/shorted input.
float slow_adc_ntc_degc(float pin_v, const slow_adc_conv_cfg_t *cfg);

// SWR safety trip decision (2026-11-11), Nano rule plus two extra tests:
//   REV_SATURATED: REV pin at/above rev_sat_mv - the ADC is at the top of its range, so the real reflected power (and
//                  SWR) can be anything above what is shown -> abort whatever the forward power.
//   REV_GE_FWD:    forward >= fwd_min_w and reflected >= forward. The Nano's SWR formula returns 0 here, so its
//                  "SWR > max" test never fired for a total mismatch; treated as infinite SWR.
//   HIGH:          forward >= fwd_min_w and SWR > swr_max (Nano: SWRfwdMin 1 W, SWRmax 3).
typedef enum { SWR_TRIP_NONE = 0, SWR_TRIP_HIGH, SWR_TRIP_REV_GE_FWD, SWR_TRIP_REV_SATURATED } swr_trip_t;
swr_trip_t slow_adc_swr_trip_check(float fwd_w, float rev_w, float swr, float rev_pin_mv,
                                   float swr_max, float fwd_min_w, float rev_sat_mv);
const char *slow_adc_swr_trip_name(swr_trip_t t);

// PA temperature trip decision (2026-11-11). Nano: TempDeg >= TxMaxTemp (default 54 degC, {T}) -> abort.
// Extra: slow_adc_ntc_degc() returns -99 for an open or shorted sensor; with fault_trips set that is an abort too
// (a shorted NTC really means "very hot", an open one hides any overheating).
typedef enum { TEMP_TRIP_NONE = 0, TEMP_TRIP_OVER, TEMP_TRIP_SENSOR_FAULT } temp_trip_t;
temp_trip_t slow_adc_temp_trip_check(float temp_c, float max_c, bool fault_trips);
// 2026-11-12: as above, but a sensor fault must be seen in fault_windows consecutive calls (one per published average)
// before it trips. *fault_streak is the caller's counter: incremented on a fault reading, cleared on a good one. Call
// it on every average (TX on or off) so a fault already present when TX starts still needs no extra wait.
temp_trip_t slow_adc_temp_trip_check_streak(float temp_c, float max_c, bool fault_trips, uint32_t *fault_streak,
                                            uint32_t fault_windows);
const char *slow_adc_temp_trip_name(temp_trip_t t);

// All four values from the three averaged pin voltages.
void slow_adc_convert(float fwd_v, float rev_v, float temp_v, uint32_t carrier_hz,
                      const slow_adc_conv_cfg_t *cfg, slow_adc_reading_t *out);

// Writes "{F1.8}{R0.08}{S1.5}{T25.00}" (no line ending) into buf. Returns the number of characters written
// (0 if buf is too small; 48 bytes is always enough).
int slow_adc_format_fields(char *buf, size_t buf_len, const slow_adc_reading_t *r);

#ifdef __cplusplus
}
#endif
