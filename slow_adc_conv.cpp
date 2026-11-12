/**
 * slow_adc_conv.cpp - see slow_adc_conv.h. Ported from the TXlink Nano's SendSWR()/calcNTC() with the same
 * constants; only the input changed (pin volts times a scale factor, instead of 10-bit counts of 0-5 V).
 */

#include "slow_adc_conv.h"
#include <math.h>
#include <stdio.h>

// The Nano's input in its own units: 10-bit counts of a 5 V full scale.
static float nano_counts(float nano_v) { return nano_v / 5.0f * 1023.0f; }

float slow_adc_fwd_watts(float pin_v, const slow_adc_conv_cfg_t *cfg)
{
    float raw = nano_counts(pin_v * cfg->fwd_scale);
    if (raw < 2.0f) return 0.0f;                        // Nano: minRaw = 2 counts
    float vrms = raw / 1023.0f * 5.0f * 0.7071f;        // detector peak -> rms
    vrms = 34.232f * vrms + 2.2884f;                    // Nano's measured scaling to Vrms at the 50 ohm load
    return vrms * vrms / 50.0f;
}

float slow_adc_rev_watts(float pin_v, uint32_t carrier_hz, const slow_adc_conv_cfg_t *cfg)
{
    float raw = nano_counts(pin_v * cfg->rev_scale);
    bool hi_reading = true;
    if (raw < 2.0f) raw = 0.0f;
    if (raw > 0.0f && raw < 100.0f) {                   // Nano: linearise low readings (diode knee)
        raw = raw * 12.549f * powf(raw, -0.555f);
        hi_reading = false;
    }
    float v = raw / 1023.0f * 5.0f;
    float w = (v / 0.0932f) * (v / 0.0932f) / 50.0f;
    if (hi_reading && w > 0.0f) w = w * 0.79f + 0.59f;  // Nano: further linear scale
    if (carrier_hz > 30000000u) w = w / 2.0f;           // Nano: compensate high reading on 6 m
    return w;
}

float slow_adc_swr(float fwd_w, float rev_w)
{
    float a = sqrtf(fabsf(fwd_w));
    float b = sqrtf(fabsf(rev_w));
    if (a - b > 0.0f) return (a + b) / (a - b);
    return 0.0f;
}

float slow_adc_ntc_degc(float pin_v, const slow_adc_conv_cfg_t *cfg)
{
    // NTC from the pin to ground, pull-up from the pin to ntc_supply_v: R = Rpull * V / (Vsup - V).
    // Open sensor: the pin sits at the rail, but the ADC (12 dB atten.) tops out near 3.1 V, so "at the rail" can
    // never be seen exactly - treat anything above 92 % of the supply (3.04 V at 3.3 V; NTC above ~11x the pull-up,
    // i.e. below about -30 degC for 50k/3950) as open. Shorted: below 1 mV.
    if (pin_v <= 0.001f || pin_v >= 0.92f * cfg->ntc_supply_v) return -99.0f;
    float r = cfg->ntc_pullup_ohm * pin_v / (cfg->ntc_supply_v - pin_v);
    // Same constants as the Nano's calcNTC() (T0 = 298 K, 273 offset), so readings compare directly.
    float t = 1.0f / ((1.0f / 298.0f) + (1.0f / cfg->ntc_beta) * logf(r / cfg->ntc_r0_ohm));
    return t - 273.0f;
}

swr_trip_t slow_adc_swr_trip_check(float fwd_w, float rev_w, float swr, float rev_pin_mv,
                                   float swr_max, float fwd_min_w, float rev_sat_mv)
{
    if (rev_pin_mv >= rev_sat_mv) return SWR_TRIP_REV_SATURATED;
    if (fwd_w >= fwd_min_w) {
        if (rev_w >= fwd_w) return SWR_TRIP_REV_GE_FWD;
        if (swr > swr_max)  return SWR_TRIP_HIGH;
    }
    return SWR_TRIP_NONE;
}

const char *slow_adc_swr_trip_name(swr_trip_t t)
{
    switch (t) {
    case SWR_TRIP_HIGH:          return "SWR above limit";
    case SWR_TRIP_REV_GE_FWD:    return "reflected >= forward";
    case SWR_TRIP_REV_SATURATED: return "REV input at ADC full scale";
    default:                     return "none";
    }
}

temp_trip_t slow_adc_temp_trip_check(float temp_c, float max_c, bool fault_trips)
{
    if (temp_c <= -98.0f) return fault_trips ? TEMP_TRIP_SENSOR_FAULT : TEMP_TRIP_NONE;
    if (temp_c >= max_c)  return TEMP_TRIP_OVER;
    return TEMP_TRIP_NONE;
}

temp_trip_t slow_adc_temp_trip_check_streak(float temp_c, float max_c, bool fault_trips, uint32_t *fault_streak,
                                            uint32_t fault_windows)
{
    if (temp_c <= -98.0f) {
        if (*fault_streak < 0xFFFFFFFFu) *fault_streak = *fault_streak + 1;
        if (!fault_trips) return TEMP_TRIP_NONE;
        return (*fault_streak >= (fault_windows ? fault_windows : 1)) ? TEMP_TRIP_SENSOR_FAULT : TEMP_TRIP_NONE;
    }
    *fault_streak = 0;
    return (temp_c >= max_c) ? TEMP_TRIP_OVER : TEMP_TRIP_NONE;
}

const char *slow_adc_temp_trip_name(temp_trip_t t)
{
    switch (t) {
    case TEMP_TRIP_OVER:         return "PA temperature at/above limit";
    case TEMP_TRIP_SENSOR_FAULT: return "PA temperature sensor open/shorted";
    default:                     return "none";
    }
}

void slow_adc_convert(float fwd_v, float rev_v, float temp_v, uint32_t carrier_hz,
                      const slow_adc_conv_cfg_t *cfg, slow_adc_reading_t *out)
{
    out->fwd_w  = slow_adc_fwd_watts(fwd_v, cfg);
    out->rev_w  = slow_adc_rev_watts(rev_v, carrier_hz, cfg);
    out->swr    = slow_adc_swr(out->fwd_w, out->rev_w);
    out->temp_c = slow_adc_ntc_degc(temp_v, cfg);
}

int slow_adc_format_fields(char *buf, size_t buf_len, const slow_adc_reading_t *r)
{
    // Same decimals as the Nano: F 1, R 2, S 1, T 2. Values are clamped so a wild input can never overflow the
    // field widths the plugin has seen before.
    float f = r->fwd_w,  rv = r->rev_w, s = r->swr, t = r->temp_c;
    if (!(f >= 0.0f)) f = 0.0f;                     // !(x >= 0) also catches NaN
    if (f > 9999.9f) f = 9999.9f;
    if (!(rv >= 0.0f)) rv = 0.0f;
    if (rv > 9999.99f) rv = 9999.99f;
    if (!(s >= 0.0f)) s = 0.0f;
    if (s > 999.9f) s = 999.9f;
    if (t != t) t = -99.0f;
    if (t < -99.0f) t = -99.0f;
    if (t > 999.0f) t = 999.0f;
    int n = snprintf(buf, buf_len, "{F%.1f}{R%.2f}{S%.1f}{T%.2f}", f, rv, s, t);
    if (n < 0 || (size_t)n >= buf_len) { if (buf_len) buf[0] = 0; return 0; }
    return n;
}
