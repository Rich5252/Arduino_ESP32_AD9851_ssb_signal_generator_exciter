#include "instrumentation.h"
#include "config.h"
#include <Arduino.h>
#include <math.h>
#include <stdio.h>

static volatile int s_mode = INSTRUMENTATION_DEFAULT_MODE;   // INSTR_OFF / INSTR_RF / INSTR_ADC
static uint32_t s_last_ms = 0;

// ---- Peak meters (2026-10-04). Each is written by dsp_task and read from the Core 1 loop: single 32-bit
// float/uint32 accesses, the only possible race is losing one peak at a report boundary. Only the meter of the
// selected mode is fed (the other record function returns at once), so a deselected meter costs one load + compare
// per tick. ----
typedef struct {
    volatile float    level;      // peak follower: instant attack, exponential decay (linear, 1.0 = 0 dB)
    volatile float    report;     // highest level since the last report
    volatile uint32_t samples;    // ticks seen, so a stalled feed reads as the floor, not a frozen value
    uint32_t          samples_at_take;
} meter_t;

static meter_t s_rf_meter  = {0.0f, 0.0f, 0, 0};
static meter_t s_adc_meter = {0.0f, 0.0f, 0, 0};
static const float s_decay_coef = powf(10.0f, -INSTRUMENTATION_LEVEL_DECAY_DB_PER_S / (20.0f * (float)SAMPLE_RATE_HZ));

static inline void IRAM_ATTR meter_feed(meter_t *m, float x)
{
    float a = fabsf(x);                            // gdeq/ampeq can undershoot slightly negative; ADC swings both ways
    float e = m->level * s_decay_coef;
    if (a > e) e = a;
    if (e < 1e-6f) e = 0.0f;                       // keep the decaying value out of denormals
    m->level = e;
    if (e > m->report) m->report = e;
    m->samples = m->samples + 1;
}

void IRAM_ATTR instrumentation_record_envelope(float envelope)
{
    if (s_mode != INSTR_RF) return;
    meter_feed(&s_rf_meter, envelope);
}

void IRAM_ATTR instrumentation_record_adc(float adc_norm)
{
    if (s_mode != INSTR_ADC) return;
    meter_feed(&s_adc_meter, adc_norm);
}

#define LEVEL_FLOOR_DB (-99.9f)

// Highest level since the previous call, in dB re 1.0, clamped to [LEVEL_FLOOR_DB, +99.9]. Positive = over-range.
static float take_level_db(meter_t *m)
{
    if (m->samples == m->samples_at_take) {
        m->report = 0.0f;
        return LEVEL_FLOOR_DB;                     // nothing was fed since the last report
    }
    m->samples_at_take = m->samples;
    float p = m->report;
    m->report = m->level;                          // next interval starts from the current (decayed) level
    if (p < 1e-5f) return LEVEL_FLOOR_DB;
    float db = 20.0f * log10f(p);
    if (db > 99.9f) db = 99.9f;
    if (db < LEVEL_FLOOR_DB) db = LEVEL_FLOOR_DB;
    return db;
}

int instrumentation_get_mode(void) { return s_mode; }

const char* instrumentation_mode_name(int mode)
{
    switch (mode) {
        case INSTR_RF:  return "RF envelope (0 dB = 1.0 at the predistort input)";
        case INSTR_ADC: return "ADC output (0 dB = ADC full scale, both rails)";
        default:        return "off";
    }
}

const char* instrumentation_cycle_mode(void)
{
    int next = (s_mode + 1) % 3;                   // OFF -> RF -> ADC -> OFF
    // Re-arm the meter being switched to: discard any stale peak/sample count, so the first message is current.
    meter_t *m = (next == INSTR_RF) ? &s_rf_meter : (next == INSTR_ADC) ? &s_adc_meter : nullptr;
    if (m) {
        m->level = 0.0f;
        m->report = 0.0f;
        m->samples_at_take = m->samples;
    }
    s_last_ms = millis();
    s_mode = next;
    return instrumentation_mode_name(next);
}

void instrumentation_format_level(char *buf, unsigned buf_len, float level_db)
{
    // Always sign + two integer digits + one decimal. Round to tenths first and take the sign from the rounded
    // value: anything that rounds to 0.0 prints as "-00.0" (never "+00.0", and never a doubled minus from -0.0f),
    // and '+' only appears for a reading of at least +0.1 dB, i.e. a real over-range.
    long tenths = lroundf(level_db * 10.0f);
    char sign = (tenths > 0) ? '+' : '-';
    long mag_t = (tenths < 0) ? -tenths : tenths;
    if (mag_t > 999) mag_t = 999;
    snprintf(buf, buf_len, "{A%c%02ld.%ld}", sign, mag_t / 10, mag_t % 10);
}

void instrumentation_service(void)
{
    int mode = s_mode;
    if (mode == INSTR_OFF) return;
    uint32_t now = millis();
    if ((uint32_t)(now - s_last_ms) < (uint32_t)INSTRUMENTATION_PERIOD_MS) return;
    s_last_ms = now;

    char buf[16];
    instrumentation_format_level(buf, sizeof buf, take_level_db(mode == INSTR_ADC ? &s_adc_meter : &s_rf_meter));
    Serial.print(buf);
    Serial.print("\r\n");
}
