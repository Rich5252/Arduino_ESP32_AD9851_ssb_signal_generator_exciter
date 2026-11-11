/**
 * slow_adc.cpp - see slow_adc.h.  (2026-11-04; driver mode bench-run, split mode NOT yet compiled or run on hardware)
 */

#include "slow_adc.h"
#include "config.h"

// Fallback so an older config.h (without this line) still builds: reads run at boot.
#ifndef SLOW_ADC_READS_DEFAULT
#define SLOW_ADC_READS_DEFAULT 1
#endif
#ifndef SLOW_ADC_READ_MODE
#define SLOW_ADC_READ_MODE 1          // 1 = split register-level read (no blocking), 0 = IDF adc_oneshot_read()
#endif
#include <Arduino.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#if SLOW_ADC_ENABLED

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "soc/sens_struct.h"          // SENS.sar_meas2_ctrl2 - ADC2 RTC-controller register, split read below
#include "diagnostics.h"              // late tick / stale commit counters shown by {K}
#include "tx_control.h"               // SWR trip action
#if AD9851_ATTACHED
#include "carrier_output.h"
#endif

typedef enum { CH_DUMMY = 0, CH_ADC2, CH_REFUSED } ch_kind_t;

typedef struct {
    const char       *name;
    int               gpio;
    float             dummy_mv;
    ch_kind_t         kind;
    adc_channel_t     chan;
    adc_cali_handle_t cali;          // NULL -> linear raw*3100/4095 fallback
    // written by the task only
    float             sum_mv;
    uint32_t          n;
    uint32_t          reads, errors;
    int               last_raw;
} slow_ch_t;

static slow_ch_t s_ch[SLOW_ADC_NCH] = {
    { "FWD",  SLOW_ADC_FWD_GPIO,  SLOW_ADC_DUMMY_FWD_MV,  CH_DUMMY, ADC_CHANNEL_0, NULL, 0, 0, 0, 0, -1 },
    { "REV",  SLOW_ADC_REV_GPIO,  SLOW_ADC_DUMMY_REV_MV,  CH_DUMMY, ADC_CHANNEL_0, NULL, 0, 0, 0, 0, -1 },
    { "TEMP", SLOW_ADC_TEMP_GPIO, SLOW_ADC_DUMMY_TEMP_MV, CH_DUMMY, ADC_CHANNEL_0, NULL, 0, 0, 0, 0, -1 },
};

static adc_oneshot_unit_handle_t s_adc2 = NULL;
static TaskHandle_t s_task = NULL;
static volatile bool s_report_on = (SLOW_ADC_REPORT_DEFAULT != 0);
// 2026-11-04: run-time switch for the reads themselves ({K2}/{K3}). While paused the task is blocked on a task
// notification (no timer wake-ups, no ADC access at all) and the report sends no slow fields.
static volatile bool s_reads_on = (SLOW_ADC_READS_DEFAULT != 0);
static volatile bool s_paused_ack = false;          // set by the task once it is actually blocked

// Latest completed average, published by the task with a sequence counter (odd while being written) so the
// Core 1 reader never sees a half-updated set and no lock is taken on either side.
static volatile uint32_t s_seq = 0;
static volatile float    s_avg_mv[SLOW_ADC_NCH];
static volatile uint32_t s_windows = 0;              // completed averages since boot

// Read-time statistics over real ADC2 reads (dummy channels do not count).
static volatile uint32_t s_read_max_us = 0;
static volatile uint64_t s_read_sum_us = 0;
static volatile uint32_t s_read_count = 0;
static volatile uint32_t s_burst_late = 0;           // bursts whose start slipped a whole period

static const slow_adc_conv_cfg_t s_conv = {
    SLOW_ADC_FWD_SCALE, SLOW_ADC_REV_SCALE,
    SLOW_ADC_NTC_SUPPLY_V, SLOW_ADC_NTC_PULLUP_OHM, SLOW_ADC_NTC_R0_OHM, SLOW_ADC_NTC_BETA
};

static float dummy_mv(int idx, slow_ch_t *c)
{
    if (idx != SLOW_ADC_FWD) return c->dummy_mv;
    // Forward: slow +/-10 % triangle, 8 s period, so the plugin's display visibly moves.
    uint32_t ms = (uint32_t)(esp_timer_get_time() / 1000) % 8000u;
    float tri = (ms < 4000u) ? (ms / 4000.0f) : ((8000u - ms) / 4000.0f);   // 0..1..0
    return c->dummy_mv * (0.9f + 0.2f * tri);
}

static void raw_to_mv(slow_ch_t *c, int raw, float *mv)
{
    c->last_raw = raw;
    int v = 0;
    if (c->cali && adc_cali_raw_to_voltage(c->cali, raw, &v) == ESP_OK) *mv = (float)v;
    else *mv = raw * 3100.0f / 4095.0f;
}

static void note_time(uint32_t dt)
{
    if (dt > s_read_max_us) s_read_max_us = dt;
    s_read_sum_us = s_read_sum_us + dt;
    s_read_count = s_read_count + 1;
}

// ---- Mode 0: IDF driver read. Blocking: setup + conversion + busy-wait inside portENTER_CRITICAL(&rtc_spinlock).
// Bench 2026-11-04: late_ticks and stale_commits both rose by exactly one per read (24/s at 24 reads/s). ----
static bool read_mv_driver(slow_ch_t *c, float *mv)
{
    int raw = -1;
    int64_t t0 = esp_timer_get_time();
    esp_err_t e = adc_oneshot_read(s_adc2, c->chan, &raw);
    note_time((uint32_t)(esp_timer_get_time() - t0));
    c->reads++;
    if (e != ESP_OK || raw < 0) { c->errors++; return false; }   // ESP_ERR_TIMEOUT = busy / arbiter said invalid
    raw_to_mv(c, raw, mv);
    return true;
}

// ---- Mode 1: split register-level read (2026-11-04). ADC2 is configured once at boot by the driver (attenuation,
// calibration code, SAR clock divider, RTC controller selection, arbiter: all left in place afterwards - nothing else
// in this firmware touches ADC2). Then each conversion is: select the pad and pulse meas2_start_sar (two register
// writes, no wait, no critical section); the result is collected on the task's NEXT wake, tens of ms later, long
// after the conversion finished. Register layout: ESP32-S3 sens_struct.h (IDF v5.5) sar_meas2_ctrl2 =
//   [15:0] meas2_data_sar (bits 12:0 data, 15:14 flag, 0 = valid), [16] done, [17] start, [18] start_force,
//   [30:19] sar2_en_pad, [31] sar2_en_pad_force.
// Whole-register reads/writes (.val) are used so the compiler never emits byte/halfword accesses to the peripheral.
#define MEAS2_DONE_BIT    (1u << 16)
#define MEAS2_START_BIT   (1u << 17)
#define MEAS2_PAD_SHIFT   19
#define MEAS2_PAD_MASK    (0xFFFu << MEAS2_PAD_SHIFT)

static void split_start(adc_channel_t chan)
{
    uint32_t v = SENS.sar_meas2_ctrl2.val;
    v = (v & ~MEAS2_PAD_MASK) | ((1u << (unsigned)chan) << MEAS2_PAD_SHIFT);
    v &= ~MEAS2_START_BIT;
    SENS.sar_meas2_ctrl2.val = v;               // pad selected, start low
    SENS.sar_meas2_ctrl2.val = v | MEAS2_START_BIT;   // rising edge starts the conversion
}

// Returns 1 = got a valid raw value, 0 = not done yet (error), -1 = done but flagged invalid (arbiter / controller).
static int split_collect(int *raw)
{
    uint32_t v = SENS.sar_meas2_ctrl2.val;
    if (!(v & MEAS2_DONE_BIT)) return 0;
    uint32_t data = v & 0xFFFFu;
    if ((data >> 14) & 3u) return -1;
    *raw = (int)(data & 0x1FFFu);
    return 1;
}

static uint32_t carrier_hz(void);   // defined further down

// 2026-11-11 safety trips on every published average (rules in slow_adc_conv.h): SWR (needs real FWD and REV
// inputs) and PA temperature (needs a real TEMP input; limit from {T}). Only while TX is requested - once per over,
// re-armed by the next {U}; a condition that persists trips again within ~250 ms of that {U}.
static void check_trips(void)
{
    if (!tx_request_get()) return;
    slow_adc_reading_t r;
    slow_adc_convert(s_avg_mv[SLOW_ADC_FWD] / 1000.0f, s_avg_mv[SLOW_ADC_REV] / 1000.0f,
                     s_avg_mv[SLOW_ADC_TEMP] / 1000.0f, carrier_hz(), &s_conv, &r);
#if SWR_TRIP_ENABLED
    if (s_ch[SLOW_ADC_FWD].kind == CH_ADC2 && s_ch[SLOW_ADC_REV].kind == CH_ADC2) {
        swr_trip_t t = slow_adc_swr_trip_check(r.fwd_w, r.rev_w, r.swr, s_avg_mv[SLOW_ADC_REV],
                                               SWR_TRIP_MAX, SWR_TRIP_FWD_MIN_W, SWR_TRIP_REV_SAT_MV);
        if (t != SWR_TRIP_NONE) {
            tx_trip(slow_adc_swr_trip_name(t), r.swr, r.fwd_w, r.rev_w, s_avg_mv[SLOW_ADC_REV], r.temp_c);
            return;
        }
    }
#endif
#if TEMP_TRIP_ENABLED
    if (s_ch[SLOW_ADC_TEMP].kind == CH_ADC2) {
        temp_trip_t t = slow_adc_temp_trip_check(r.temp_c, (float)tx_temp_max_get(), TEMP_TRIP_ON_SENSOR_FAULT != 0);
        if (t != TEMP_TRIP_NONE)
            tx_trip(slow_adc_temp_trip_name(t), r.swr, r.fwd_w, r.rev_w, s_avg_mv[SLOW_ADC_REV], r.temp_c);
    }
#endif
}

static void publish_average(void)
{
    s_seq = s_seq + 1;                                   // odd: writing
    for (int i = 0; i < SLOW_ADC_NCH; i++) {
        slow_ch_t *c = &s_ch[i];
        if (c->n) s_avg_mv[i] = c->sum_mv / (float)c->n; // no good sample this window: keep the last one
        c->sum_mv = 0.0f;
        c->n = 0;
    }
    s_windows = s_windows + 1;
    s_seq = s_seq + 1;                                   // even: stable
    check_trips();
}

static void slow_adc_task(void *arg)
{
    (void)arg;
#if SLOW_ADC_READ_MODE == 1
    // One channel per wake, so 3 wakes make one "burst" and the per-channel rate is unchanged.
    const TickType_t period = pdMS_TO_TICKS(SLOW_ADC_BURST_PERIOD_MS / SLOW_ADC_NCH);
    const uint32_t steps_per_avg = (uint32_t)SLOW_ADC_NCH * SLOW_ADC_AVG_BURSTS;
#else
    const TickType_t period = pdMS_TO_TICKS(SLOW_ADC_BURST_PERIOD_MS);
    const uint32_t steps_per_avg = SLOW_ADC_AVG_BURSTS;
#endif
    TickType_t wake = xTaskGetTickCount();
    uint32_t steps = 0;
    int pending = -1;                                    // split mode: channel index with a conversion in flight
    (void)pending;                                       // unused in driver mode
    for (;;) {
        if (!s_reads_on) {
            for (int i = 0; i < SLOW_ADC_NCH; i++) { s_ch[i].sum_mv = 0.0f; s_ch[i].n = 0; }   // drop a partial window
            steps = 0;
            pending = -1;                                // forget any conversion in flight
            s_windows = 0;                               // report sends nothing until a fresh average exists
            s_paused_ack = true;
            while (!s_reads_on) ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            s_paused_ack = false;
            wake = xTaskGetTickCount();                  // restart the schedule, so the pause is not counted late
        }
        // xTaskDelayUntil returns pdFALSE if the deadline had already passed (the task was starved).
        if (xTaskDelayUntil(&wake, period) == pdFALSE) s_burst_late = s_burst_late + 1;
        if (!s_reads_on) continue;                       // paused while waiting: skip this step

#if SLOW_ADC_READ_MODE == 1
        int idx = (int)(steps % SLOW_ADC_NCH);
        int64_t t0 = esp_timer_get_time();
        if (pending >= 0) {                              // collect the conversion started on the previous wake
            slow_ch_t *pc = &s_ch[pending];
            int raw = 0;
            int r = split_collect(&raw);
            pc->reads++;
            if (r == 1) {
                float mv;
                raw_to_mv(pc, raw, &mv);
                pc->sum_mv += mv;
                pc->n++;
            } else {
                pc->errors++;
            }
            pending = -1;
        }
        slow_ch_t *c = &s_ch[idx];
        if (c->kind == CH_ADC2) {
            split_start(c->chan);
            pending = idx;
        } else {
            c->sum_mv += dummy_mv(idx, c);
            c->n++;
        }
        note_time((uint32_t)(esp_timer_get_time() - t0));
#else
        for (int i = 0; i < SLOW_ADC_NCH; i++) {
            slow_ch_t *c = &s_ch[i];
            float mv;
            if (c->kind == CH_ADC2) {
                if (!read_mv_driver(c, &mv)) continue;
            } else {
                mv = dummy_mv(i, c);
            }
            c->sum_mv += mv;
            c->n++;
        }
#endif
        if (++steps >= steps_per_avg) {
            steps = 0;
            publish_average();
        }
    }
}

static void setup_channel(slow_ch_t *c)
{
    if (c->gpio < 0) { c->kind = CH_DUMMY; return; }

    adc_unit_t unit;
    adc_channel_t chan;
    if (adc_oneshot_io_to_channel(c->gpio, &unit, &chan) != ESP_OK) {
        Serial.printf("[slowadc] %s: GPIO%d has no ADC function - using dummy value\r\n", c->name, c->gpio);
        c->kind = CH_REFUSED;
        return;
    }
    if (unit != ADC_UNIT_2) {
        Serial.printf("[slowadc] %s: GPIO%d is on ADC1, which streams the mic (continuous mode) - refused, "
                      "using dummy value\r\n", c->name, c->gpio);
        c->kind = CH_REFUSED;
        return;
    }
    if (!s_adc2) {
        adc_oneshot_unit_init_cfg_t ucfg = {};
        ucfg.unit_id = ADC_UNIT_2;
        ucfg.ulp_mode = ADC_ULP_MODE_DISABLE;
        esp_err_t e = adc_oneshot_new_unit(&ucfg, &s_adc2);
        if (e != ESP_OK) {
            Serial.printf("[slowadc] ADC2 oneshot unit init failed (err %d) - %s using dummy value\r\n", (int)e, c->name);
            s_adc2 = NULL;
            c->kind = CH_REFUSED;
            return;
        }
    }
    adc_oneshot_chan_cfg_t ccfg = {};
    ccfg.atten = ADC_ATTEN_DB_12;
    ccfg.bitwidth = ADC_BITWIDTH_DEFAULT;
    if (adc_oneshot_config_channel(s_adc2, chan, &ccfg) != ESP_OK) {
        Serial.printf("[slowadc] %s: ADC2 channel %d config failed - using dummy value\r\n", c->name, (int)chan);
        c->kind = CH_REFUSED;
        return;
    }
    c->chan = chan;
    c->kind = CH_ADC2;
    c->cali = NULL;
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t kcfg = {};
    kcfg.unit_id = ADC_UNIT_2;
    kcfg.chan = chan;
    kcfg.atten = ADC_ATTEN_DB_12;
    kcfg.bitwidth = ADC_BITWIDTH_DEFAULT;
    if (adc_cali_create_scheme_curve_fitting(&kcfg, &c->cali) != ESP_OK) c->cali = NULL;
#endif
    Serial.printf("[slowadc] %s: GPIO%d = ADC2 channel %d, calibration %s\r\n", c->name, c->gpio, (int)chan,
                  c->cali ? "curve fitting (mV)" : "none (linear 0-3100 mV estimate)");
}

void slow_adc_init(void)
{
    for (int i = 0; i < SLOW_ADC_NCH; i++) {
        setup_channel(&s_ch[i]);
        s_avg_mv[i] = 0.0f;
    }
    // One driver read per ADC2 channel at boot (both modes): proves the channel works and, for the split mode, leaves
    // the ADC2 RTC controller, attenuation, calibration code and arbiter configured for the register-level reads.
    for (int i = 0; i < SLOW_ADC_NCH; i++) {
        slow_ch_t *c = &s_ch[i];
        if (c->kind != CH_ADC2) continue;
        int raw = -1;
        esp_err_t e = adc_oneshot_read(s_adc2, c->chan, &raw);
        Serial.printf("[slowadc] %s: first driver read %s raw %d\r\n", c->name, e == ESP_OK ? "OK" : "FAILED", raw);
    }
    xTaskCreatePinnedToCore(slow_adc_task, "slow_adc", 3072, NULL, SLOW_ADC_TASK_PRIORITY, &s_task, SLOW_ADC_TASK_CORE);
    Serial.printf("[slowadc] running: FWD %s, REV %s, TEMP %s; burst every %d ms, average of %d bursts, task on "
                  "core %d prio %d, read mode %s, reads %s, report %s\r\n",
                  s_ch[0].kind == CH_ADC2 ? "ADC2" : "dummy", s_ch[1].kind == CH_ADC2 ? "ADC2" : "dummy",
                  s_ch[2].kind == CH_ADC2 ? "ADC2" : "dummy", SLOW_ADC_BURST_PERIOD_MS, SLOW_ADC_AVG_BURSTS,
                  SLOW_ADC_TASK_CORE, SLOW_ADC_TASK_PRIORITY, SLOW_ADC_READ_MODE == 1 ? "split" : "driver",
                  s_reads_on ? "on" : "paused", s_report_on ? "on" : "off");
}

static bool snapshot_mv(float mv[SLOW_ADC_NCH])
{
    if (s_windows == 0) return false;
    for (int tries = 0; tries < 4; tries++) {
        uint32_t a = s_seq;
        if (a & 1u) continue;
        for (int i = 0; i < SLOW_ADC_NCH; i++) mv[i] = s_avg_mv[i];
        if (s_seq == a) return true;
    }
    return true;   // the task publishes every 250 ms and takes microseconds to do it; a torn read is harmless here
}

static uint32_t carrier_hz(void)
{
#if AD9851_ATTACHED
    return carrier_output_get_freq_hz();
#else
    return 14200000u;
#endif
}

bool slow_adc_get_reading(slow_adc_reading_t *out)
{
    float mv[SLOW_ADC_NCH];
    if (!snapshot_mv(mv)) return false;
    slow_adc_convert(mv[0] / 1000.0f, mv[1] / 1000.0f, mv[2] / 1000.0f, carrier_hz(), &s_conv, out);
    return true;
}

int slow_adc_format_report(char *buf, size_t buf_len)
{
    if (!s_report_on || !s_reads_on) return 0;
    slow_adc_reading_t r;
    if (!slow_adc_get_reading(&r)) return 0;
    return slow_adc_format_fields(buf, buf_len, &r);
}

bool slow_adc_report_enabled(void) { return s_report_on; }
void slow_adc_set_report_enabled(bool on) { s_report_on = on; }
bool slow_adc_reads_enabled(void) { return s_reads_on; }
void slow_adc_set_reads_enabled(bool on)
{
    s_reads_on = on;
    if (on && s_task) xTaskNotifyGive(s_task);              // wake it if it is blocked in the pause
}

void slow_adc_format_status(char *buf, size_t buf_len)
{
    float mv[SLOW_ADC_NCH] = {0, 0, 0};
    bool have = snapshot_mv(mv);
    slow_adc_reading_t r = {0, 0, 0, 0};
    if (have) slow_adc_convert(mv[0] / 1000.0f, mv[1] / 1000.0f, mv[2] / 1000.0f, carrier_hz(), &s_conv, &r);

    size_t n = 0;
    for (int i = 0; i < SLOW_ADC_NCH && n < buf_len; i++) {
        slow_ch_t *c = &s_ch[i];
        const char *k = (c->kind == CH_ADC2) ? "ADC2" : (c->kind == CH_REFUSED) ? "refused" : "dummy";
        int w = snprintf(buf + n, buf_len - n, "->   %-4s %-7s GPIO%-3d %6.1f mV  reads %lu err %lu raw %d\r\n",
                         c->name, k, c->gpio, mv[i], (unsigned long)c->reads, (unsigned long)c->errors, c->last_raw);
        if (w < 0) break;
        n += (size_t)w;
    }
    if (n < buf_len) {
        uint32_t cnt = s_read_count;
        int w = snprintf(buf + n, buf_len - n,
                 "->   = %.1f W fwd, %.2f W rev, SWR %.1f, %.2f degC; %s us max %lu mean %.1f (n %lu); late %lu;"
                 " reads %s; report %s; core %d\r\n",
                 r.fwd_w, r.rev_w, r.swr, r.temp_c, SLOW_ADC_READ_MODE == 1 ? "split step" : "driver read",
                 (unsigned long)s_read_max_us, cnt ? (double)s_read_sum_us / cnt : 0.0, (unsigned long)cnt,
                 (unsigned long)s_burst_late,
                 s_reads_on ? (s_paused_ack ? "resuming" : "on") : (s_paused_ack ? "paused" : "pausing"),
                 s_report_on ? "on" : "off", SLOW_ADC_TASK_CORE);
        if (w > 0) n += (size_t)w;
    }
    // DSP health since the previous {K}: these rose by exactly one per driver-mode read on 2026-11-04.
    // late ticks are cleared by 'r' (diagnostics_reset), stale commits are not; a counter that went down is
    // treated as reset, so its whole value counts as the delta.
    static uint32_t s_prev_late = 0, s_prev_stale = 0;
    static int64_t s_prev_us = 0;
    uint32_t late = diagnostics_get_late_tick_count(), stale = diagnostics_get_stale_commit_count();
    int64_t now_us = esp_timer_get_time();
    uint32_t dl = (late >= s_prev_late) ? late - s_prev_late : late;
    uint32_t ds = (stale >= s_prev_stale) ? stale - s_prev_stale : stale;
    double secs = s_prev_us ? (double)(now_us - s_prev_us) / 1e6 : 0.0;
    if (n < buf_len) {
        if (secs > 0.0)
            snprintf(buf + n, buf_len - n, "->   dsp since last K (%.1f s): late ticks +%lu (%.2f/s), stale commits +%lu (%.2f/s); trips %lu, PA temp limit %d degC\r\n",
                     secs, (unsigned long)dl, dl / secs, (unsigned long)ds, ds / secs, (unsigned long)tx_trip_count(), tx_temp_max_get());
        else
            snprintf(buf + n, buf_len - n, "->   dsp: late ticks %lu, stale commits %lu (rates from the next K)\r\n",
                     (unsigned long)late, (unsigned long)stale);
    }
    s_prev_late = late; s_prev_stale = stale; s_prev_us = now_us;
    s_read_sum_us = 0;   // mean and count also restart, so each K describes only the interval since the last one
    s_read_count = 0;
    s_read_max_us = 0;   // so the next {K} shows the worst case since this one
}

#else   // SLOW_ADC_ENABLED == 0: everything compiles to stubs

void slow_adc_init(void) {}
int slow_adc_format_report(char *buf, size_t buf_len) { (void)buf; (void)buf_len; return 0; }
bool slow_adc_get_reading(slow_adc_reading_t *out) { (void)out; return false; }
bool slow_adc_report_enabled(void) { return false; }
void slow_adc_set_report_enabled(bool on) { (void)on; }
bool slow_adc_reads_enabled(void) { return false; }
void slow_adc_set_reads_enabled(bool on) { (void)on; }
void slow_adc_format_status(char *buf, size_t buf_len)
{
    snprintf(buf, buf_len, "->   slow ADC channels not built (SLOW_ADC_ENABLED 0 in config.h)\r\n");
}

#endif
