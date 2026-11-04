/**
 * slow_adc.cpp - see slow_adc.h.  (2026-11-04, not yet compiled for the ESP32 or run on hardware)
 */

#include "slow_adc.h"
#include "config.h"
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

static bool read_mv(slow_ch_t *c, float *mv)
{
    int raw = -1;
    int64_t t0 = esp_timer_get_time();
    esp_err_t e = adc_oneshot_read(s_adc2, c->chan, &raw);
    uint32_t dt = (uint32_t)(esp_timer_get_time() - t0);
    if (dt > s_read_max_us) s_read_max_us = dt;
    s_read_sum_us = s_read_sum_us + dt;
    s_read_count = s_read_count + 1;
    c->reads++;
    if (e != ESP_OK || raw < 0) { c->errors++; return false; }   // ESP_ERR_TIMEOUT = busy / arbiter said invalid
    c->last_raw = raw;
    int v = 0;
    if (c->cali && adc_cali_raw_to_voltage(c->cali, raw, &v) == ESP_OK) *mv = (float)v;
    else *mv = raw * 3100.0f / 4095.0f;
    return true;
}

static void slow_adc_task(void *arg)
{
    (void)arg;
    const TickType_t period = pdMS_TO_TICKS(SLOW_ADC_BURST_PERIOD_MS);
    TickType_t wake = xTaskGetTickCount();
    uint32_t bursts = 0;
    for (;;) {
        if (!s_reads_on) {
            for (int i = 0; i < SLOW_ADC_NCH; i++) { s_ch[i].sum_mv = 0.0f; s_ch[i].n = 0; }   // drop a partial window
            bursts = 0;
            s_windows = 0;                                   // report sends nothing until a fresh average exists
            s_paused_ack = true;
            while (!s_reads_on) ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            s_paused_ack = false;
            wake = xTaskGetTickCount();                      // restart the schedule, so the pause is not counted late
        }
        // vTaskDelayUntil returns pdFALSE if the deadline had already passed (the task was starved).
        if (xTaskDelayUntil(&wake, period) == pdFALSE) s_burst_late = s_burst_late + 1;
        if (!s_reads_on) continue;                           // paused while waiting: skip this burst

        for (int i = 0; i < SLOW_ADC_NCH; i++) {
            slow_ch_t *c = &s_ch[i];
            float mv;
            if (c->kind == CH_ADC2) {
                if (!read_mv(c, &mv)) continue;
            } else {
                mv = dummy_mv(i, c);
            }
            c->sum_mv += mv;
            c->n++;
        }

        if (++bursts >= SLOW_ADC_AVG_BURSTS) {
            bursts = 0;
            s_seq = s_seq + 1;                                   // odd: writing
            for (int i = 0; i < SLOW_ADC_NCH; i++) {
                slow_ch_t *c = &s_ch[i];
                if (c->n) s_avg_mv[i] = c->sum_mv / (float)c->n; // no good sample this window: keep the last one
                c->sum_mv = 0.0f;
                c->n = 0;
            }
            s_windows = s_windows + 1;
            s_seq = s_seq + 1;                                   // even: stable
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
    xTaskCreatePinnedToCore(slow_adc_task, "slow_adc", 3072, NULL, SLOW_ADC_TASK_PRIORITY, &s_task, SLOW_ADC_TASK_CORE);
    Serial.printf("[slowadc] running: FWD %s, REV %s, TEMP %s; burst every %d ms, average of %d bursts, task on "
                  "core %d prio %d, reads %s, report %s\r\n",
                  s_ch[0].kind == CH_ADC2 ? "ADC2" : "dummy", s_ch[1].kind == CH_ADC2 ? "ADC2" : "dummy",
                  s_ch[2].kind == CH_ADC2 ? "ADC2" : "dummy", SLOW_ADC_BURST_PERIOD_MS, SLOW_ADC_AVG_BURSTS,
                  SLOW_ADC_TASK_CORE, SLOW_ADC_TASK_PRIORITY, s_reads_on ? "on" : "paused", s_report_on ? "on" : "off");
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
        snprintf(buf + n, buf_len - n,
                 "->   = %.1f W fwd, %.2f W rev, SWR %.1f, %.2f degC; read us max %lu mean %.1f (n %lu); late %lu;"
                 " reads %s; report %s; core %d\r\n",
                 r.fwd_w, r.rev_w, r.swr, r.temp_c, (unsigned long)s_read_max_us,
                 cnt ? (double)s_read_sum_us / cnt : 0.0, (unsigned long)cnt, (unsigned long)s_burst_late,
                 s_reads_on ? (s_paused_ack ? "resuming" : "on") : (s_paused_ack ? "paused" : "pausing"),
                 s_report_on ? "on" : "off", SLOW_ADC_TASK_CORE);
    }
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
