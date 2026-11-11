/**
 * fan_control.cpp - see fan_control.h.  (2026-11-11, uncompiled)
 */

#include "fan_control.h"
#include "config.h"
#include "tx_control.h"
#include "slow_adc.h"
#include <Arduino.h>
#include "driver/ledc.h"

#ifndef FAN_PWM_HZ
#define FAN_PWM_HZ          25000
#endif
#ifndef FAN_SERVICE_MS
#define FAN_SERVICE_MS      250
#endif
#ifndef FAN_HIGH_TEMP_C
#define FAN_HIGH_TEMP_C     38.0f
#endif
#ifndef FAN_KICK_MS
#define FAN_KICK_MS         500
#endif
#ifndef FAN_LOW_PWM_DEFAULT
#define FAN_LOW_PWM_DEFAULT 150
#endif
#ifndef FAN_HOLD_MIN_DEFAULT
#define FAN_HOLD_MIN_DEFAULT 3
#endif

#define FAN_LEDC_TIMER   LEDC_TIMER_1
#define FAN_LEDC_CH      LEDC_CHANNEL_1

static volatile int s_low_pwm = FAN_LOW_PWM_DEFAULT;
static volatile int s_hold_min = FAN_HOLD_MIN_DEFAULT;
static int      s_duty = 0;              // what is on the pin now, 0..255
static bool     s_ok = false;
static uint32_t s_hold_start_ms = 0;
static bool     s_hold_active = false;
static uint32_t s_kick_until_ms = 0;
static uint32_t s_last_ms = 0;
static bool     s_was_tx = false;
static uint32_t s_tx_starts_seen = 0;

static void write_duty(int d)
{
    if (d < 0) d = 0;
    if (d > 255) d = 255;
    if (d == s_duty || !s_ok) return;
    uint32_t hw = (d >= 255) ? 256u : (uint32_t)d;     // 8-bit LEDC: 256 = 100 % on (IDF allows 0..2^bits)
    ledc_set_duty(LEDC_LOW_SPEED_MODE, FAN_LEDC_CH, hw);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, FAN_LEDC_CH);
    s_duty = d;
}

void fan_control_init(void)
{
#if FANPIN_GPIO >= 0
    ledc_timer_config_t t = {};
    t.speed_mode = LEDC_LOW_SPEED_MODE;
    t.duty_resolution = LEDC_TIMER_8_BIT;
    t.timer_num = FAN_LEDC_TIMER;
    t.freq_hz = FAN_PWM_HZ;
    t.clk_cfg = LEDC_AUTO_CLK;
    if (ledc_timer_config(&t) != ESP_OK) { Serial.printf("[fan] LEDC timer config failed - fan stays off\r\n"); return; }
    ledc_channel_config_t c = {};
    c.gpio_num = FANPIN_GPIO;
    c.speed_mode = LEDC_LOW_SPEED_MODE;
    c.channel = FAN_LEDC_CH;
    c.timer_sel = FAN_LEDC_TIMER;
    c.duty = 0;
    c.hpoint = 0;
    if (ledc_channel_config(&c) != ESP_OK) { Serial.printf("[fan] LEDC channel config failed - fan stays off\r\n"); return; }
    s_ok = true;
    s_duty = 0;
#endif
}

void fan_control_service(void)
{
    uint32_t now = millis();
    if ((uint32_t)(now - s_last_ms) < FAN_SERVICE_MS) return;
    s_last_ms = now;

    uint32_t starts = tx_start_count();
    bool tx = (tx_seq_state() != TX_SEQ_RX) || (starts != s_tx_starts_seen);   // also a TX shorter than 250 ms
    s_tx_starts_seen = starts;
    if (s_was_tx && !tx) { s_hold_start_ms = now; s_hold_active = true; }   // TX just ended: start the hold
    s_was_tx = tx;

    slow_adc_reading_t r;
    bool hot = false;
    if (slow_adc_get_reading(&r)) hot = (r.temp_c > FAN_HIGH_TEMP_C) || (r.temp_c <= -98.0f);  // -99 = sensor fault
    if (hot && !tx) { s_hold_start_ms = now; s_hold_active = true; }       // keep running while hot

    int target;
    if (tx || hot) target = 255;
    else if (s_hold_active && (uint32_t)(now - s_hold_start_ms) < (uint32_t)s_hold_min * 60000u) target = s_low_pwm;
    else { target = 0; s_hold_active = false; }

    if (s_duty == 0 && target > 0 && target < 255) s_kick_until_ms = now + FAN_KICK_MS;   // start kick
    if (target > 0 && (int32_t)(s_kick_until_ms - now) > 0) target = 255;
    write_duty(target);
}

void fan_set_low_pwm(int pwm)   { if (pwm < 0) pwm = 0; if (pwm > 255) pwm = 255; s_low_pwm = pwm; }
int  fan_get_low_pwm(void)      { return s_low_pwm; }
void fan_set_hold_min(int m)    { if (m < 0) m = 0; if (m > 120) m = 120; s_hold_min = m; }
int  fan_get_hold_min(void)     { return s_hold_min; }
int  fan_get_duty(void)         { return s_duty; }
