#pragma once

/**
 * slow_adc.h  (2026-11-04)  -- Phase 2 step 1: slow ADC channels, dummy-capable.
 *
 * Three slow inputs ported from the TXlink Nano: forward power, reflected power and PA temperature (NTC). They are
 * read in short bursts (all three back to back) by a low-priority task, averaged, converted with the Nano's curves
 * (slow_adc_conv.h) and appended as "{F..}{R..}{S..}{T..}" to the existing 250 ms instrumentation line, so no new
 * serial cadence is added. The line looks like
 *     {F1.8}{R0.08}{S1.5}{T25.00}            ({A} meter off)
 *     {A-12.3}{F1.8}{R0.08}{S1.5}{T25.00}    ({A} meter on, '#')
 *
 * Each channel is either
 *   DUMMY : its GPIO in ESP32_hardware.h is -1. No ADC access at all; a synthetic pin voltage is used (SLOW_ADC_DUMMY_*_MV,
 *           forward power slowly swings +/-10 % so the plugin display visibly updates). This is the default until
 *           the real pins are chosen, so the serial/plugin side can be tested with zero risk to the audio path.
 *   ADC2  : a GPIO that maps to ADC2 (GPIO11-20 on the S3; 11/12 are the AD9851, 19/20 USB). Read with the IDF
 *           adc_oneshot driver, 12 dB attenuation, curve-fitting calibration to mV when the eFuse data allows.
 *   ADC1 pins are REFUSED (the channel falls back to dummy and says so at boot): ADC1 is in continuous mode for the
 *   mic and one ADC unit cannot run oneshot and continuous at the same time.
 *
 * What a real read costs (from the ESP-IDF v5.5 source, adc_oneshot.c / adc_oneshot_hal.c, read 2026-11-04; NOT
 * measured on this board): adc_oneshot_read() does the whole setup + conversion + busy-wait for "done" inside
 * portENTER_CRITICAL(&rtc_spinlock), i.e. with interrupts masked on the calling core for the duration. On the S3 the
 * oneshot ADC2 path uses the RTC (SENS) controller and the ADC2 arbiter, which is separate from ADC1's digital/DMA
 * controller, so it LOOKS compatible with the mic stream, but that is reading, not proof. Two consequences:
 *   - on Core 1 a read would delay the 20 kHz gptimer ISR (AD9851 write) and the ADC conv_done ISR;
 *   - on Core 0 it would delay dsp_task's wake-up (watch overruns / late / stale_commits).
 * SLOW_ADC_TASK_CORE picks which; {K} reports the measured read time (max and mean) so the cost is known.
 * Bench 2026-11-04 (driver mode, core 0): ADC2 oneshot works alongside the ADC1 mic stream (19227 reads, 0 errors),
 * but late dsp ticks and stale AD9851 commits each rose by exactly one per read (24/s). The {K} "read time" of
 * ~257 us mean is wall time including pre-emption by dsp_task, not the masked time (worst dsp gap was 104 us).
 * Hence SLOW_ADC_READ_MODE 1 (split read, slow_adc.cpp): configure once with the driver, then per conversion only
 * select the pad + pulse start (two register writes), collect the result on the next wake - no busy-wait, no
 * critical section. {K} also shows late ticks / stale commits per second since the previous {K}.
 *
 * Not in this step: SWR/temperature trip, fan PWM, the Nano's TempTxOffset (RF pickup) correction.
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "slow_adc_conv.h"

enum { SLOW_ADC_FWD = 0, SLOW_ADC_REV = 1, SLOW_ADC_TEMP = 2, SLOW_ADC_NCH = 3 };

// Call once from setup(), after adc_capture_init() (the mic stream). Starts the task. No-op when SLOW_ADC_ENABLED 0.
void slow_adc_init(void);

// Appends "{F..}{R..}{S..}{T..}" for the latest completed average to buf. Returns characters written, 0 if
// disabled, not yet averaged, or the report is switched off ({K0}). Called from instrumentation_service().
int slow_adc_format_report(char *buf, size_t buf_len);

// Latest converted reading (for the later fan / SWR-trip code). Returns false until the first average exists.
bool slow_adc_get_reading(slow_adc_reading_t *out);

bool slow_adc_report_enabled(void);
void slow_adc_set_report_enabled(bool on);

// Run-time pause of the reads themselves ({K2} pause, {K3} run). Paused = the task is blocked (no wake-ups, no ADC
// access) and no slow fields are reported. NOTE for later: while paused there is no SWR/temperature data, so the
// future PA trip must treat "paused" as "no protection".
bool slow_adc_reads_enabled(void);
void slow_adc_set_reads_enabled(bool on);

// One-line-per-channel status for the {K} command, written into buf (no {X..} fields in it, so a plugin parser
// cannot mistake it for data). Also resets the max read time.
void slow_adc_format_status(char *buf, size_t buf_len);
