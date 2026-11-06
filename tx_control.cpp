/**
 * tx_control.cpp - see tx_control.h.  (2026-11-06, uncompiled)
 */

#include "tx_control.h"
#include "config.h"
#include "envelope_floor.h"
#include <Arduino.h>
#include <stdlib.h>

#ifndef TX_LEVEL_SLEW_MS
#define TX_LEVEL_SLEW_MS        5.0f     // ms for a full-scale (0 <-> 100 %) level change
#endif
#ifndef TX_LEVEL_FLOOR_FADE
#define TX_LEVEL_FLOOR_FADE     0.10f    // below this scale the envelope floor fades out (L0 = no output)
#endif

static volatile bool  s_requested = false;
static volatile bool  s_tx_on = false;
static volatile int   s_level = 0;
static volatile bool  s_local_pa = true;
static volatile float s_target = 0.0f;   // written by the command side (Core 1), read by dsp_task (Core 0)
static float          s_scale = 0.0f;    // owned by dsp_task

static void apply_txout(void)
{
    bool on = s_requested && s_local_pa;
    digitalWrite(TXOUT_GPIO, on ? HIGH : LOW);   // active high: HIGH = TX on
    s_tx_on = on;
}

void tx_control_init(void)
{
    digitalWrite(TXOUT_GPIO, LOW);     // set the output latch low first, so the pin never glitches high
    pinMode(TXOUT_GPIO, OUTPUT);
    s_requested = false;
    s_local_pa = true;
    s_level = 0;
    s_target = 0.0f;
    s_scale = 0.0f;
    apply_txout();
}

void tx_request_set(bool on) { s_requested = on; apply_txout(); }
bool tx_request_get(void)    { return s_requested; }
bool tx_out_get(void)        { return s_tx_on; }

void tx_level_set(int level)
{
    bool local = (level >= 0);         // Nano: TXlevel < 0 -> LocalPAactive = false
    int mag = abs(level);
    if (mag > 100) mag = 100;
    s_level = local ? mag : -mag;
    s_local_pa = local;
    s_target = (float)mag / 100.0f;
    apply_txout();                     // a negative L drops TXout at once, a positive one restores it if requested
}

int  tx_level_get(void)       { return s_level; }
bool tx_local_pa_active(void) { return s_local_pa; }

float IRAM_ATTR tx_level_apply(float envelope)
{
    const float step = 1.0f / (TX_LEVEL_SLEW_MS * 0.001f * (float)SAMPLE_RATE_HZ);
    float t = s_target;
    if (s_scale < t)      { s_scale += step; if (s_scale > t) s_scale = t; }
    else if (s_scale > t) { s_scale -= step; if (s_scale < t) s_scale = t; }
    float s = s_scale;
    if (s >= 1.0f) return envelope;                       // L100: bit-identical to the previous behaviour
    float fade = s / TX_LEVEL_FLOOR_FADE;
    if (fade > 1.0f) fade = 1.0f;
    float f = envelope_floor_get() * fade;
    return f + (envelope - f) * s;
}

float tx_level_current_scale(void) { return s_scale; }
