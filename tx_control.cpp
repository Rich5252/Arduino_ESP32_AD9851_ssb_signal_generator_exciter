/**
 * tx_control.cpp - see tx_control.h.  (2026-11-06, uncompiled)
 */

#include "tx_control.h"
#include "config.h"
#include <Arduino.h>

static volatile bool s_tx_on = false;

void tx_control_init(void)
{
    digitalWrite(TXOUT_GPIO, LOW);     // set the output latch low first, so the pin never glitches high
    pinMode(TXOUT_GPIO, OUTPUT);
    digitalWrite(TXOUT_GPIO, LOW);
    s_tx_on = false;
}

void tx_out_set(bool on)
{
    digitalWrite(TXOUT_GPIO, on ? HIGH : LOW);   // active high: HIGH = TX on
    s_tx_on = on;
}

bool tx_out_get(void) { return s_tx_on; }
