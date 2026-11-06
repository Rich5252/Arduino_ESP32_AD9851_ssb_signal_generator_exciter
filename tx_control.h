#pragma once

/**
 * tx_control.h  (2026-11-06)
 *
 * TX switching, first step of the TXlink TX sequencing port. For now only the "TXout" line: a digital output on
 * TXOUT_GPIO (ESP32_hardware.h, GPIO40) that switches the user's PA power switching, ACTIVE HIGH (high = TX on).
 *   {U} -> TXout on,  {D} -> TXout off   (brace commands, same letters as the TXlink Nano)
 * Boot state: OFF. The pin is driven low as the very first thing in setup(); an external pull-down keeps it off
 * during reset and boot before that (the pin floats until then).
 *
 * Not yet ported from the Nano: what else {U}/{D} did there (DDS power-down / power-up, the {f} message to the
 * modules on {U}, the TX power level), SWR/temperature trips, key/PTT inputs, sequencing delays, a TX timeout.
 */

#include <stdbool.h>

void tx_control_init(void);     // drive TXout low (off); call first in setup()
void tx_out_set(bool on);
bool tx_out_get(void);
