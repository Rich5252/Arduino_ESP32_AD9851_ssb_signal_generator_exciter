#pragma once

/**
 * module_link.h  (2026-11-05)
 *
 * Serial line to the user's filter / ATU modules, replacing what the TXlink Nano did by accident of hardware: on the
 * Nano the USB-serial chip shares the hardware UART, so everything printed to the PC also appeared on the TX pin
 * the modules listen to. On the ESP32-S3 `Serial` is native USB, so the modules get a dedicated output here instead:
 * UART0 TX on the board's TX header pin (MODULE_LINK_TX_GPIO, ESP32_hardware.h), 115200 8N1, TX only.
 *
 * Only the message the modules use is sent: "{f<Hz>}" + CR LF, same as the Nano's SendTunekHz(): the carrier
 * frequency rounded to the nearest kHz, written in Hz (e.g. 14201499 Hz -> "{f14201000}"). Sent
 *   - once at boot (after the carrier is set up), so the modules select the right filter/ATU setting at power-up,
 *   - on every accepted {F<Hz>} from the plugin (always, like the Nano, even if the kHz value did not change).
 * Not sent to USB: the plugin never needed it. (The Nano also sends the exact frequency at key-down and on {U};
 * those hooks come with the TX sequencing port - module_link_send_freq_exact() is ready for it.)
 *
 * The ESP32's own ROM/bootloader start-up text also comes out of this pin at reset (115200), before this code runs;
 * the modules only act on "{f...}" so it should be harmless. Writes never block: a line that does not fit in the
 * TX buffer is dropped and counted.
 */

#include <stdint.h>

void module_link_init(void);                       // call once in setup(); no-op when MODULE_LINK_ENABLED 0
void module_link_send_freq_khz(uint32_t carrier_hz);    // "{f<Hz rounded to kHz>}\r\n"
void module_link_send_freq_exact(uint32_t carrier_hz);  // "{f<Hz>}\r\n" (Nano key-down form, for later)
uint32_t module_link_sent_count(void);
uint32_t module_link_dropped_count(void);
