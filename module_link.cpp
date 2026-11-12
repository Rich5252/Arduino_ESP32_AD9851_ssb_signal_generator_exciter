/**
 * module_link.cpp - see module_link.h.  (2026-11-05, uncompiled)
 */

#include "module_link.h"
#include "config.h"
#include <Arduino.h>
#include <stdio.h>

#if MODULE_LINK_ENABLED

// Serial0 is UART0 only while `Serial` is the native USB port (Tools > USB CDC On Boot: Enabled). With it disabled,
// `Serial` itself would be UART0 on these same pins and the plugin traffic would go to the modules.
#if defined(ARDUINO_USB_CDC_ON_BOOT) && !ARDUINO_USB_CDC_ON_BOOT
#error "MODULE_LINK_ENABLED needs USB CDC On Boot = Enabled (Serial on native USB, Serial0 free for the modules)"
#endif

static uint32_t s_sent = 0, s_dropped = 0;
static bool s_up = false;

void module_link_init(void)
{
    Serial0.setTxBufferSize(256);   // must be before begin(); one "{f..}" line is 13 bytes
    // RX pin given explicitly so the driver cannot pick some other default; nothing is read from it.
#ifndef MODULE_LINK_TX_INVERT
#define MODULE_LINK_TX_INVERT 0
#endif
    // Last argument inverts the UART lines (TX and RX; RX is unused) for the inverting transistor driver, see config.h.
    Serial0.begin(MODULE_LINK_BAUD, SERIAL_8N1, MODULE_LINK_RX_GPIO, MODULE_LINK_TX_GPIO, MODULE_LINK_TX_INVERT != 0);
    s_up = true;
}

static void send_line(const char *line, int n)
{
    if (!s_up || n <= 0) return;
    if (Serial0.availableForWrite() < n) { s_dropped++; return; }   // never block loop()
    Serial0.write((const uint8_t *)line, (size_t)n);
    s_sent++;
}

void module_link_send_freq_exact(uint32_t carrier_hz)
{
    char buf[24];
    int n = snprintf(buf, sizeof buf, "{f%lu}\r\n", (unsigned long)carrier_hz);
    send_line(buf, n);
}

void module_link_send_freq_khz(uint32_t carrier_hz)
{
    uint32_t khz = (carrier_hz + 500u) / 1000u;    // Nano: thiskHz = (CurFreq + 500) / 1000, then * 1000
    module_link_send_freq_exact(khz * 1000u);
}

void module_link_send_atu(const char *body)
{
    if (!body) return;
    char buf[16];
    int n = snprintf(buf, sizeof buf, "{%s}", body);     // no CR/LF: the Nano's echo had none
    send_line(buf, n);
}

uint32_t module_link_sent_count(void) { return s_sent; }
uint32_t module_link_dropped_count(void) { return s_dropped; }

#else

void module_link_init(void) {}
void module_link_send_freq_khz(uint32_t carrier_hz) { (void)carrier_hz; }
void module_link_send_freq_exact(uint32_t carrier_hz) { (void)carrier_hz; }
void module_link_send_atu(const char *body) { (void)body; }
uint32_t module_link_sent_count(void) { return 0; }
uint32_t module_link_dropped_count(void) { return 0; }

#endif
