#pragma once

/**
 * ESP32_hardware.h  (2026-11-05; pin moves by the user 2026-11-10)
 *
 * One place for every GPIO this firmware uses, the pins reserved for planned work, and the pins to keep clear of
 * (with the reason, briefly). Pin NUMBERS live here only; the code that drives a pin, and the long history of why
 * it moved, stays with that code (comments in config.h, adc_capture.cpp, envelope_output.h, carrier_output.h).
 * Included by config.h, so every file that includes config.h sees these. Plain #defines only (C and C++).
 *
 * Board: ESP32-S3 Super Mini (ssb_mic_test_commands.md, 2026-09-02): GPIO1-13 on the main header, GPIO39-48 on small
 * solder pads, TX/RX (GPIO43/44) on the header, 14-16 reachable for the slow ADC. Many S3 Super Mini boards carry a
 * WS2812 RGB LED on GPIO48 - check this one before using 48 as the TX-on output (the LED would load and react to it).
 *
 * ---- In use ---------------------------------------------------------------------------------------------------
 *   GPIO  2   RSET envelope PWM out (LEDC timer 0, channel 0)             envelope_output.cpp
 *   GPIO  4   timing debug: dsp_task busy window (TIMING_DEBUG_ENABLED)   ssb_mic_test.ino
 *   GPIO  5   timing debug: gptimer alarm ISR (TIMING_DEBUG_ENABLED)      ssb_mic_test.ino
 *   GPIO  6   mic input, ADC1 channel 5, adc_continuous                   adc_capture.cpp
 *   GPIO  9   AD9851 RESET                                                AD9851.c / carrier_output.cpp
 *   GPIO 10   AD9851 DATA
 *   GPIO 11   AD9851 FQ_UD
 *   GPIO 12   AD9851 W_CLK
 *   GPIO  1   KeyIn (Nano name), active low, pull-up - reserved, no code yet
 *   GPIO  7   MicPTTin, active low, pull-up - reserved, no code yet (next to the mic input GPIO6)
 *   GPIO 14   slow ADC PA temperature (NTC, 47k pull-up to 3V3), ADC2 channel 3   slow_adc.cpp
 *   GPIO 15   slow ADC FWD power, ADC2 channel 4
 *   GPIO 16   slow ADC REV power, ADC2 channel 5
 *   GPIO 19   USB D-  (USB-CDC serial to the PC / SDR plugin)             not configurable
 *   GPIO 20   USB D+
 *   GPIO 40   TXout (Nano name): antenna relay + local 2 W PA power, ACTIVE HIGH (high = TX on), {U}/{D}
 *             (tx_control.cpp). Driven low first thing in setup(); fit a ~10k pull-down so it stays off at reset/boot.
 *   GPIO 39   chirp-mode square-wave reference; also the (off by default) ADC-ISR and command-window debug marker
 *   GPIO 43   header TX: UART0 TX to the filter/ATU modules, "{f<Hz>}" at 115200 (module_link.cpp). Also carries the
 *             chip's ROM/bootloader text at reset. Driver: one 2N7000/BSS138 (gate from GPIO43, ~100k gate-GND,
 *             drain to the module line pulled up by the Nanos' ~1k) with MODULE_LINK_TX_INVERT 1 in config.h.
 *   GPIO 44   header RX: UART0 RX, claimed by the same UART but nothing is read (modules never talk back)
 *
 * ---- Reserved for the TXlink port (no code uses them yet) - TXlink4_Nano names where the Nano had the pin -------
 *   GPIO  1   KeyIn                  input, active low, pull-up (Nano D8)            (assigned 2026-11-10)
 *   GPIO  7   MicPTTin               input, active low, pull-up (new, no Nano equivalent)
 *   GPIO 41   ExtPAout               external (remote) PA PTT, active high (Nano D4)
 *   GPIO 42   FanPin                 PA fan out, active high (Nano D10, on/off there; PWM here needs its own LEDC
 *                                    timer - timer 0 belongs to the RSET PWM)
 *   GPIO 47, 48  free (48 was "local PA TX-on", which is TXout - 2026-11-06)
 *   Inputs on 1/7: external 4.7-10k pull-up to 3V3 (never 5 V) and ~1k series + 10-100 nF to GND at the pin.
 *   Fit ~10k pull-downs on 40/41/42 so TXout, the external PA and the fan are off during reset/boot.
 *   Nano pins with no pin here: KeyOut (D7, RF on via the DDS level shaper) and TXlevelPin (D5, PWM DDS level) -
 *   on the ESP32 RF on/off and level are done in firmware (DDS + RSET envelope PWM on GPIO2).
 *   40-42 are the JTAG MTDO/MTDI/MTMS pins: free while pad JTAG is not used (USB-JTAG is on 19/20).
 *   47/48 run at 1.8 V on "V" modules (1.8 V flash); see the board note above about an RGB LED on GPIO48.
 *
 * ---- Avoid ----------------------------------------------------------------------------------------------------
 *   GPIO  0      strapping (boot mode, BOOT button).
 *   GPIO  3      strapping (JTAG source). ALSO: toggling it as a debug output stopped the mic ADC producing any
 *                conversions (2026-09, mechanism unknown).
 *   GPIO 45      strapping: VDD_SPI voltage select - high at reset can select 1.8 V flash supply.
 *   GPIO 46      strapping: ROM log / boot mode with GPIO0. Outputs with pull-downs only, never pulled-up inputs.
 *   GPIO 13      5 kHz pulses from the mic ADC driver's activity were seen on this pad (2026-09-02).
 *   GPIO 1-20    ADC pins in general: keep fast-switching signals off them where there is a choice (see 3 and 13).
 *                4/5 (debug) were checked as harmless on the bench; 9-12 (AD9851) work in practice.
 *   ADC1 pins    (1-10): ADC1 runs in continuous mode for the mic, so no oneshot reads on ADC1.
 *   GPIO 26-32   SPI flash / PSRAM inside the module.
 *   GPIO 33-37   octal PSRAM on R8/R16 modules (free on modules without octal PSRAM - not checked for this one).
 *   (GPIO 43/44 are in use as the module link above; 44 must not be driven by a 5 V signal.)
 *
 * ---- Spare (not used, not on the avoid list) -----------------------------------------------------------------
 *   8 (ADC1 - analogue or slow signals only), 17, 18 (ADC2), 21, 38, 47, 48. Also 4/5/39 if the debug pins go.
 *   Check the board actually brings each one out.
 *
 * ---- Other on-chip resources claimed ---------------------------------------------------------------------------
 *   LEDC timer 0 / channel 0: RSET PWM.  ADC1: mic (continuous).  ADC2: slow channels (driver set-up once at boot,
 *   then register-level start/collect, see slow_adc.cpp).  gptimer: 20 kHz sample timer (ISR does the AD9851 write).
 */

// ---- Mic ADC (ADC1, continuous mode) ----
#define MIC_ADC_GPIO              6
#define MIC_ADC1_CHANNEL          ADC_CHANNEL_5     // must match MIC_ADC_GPIO (GPIO6 = ADC1 ch5 on the S3)

// ---- AD9851 DDS, bit-banged serial load ----
#define AD9851_PIN_RESET          9
#define AD9851_PIN_DATA           10
#define AD9851_PIN_FQUD           11
#define AD9851_PIN_WCLK           12
// The AD9851 control lines go through INVERTING BS170 stages (AD9851.c, AD9851_INVERTING_LEVEL_SHIFT): GPIO9 LOW =
// AD9851 RESET asserted (DDS held at 0 Hz, no RF). Used by tx_control.cpp to hold the DDS off early in setup() and in
// the reboot shutdown handler. A ~100k gate-to-GND pull-down on that BS170 does the same while the ESP32 is in
// reset / being flashed (no firmware running).
#define AD9851_RESET_ASSERT_LEVEL 0

// ---- Envelope (RSET) PWM out ----
#define RSET_MOD_LEDC_GPIO        2

// ---- Slow ADC inputs (ADC2) - only ADC2 pins are accepted, see slow_adc.h ----
#define SLOW_ADC_FWD_GPIO         15     // ADC2 ch4
#define SLOW_ADC_REV_GPIO         16     // ADC2 ch5
#define SLOW_ADC_TEMP_GPIO        14     // ADC2 ch3

// ---- Timing debug pins (TIMING_DEBUG_ENABLED and friends in config.h) ----
#define TIMING_DEBUG_GPIO         4      // dsp_task busy window
#define TIMING_DEBUG_GPIO_ISR     5      // gptimer alarm ISR
#define TIMING_DEBUG_GPIO_ADC     39     // ADC ISR marker / command window / chirp reference (one at a time)

// ---- TX switching (tx_control.cpp) ----
#define TXOUT_GPIO                40     // TXout, active high (high = TX on)

// ---- TXlink port, reserved (not used by any code yet); Nano names ----
#define KEYIN_GPIO                1      // Nano KeyIn, active low
#define MIC_PTT_IN_GPIO           7      // new, active low
#define FANPIN_GPIO               42     // Nano FanPin, active high
#define EXTPAOUT_GPIO             41     // Nano ExtPAout, external PA PTT, active high

// ---- Module link: UART0 to the filter/ATU modules (module_link.cpp), board header TX/RX ----
#define MODULE_LINK_TX_GPIO       43
#define MODULE_LINK_RX_GPIO       44     // claimed by the UART, unused

// ---- MCP4725 I2C DAC: NOT FITTED. Only used when MCP4725_I2C_ENABLED is 1 (config.h); config.h refuses to build if
// these pins clash with TXout or ExtPAout. ----
#define MCP4725_SDA_GPIO          47
#define MCP4725_SCL_GPIO          48

// ---- Clash checks for the pins above ----
#if TXOUT_GPIO == MIC_PTT_IN_GPIO || TXOUT_GPIO == FANPIN_GPIO || TXOUT_GPIO == EXTPAOUT_GPIO || \
    TXOUT_GPIO == KEYIN_GPIO || TXOUT_GPIO == MODULE_LINK_TX_GPIO
#error "TXOUT_GPIO clashes with another pin in ESP32_hardware.h"
#endif
#if KEYIN_GPIO >= 0 && (KEYIN_GPIO == MIC_PTT_IN_GPIO || KEYIN_GPIO == FANPIN_GPIO || KEYIN_GPIO == EXTPAOUT_GPIO || \
    KEYIN_GPIO == MODULE_LINK_TX_GPIO || KEYIN_GPIO == MODULE_LINK_RX_GPIO)
#error "KEYIN_GPIO clashes with another pin in ESP32_hardware.h"
#endif
