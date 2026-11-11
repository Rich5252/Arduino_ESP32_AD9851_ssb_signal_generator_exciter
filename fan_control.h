#pragma once

/**
 * fan_control.h  (2026-11-11)  - PA fan, ported from the TXlink Nano's CheckFanState().
 *
 * FanPin (FANPIN_GPIO, ESP32_hardware.h, GPIO42), active high, LEDC PWM on its own timer/channel (LEDC timer 1,
 * channel 1 - timer 0 / channel 0 are the RSET envelope PWM at 64 kHz), 8-bit duty in the Nano's units 0..255
 * (255 = fully on). PWM frequency FAN_PWM_HZ (config.h).
 *
 * Behaviour (Nano), evaluated every FAN_SERVICE_MS from loop():
 *   - TX active (sequencer not in RX): full (255).
 *   - after TX: low speed {R} (default 150) for the hold time {B} minutes (default 3), then off;
 *   - PA temperature above FAN_HIGH_TEMP_C (Nano FanHighTemp 38 degC) while the fan runs: full.
 * Deliberate differences from the Nano:
 *   - hot with no TX: the fan stays on (full) and the hold time restarts, instead of the Nano's on/off flapping
 *     every check once the hold had expired;
 *   - temperature sensor fault (reads -99): treated as hot;
 *   - start kick: from off to any lower speed the fan gets FAN_KICK_MS at full first (the Nano did 500 ms at
 *     boot only, "needed for proper start").
 * Boot: off (the pin is held low from the first line of setup(), see tx_control force_safe_outputs()).
 */

#include <stdint.h>
#include <stdbool.h>

void fan_control_init(void);
void fan_control_service(void);          // loop()

void fan_set_low_pwm(int pwm);           // {R} 0..255
int  fan_get_low_pwm(void);
void fan_set_hold_min(int minutes);      // {B} 0..120
int  fan_get_hold_min(void);
int  fan_get_duty(void);                 // current 0..255
