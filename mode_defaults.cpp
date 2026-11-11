/**
 * mode_defaults.cpp - see mode_defaults.h.
 */

#include "mode_defaults.h"
#include "config.h"
#include "tx_control.h"
#include "serial_commands.h"
#include "test_signals.h"
#include "envelope_output.h"
#include "dsp_state.h"
#include "ssb_dsp.h"

#ifndef MODE_DEFAULTS_ENABLED
#define MODE_DEFAULTS_ENABLED 0
#endif

static int preset_for_mode(int mode)
{
    switch (mode) {
    case TX_MODE_USB:
    case TX_MODE_LSB:  return MODE_PRESET_VOICE;
    case TX_MODE_TUNE: return MODE_PRESET_TUNE;
    case TX_MODE_CW:
    case TX_MODE_RTTY:
    case TX_MODE_FT2:
    case TX_MODE_FT4:
    case TX_MODE_FT8:
    case TX_MODE_JTTY: return MODE_PRESET_CARRIER;
    default:           return -2;      // unknown mode: nothing at all
    }
}

int mode_defaults_apply(int mode)
{
#if MODE_DEFAULTS_ENABLED
    int preset = preset_for_mode(mode);
    if (preset == -2) return -1;

    envelope_output_duty_override_set_enabled(false);

    if (preset >= 0 && preset <= 9) settings_apply_preset(preset);
    else preset = -1;

    if (mode == TX_MODE_TUNE) {
        test_signals_twotone_restore_tone_defaults();
        test_signals_set_twotone_phase_gen((twotone_phase_gen_t)MODE_TUNE_PHASE_GEN);
        // Same restart as the 'O' key: both are flag requests consumed in dsp_task.
        ssb_dsp_request_reset(dsp_state_get_ssb());
        test_signals_twotone_request_reset();
    }
    return preset;
#else
    (void)mode;
    return -1;
#endif
}
