# Handoff for the next thread (written 2026-11-04)

Purpose: everything a fresh session needs that is NOT already in `moving_forward_notes.md` / `null_bias_investigation.md`.
Those two files are the project's memory; this file says how to use them and what state things are in.

## 1. First, get the code into the repo (important)
The previous thread's sandbox is gone when it ends, and its work exists only as **8 local commits that were never pushed**
(base: user's GitHub repo HEAD `df4bbee`, "Change ADC audio level reading to reject DC level"). The same files were copied to
`C:\Users\office2\Downloads\Claude outputs` (Windows). Before the new thread does anything:
1. Copy those files over the repo working copy (keep `host_tests\` as a subfolder, never in the sketch root).
2. Build in the Arduino IDE to confirm, then commit and push from your machine.
3. The new thread then clones the repo: `https://github.com/Rich5252/Arduino_ESP32_AD9851_ssb_signal_generator_exciter`
   (project reference for the Nano: `https://github.com/Rich5252/TXlink4_Nano_Latest`).
If the new thread clones the repo before step 3 it will be MISSING: the `{F}`/`{C}` brace commands, the ppb clock
calibration, the slew-key change and the newest notes entries.

Files changed/added in that work: `brace_input.h/.cpp` (new), `carrier_output.h/.cpp`, `AD9851.c/.h`, `diagnostics.cpp`,
`serial_commands.cpp`, `ssb_mic_test.ino`, `envelope_interp.h`, `settings.h`, `ssb_dsp.h`, `ssb_dsp.c`, `test_signals.h`,
`ssb_mic_test_commands.md`, `host_tests/test_brace_input.cpp`, `host_tests/test_clock_cal.cpp`, `host_tests/README.md`, both notes files.

## 2. State of the firmware (as of the last commit)
- **Nothing in this phase has been compiled for the ESP32 by Claude.** The user builds in the Arduino IDE and has confirmed
  that `{F<Hz>}` and the ppb calibration `{C<ppb>}` work on the bench. Host tests exist for the parser and the calibration
  maths (`host_tests/`, 30 + 92 checks). The sandbox has no ESP-IDF/Arduino toolchain.
- `{F<Hz>}` sets the carrier (1-60 MHz, not saved over reboot). `{C<ppb>}` is the AD9851 clock calibration, same convention as
  the Nano library (`calibrated_clock = clk*(1e9-ppb)/1e9`; positive = real clock slow = output goes up; trim:
  `new = old + (f_set - f_meas)/f_set*1e9`). Boot default `CARRIER_CAL_PPB` is 11408 (= the old fixed +162 Hz at 14.2 MHz).
  The user said 160 Hz is "very close, within my measurement accuracy" (= about 11268 ppb) and has NOT yet chosen a new default.
  Ask the user for the final value.
- Single-key slew keys moved from `{`/`}` to `_` (tighten) and `=` (loosen); braces are reserved for brace commands.
- The carrier canary now compares against a complement shadow copy (weaker than the old compile-time compare).
- `{A}` meter (`#` cycles off / RF / ADC): format is sign + 2 digits + 1 decimal, e.g. `{A-06.0}`, `+` is possible when over range.
  The user later edited the format themselves (their commit message mentions `{A-000.0}`): re-read `instrumentation.cpp`
  before touching it. The plugin must accept a leading `+`.
- The predistort LUT, group-delay equaliser, relative delay etc. were all measured at 14.2 MHz; not re-tuned per band.

## 3. Phase 2 task (what the new thread is for)
Integrate Nano TXlink features: three slow ADC channels (forward power, reflected power, PA temperature; >= 2 samples/s
each, SWR is the PA safety shutdown) and a fan PWM (full while keyed, temperature-controlled when key-up). The feasibility
write-up is the last entry in both notes files ("2026-11-04 — Phase 2 feasibility"). Summary of the plan: try ADC2
`adc_oneshot` from a low-priority task first (test for ISR jitter, `[adc]` starve counters, mic noise floor and spurs), fall
back to leaving these on the Nano, treat adding them to the ADC1 continuous pattern as last resort (about double the total
conversion rate; the 80 kHz region was abandoned earlier). Open facts to settle first: which exact pins the user's board
exposes, whether ADC2 one-shot works while ADC1 streams (docs do not say), ESP32-S3 input range (about 0-3.1 V) versus the
Nano's 5 V scaling constants, and the user's wiring/impedance of the detectors and NTC.
Nano reference code: `SWRmeasure()`, `SendSWR()`, `CheckFanState()` in `TXlink4_Nano_Latest/TXlink4_Nano.ino`;
output messages are `{F..}{R..}{S..}{T..}` about 4 times a second.

## 4. Hardware facts worth keeping
- ESP32-S3; AD9851 DDS on a 30 MHz reference with the 6x multiplier (180 MHz); bit-banged DATA/W_CLK/FQ_UD/RESET on GPIO 10/12/11/9.
- Mic ADC: GPIO6 (ADC1 ch5), `adc_continuous`, 40 kHz, 20 kHz DSP rate. RSET LEDC PWM on GPIO2 (10-bit). MCP4725 I2C DAC on 47/48
  (`dac_task_enabled 0`). Timing-debug pins: 4, 5, 39. Free ADC1 pins by elimination: GPIO1, 3 (strapping, avoid), 7, 8.
- Architecture facts from the notes: both ISRs (gptimer, ADC conv_done) run on Core 1 together with Arduino `loop()` and USB-CDC
  serial; `dsp_task` is on Core 0 and is tight on time. Serial/USB-CDC activity on Core 1 has been A/B-shown to cause about
  20 Hz-spaced sidebands; any periodic extra work (including slow ADC reads) should be spectrum-tested.
- New PA: 2 W with RD15HVF1 MOSFETs (user report, 2026-11-04).

## 5. Open items (not done, none requested to proceed)
- ~20 Hz spurs with `{A}` enabled: cause unresolved. Suggested tests: vary `INSTRUMENTATION_PERIOD_MS`, a build flag that
  computes the meter but prints nothing, a single `Serial.write` per `{A}` line, close the port, scope the timing pin.
  Two questions to the user were never answered: which meter point was active, and were 4/8/12/16 Hz lines also present.
- MOD160 two-tone generator has a hard-coded 160-sample period that is only valid at 16 kHz (at 20 kHz it is 200); offered fix
  `SAMPLE_RATE_HZ/100u`.
- Jitter of the AD9851 write ISR (start and duration spread); lower-vs-upper sideband asymmetry; PWM filter LF/DC re-measure;
  other items listed under "Open items carried from earlier sessions" in `moving_forward_notes.md`.
- Host tests lost with the old sandbox (not in the repo): `test_level_meter.cpp`, `test_twotone_gen.cpp`, `test_hilbert_fold.c`,
  `sim_legacy_null_walk.c` and the EER simulation scripts. Recreate only if wanted.

## 6. How the previous thread worked (carry over what you still want)
- Log every finding in BOTH notes files: insert before `## Open items carried from earlier sessions, still unresolved` in
  `moving_forward_notes.md` and before `## Where to resume` in `null_bias_investigation.md`. Dated headings, hedged wording,
  confirmed vs unverified kept apart, own corrections recorded.
- Commit locally with the trailers `Co-Authored-By: Claude Sonnet 5.5 <noreply@anthropic.com>` and
  `Claude-Session: <session url>`. Never `git push` from the assistant. A stop hook nags about unpushed commits; the agreed reply is
  exactly "Acknowledged — not pushing."
- Deliver changed files to the chat and copy them into `C:\Users\office2\Downloads\Claude outputs` (the only connected folder
  is `C:\Users\office2\Downloads`).
- Firmware changes only on instruction. State clearly when something is uncompiled/untested on the hardware.
- The user has no Python: give charts as PNG. Host-only test programs (with `main()`) live in `host_tests/`, never in the sketch
  root (Arduino compiles every .c/.cpp there; a stray `main()` caused a link error).
- Send a short progress message when a long run goes quiet.
- The user's SDRuno TXlink plugin (C#) sends commands one character at a time. When using C# interpolated strings with format
  specifiers, avoid `}}` directly after the specifier (`$"{{F{x:F0}}}"` produced `{FF...}`).

## 7. Suggested first message in the new thread
"Continuing the ESP32 SSB/EER transmitter project. Handoff is in phase2_handoff.md plus the two notes files (repo: <URL>; my
latest changes are pushed). New phase: add three slow ADC channels (FWD/REV/PA temp) and the fan PWM from my Nano TXlink
project (TXlink4_Nano_Latest repo). Start with the feasibility test described in the last notes entry. Keep the standing rules
in section 6 of the handoff."
