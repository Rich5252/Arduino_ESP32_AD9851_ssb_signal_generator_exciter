#pragma once

/**
 * brace_input.h  (2026-10-12)
 *
 * Character-at-a-time parser for the "{X<argument>}" serial command style (as used by the TXlink SDR plugin and the
 * old TXlink Nano sketch), kept apart from the single-key command set so the two never mix:
 *
 *   '{'  starts a command (and throws away any partial command that was in progress),
 *   the first character after it is the command letter, the rest up to '}' is the argument,
 *   '}'  ends the command. It is also the escape: it always ends brace mode, so normal single-key commands resume.
 *
 * While a brace command is open EVERY other character is swallowed by the parser, including digits, letters and
 * the single-key commands (so "{F14200000}" cannot trigger the digit preset-load commands). CR/LF are ignored
 * inside the braces. Characters can arrive one at a time with any gaps; an open command that gets no character for
 * BRACE_INPUT_TIMEOUT_MS is dropped (checked with brace_input_check_timeout()) so a lost '}' cannot leave the
 * single-key commands switched off. A '}' with no open command is swallowed and does nothing.
 *
 * No Arduino/ESP dependencies: this file is host-testable (host_tests/test_brace_input.cpp).
 */

#include <stdint.h>
#include <stdbool.h>

#define BRACE_ARG_MAX            15        // longest accepted argument, in characters
#define BRACE_INPUT_TIMEOUT_MS   5000u     // max gap between characters of an open command

typedef enum {
    BRACE_PASS = 0,     // not part of a brace command - handle as a normal single-key command
    BRACE_CONSUMED,     // swallowed by the parser, nothing to do yet
    BRACE_COMMAND,      // '}' completed a command - see the brace_cmd_t that was filled in
    BRACE_CANCELLED,    // '}' closed an empty "{}" - brace mode left, nothing to run
    BRACE_OVERFLOW      // '}' closed a command whose argument was longer than BRACE_ARG_MAX - discarded
} brace_event_t;

typedef struct {
    char letter;                     // command letter (the first character after '{')
    char arg[BRACE_ARG_MAX + 1];     // the characters after the letter, NUL-terminated (may be empty)
} brace_cmd_t;

// Feed one received character. now_ms is any monotonically increasing millisecond clock (millis()).
brace_event_t brace_input_feed(char c, uint32_t now_ms, brace_cmd_t *cmd);

// True while a command is open (between '{' and '}').
bool brace_input_active(void);

// Call regularly. Returns true exactly once if an open command has been idle for BRACE_INPUT_TIMEOUT_MS and was dropped.
bool brace_input_check_timeout(uint32_t now_ms);

// Strict unsigned decimal parse: digits only, 1..9 of them. Returns false for empty, signs, spaces, other characters
// or too many digits.
bool brace_parse_uint(const char *s, uint32_t *out);

// (2026-10-12) Signed decimal parse for {C<ppb>}: an optional single leading '+' or '-', then 1..9 digits.
bool brace_parse_int(const char *s, int32_t *out);
