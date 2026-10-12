#include "brace_input.h"

static bool     s_active = false;
static bool     s_have_letter = false;
static bool     s_overflow = false;
static char     s_letter = 0;
static char     s_arg[BRACE_ARG_MAX + 1];
static int      s_arg_len = 0;
static uint32_t s_last_ms = 0;

static void start_command(uint32_t now_ms)
{
    s_active = true;
    s_have_letter = false;
    s_overflow = false;
    s_letter = 0;
    s_arg_len = 0;
    s_arg[0] = 0;
    s_last_ms = now_ms;
}

brace_event_t brace_input_feed(char c, uint32_t now_ms, brace_cmd_t *cmd)
{
    if (c == '{') {                 // always (re)starts, discarding whatever was open
        start_command(now_ms);
        return BRACE_CONSUMED;
    }
    if (!s_active) {
        // A '}' with nothing open is swallowed (it is not a single-key command); everything else is a normal key.
        return (c == '}') ? BRACE_CONSUMED : BRACE_PASS;
    }

    s_last_ms = now_ms;
    if (c == '}') {                 // end of command, and the escape back to normal single-key commands
        s_active = false;
        if (s_overflow) return BRACE_OVERFLOW;
        if (!s_have_letter) return BRACE_CANCELLED;
        if (cmd) {
            cmd->letter = s_letter;
            for (int i = 0; i <= s_arg_len; i++) cmd->arg[i] = s_arg[i];
        }
        return BRACE_COMMAND;
    }
    if (c == '\r' || c == '\n') return BRACE_CONSUMED;      // line endings inside the braces are ignored

    if (!s_have_letter) {
        s_letter = c;
        s_have_letter = true;
    } else if (s_arg_len < BRACE_ARG_MAX) {
        s_arg[s_arg_len++] = c;
        s_arg[s_arg_len] = 0;
    } else {
        s_overflow = true;          // keep swallowing until '}' so the tail is not run as single-key commands
    }
    return BRACE_CONSUMED;
}

bool brace_input_active(void) { return s_active; }

bool brace_input_check_timeout(uint32_t now_ms)
{
    if (s_active && (uint32_t)(now_ms - s_last_ms) > BRACE_INPUT_TIMEOUT_MS) {
        s_active = false;
        return true;
    }
    return false;
}

bool brace_parse_uint(const char *s, uint32_t *out)
{
    if (!s || !*s) return false;
    uint32_t v = 0;
    int n = 0;
    for (; *s; s++, n++) {
        if (*s < '0' || *s > '9' || n >= 9) return false;
        v = v * 10u + (uint32_t)(*s - '0');
    }
    if (out) *out = v;
    return true;
}

bool brace_parse_int(const char *s, int32_t *out)
{
    if (!s) return false;
    bool neg = false;
    if (*s == '+' || *s == '-') { neg = (*s == '-'); s++; }
    uint32_t mag;
    if (!brace_parse_uint(s, &mag)) return false;       // 1..9 digits, so mag fits comfortably in int32_t
    if (out) *out = neg ? -(int32_t)mag : (int32_t)mag;
    return true;
}
