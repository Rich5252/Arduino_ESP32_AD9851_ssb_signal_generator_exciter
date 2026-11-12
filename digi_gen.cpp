/**
 * digi_gen.cpp - see digi_gen.h. Port of the TXlink Nano digital modes (TXlink4_Nano_Latest d3db0ad):
 * TXlink4_Nano.ino (DigitalTXmsg, AdjustStart, set_digital_tx_buffer, setDigDefaults, the {V}{W}{X}{Y}{Z} commands),
 * RTTY.ino, rtty_queue.ino, JTTY.ino. Lines marked "Nano:" follow the Nano code; "ESP32:" marks a deliberate change.
 */

#include "digi_gen.h"
#include <string.h>
#include <math.h>

#ifndef DIGI_HOT
#define DIGI_HOT                // tick-path attribute hook (nothing needed: dsp_task is a task, not an ISR)
#endif

#define fence_release() __atomic_thread_fence(__ATOMIC_RELEASE)
#define fence_acquire() __atomic_thread_fence(__ATOMIC_ACQUIRE)

// ======================================================================================================== constants
// Nano: tone spacing / duration / symbol count per mode. ESP32: spacing in exact Hz (the Nano used FTW increments
// worth 6.2499 / 20.8332 / 41.6664 / 31.2498 Hz).
#define FT8_TONE_SPACING_HZ    6.25f
#define FT8_tone_duration      160
#define FT8_SYMBOL_COUNT       79
#define FT4_TONE_SPACING_HZ    (12000.0f / 576.0f)   // 20.8333
#define FT4_tone_duration      48
#define FT4_SYMBOL_COUNT       105
#define FT2_TONE_SPACING_HZ    (12000.0f / 288.0f)   // 41.6667
#define FT2_tone_duration      24
#define FT2_SYMBOL_COUNT       105
#define JTTY_TONE_SPACING_HZ   31.25f
#define JTTY_tone_duration     32

// Nano: "ERROR G4AHN" symbol lists, used when a {Z} list does not match the mode's symbol count
#define FT8TxMsgErr "3140652000026673266772757272250010533140652177703660737035675174622021543140652"

// Nano RTTY.ino
#define RTTY_BAUD              45.45
static const long BIT_DURATION_US      = (long)(1000000.0 / RTTY_BAUD + 0.5);   // 22002 us
static const long STOP_BIT_DURATION_US = (long)(BIT_DURATION_US * 2 + 0.5);     // 44004 us (Nano comment says 1.5)
#define MARK_FREQ   (170 / 2)
#define SPACE_FREQ  (-170 / 2)
static const int LETTERS_SHIFT_CODE = 31, FIGURES_SHIFT_CODE = 27, CR_CODE = 8, LF_CODE = 2, SPACE_CODE = 4, NULL_CODE = 0;

// Nano rtty_queue.h
#define MAX_QUEUE_MESSAGES 4
#define MAX_MESSAGE_LENGTH 72

// Nano JTTY.h
#define JTTY_SYNC_LEN      13
#define JTTY_DATA_LEN      46
#define JTTY_FRAME_LEN     (JTTY_SYNC_LEN + JTTY_DATA_LEN)   // 59
#define JTTY_MAX_FRAMES    16
#define JTTY_MAX_DATA      (JTTY_MAX_FRAMES * JTTY_DATA_LEN)  // 736 symbols
#define JTTY_BUF_BYTES     (JTTY_MAX_DATA / 4)                // 184 bytes
static const uint8_t jttySync[JTTY_SYNC_LEN] = {0,2,2,3,0,0,3,2,1,3,1,2,0};   // Nano: from WSJT-X jtty_fec_mod.f90

// Nano Gaussian_G_Scaled[] (TXlink4_Nano.ino), used only by shape 0
static const uint16_t Gaussian_G_Scaled[33] = {
    0, 518, 705, 948, 1264, 1667, 2177, 2815, 3602,
    4565, 5726, 7110, 8741, 10638, 12817, 15288, 18052,
    21102, 24421, 27978, 31732, 35630, 39604, 43582, 47477,
    51203, 54668, 57782, 60461, 62631, 64228, 65206, 65535
};

// ============================================================================================ transition table
// phi[x + W], x = ticks from a symbol boundary in -W..W: fraction of the tone change done (0 before, 1 after).
#define DIGI_MAX_W 1100
typedef struct {
    int   mode;
    int   W;                 // half width, ticks
    int   nsym;              // ticks per symbol (FT/JTTY)
    float spacing_hz;
    int   symbol_count;      // FT modes
    int   tone_duration_ms;
    float phi[2 * DIGI_MAX_W + 1];
} digi_tab_t;
static digi_tab_t s_tab[2];
static volatile int s_tab_pub = 0;

static digi_cfg_t s_cfg = { 20000, 1, 2.0f, 1.0f, 1.0f, 2.0f, 0.0f };

// ======================================================================================== shared (command side)
static volatile int  s_mode = DIGI_MODE_FT8;          // Nano TxMode
static volatile bool s_digi_tx_on = false;            // Nano DigiTxOn
static volatile int  s_tx_delay_ms = 250;             // Nano FT8txDelay {V}
static volatile int  s_df = 0;                        // Nano FTxTxDF {Y}
static volatile long s_millis_to_end = 14999;         // Nano FTxMillisToEnd {W}
static volatile bool s_abort_req = false;

static char s_msg[106] = FT8TxMsgErr;                 // Nano FTxTxMsg
typedef struct { uint8_t sym[106]; long tunechk; } ftbuf_t;   // Nano tx_buffer[] + TuneChk
static ftbuf_t s_ftbuf[2];
static volatile int      s_ftbuf_pub = 0;
static volatile uint32_t s_msg_chg = 0;               // Nano FTxTxMsgChanged, as a counter

static char s_rtty_q[MAX_QUEUE_MESSAGES][MAX_MESSAGE_LENGTH];   // Nano message_buffer
static volatile int      s_q_head = 0, s_q_tail = 0;            // head: loop writes; tail: tick writes
static volatile uint32_t s_q_clear_req = 0;
static volatile int      s_q_clear_head = 0;

static uint8_t           s_jtty_buf[JTTY_BUF_BYTES];  // Nano JTTY_BUF (shared with the RTTY queue there)
static volatile uint16_t s_jtty_count = 0;            // Nano jttyDataCount (published data symbols)
static volatile bool     s_jtty_tune = false;         // Nano jttyTune
static volatile uint32_t s_jtty_gen = 0;              // ESP32: bumped by every reset, so a parse cannot publish into
                                                      // a message the tick has just ended
static uint16_t s_jtty_rx_pos;
static bool     s_jtty_rx_bad;

// ============================================================================================ tick-side state
enum { ST_IDLE = 0, ST_WAIT, ST_DELAY, ST_RUN };
static volatile int      s_state = ST_IDLE;
static uint32_t          s_keydown_seen = 0;
static bool              s_keydown_init = false;
static uint32_t          s_delay_left = 0;
static const digi_tab_t *s_rt = 0;                    // table latched at the start of a run
static uint32_t          s_run_tick = 0;
static float             s_prev = 0, s_cur = 0, s_next = 0, s_freq = 0;
static int               s_u = 0, s_sym_len = 0;
static bool              s_next_frozen = false;
// FT
static int      s_i = 0;
static uint32_t s_t0 = 0;
static long     s_local_end = 0;                      // Nano _FTxMillisToEnd
static uint32_t s_chg_seen = 0;
static int      s_df_seen = 0;
static bool     s_restart_pending = false;
static int      s_restart_ai = 0;
// RTTY
static int  s_rt_phase = 0, s_rt_idx = 0, s_rt_pos = 0, s_rt_bit = 0, s_rt_code = 0;
static bool s_rt_ltrs = true;
static char s_rt_text[MAX_MESSAGE_LENGTH];
static uint32_t s_q_clear_seen = 0;
static double   s_rt_t_us = 0, s_rt_bit_start_us = 0, s_rt_bit_end_us = 0;
static int      s_rt_pending = -1;                    // next code, fetched when the stop bit starts (lookahead)
// JTTY
static uint32_t s_jt_total = 0;

// ---- status ----
static volatile uint32_t s_starts = 0, s_ends = 0, s_restarts = 0, s_aborts = 0, s_symbols_sent = 0;
static volatile int      s_first_symbol = 0;

// ======================================================================================================== helpers
static int ticks_of_ms(long ms) { return (int)((int64_t)ms * (int64_t)s_cfg.tick_hz / 1000); }
static long ms_of_ticks(uint32_t t) { return (long)((uint64_t)t * 1000u / s_cfg.tick_hz); }

bool digi_mode_is_digital(int mode)
{
    return mode == DIGI_MODE_RTTY || mode == DIGI_MODE_FT2 || mode == DIGI_MODE_FT4 || mode == DIGI_MODE_FT8 ||
           mode == DIGI_MODE_JTTY;
}

const char *digi_mode_name(int mode)
{
    switch (mode) {
    case DIGI_MODE_RTTY: return "RTTY";
    case DIGI_MODE_FT2:  return "FT2";
    case DIGI_MODE_FT4:  return "FT4";
    case DIGI_MODE_FT8:  return "FT8";
    case DIGI_MODE_JTTY: return "JTTY";
    default:             return "-";
    }
}

static inline float phi_at(const digi_tab_t *t, int x)
{
    if (x <= -t->W) return t->phi[0];
    if (x >= t->W) return 1.0f;
    return t->phi[x + t->W];
}

// Frequency for this tick: previous / current / next tone, erf steps centred on the symbol's start (u = 0) and end
// (u = sym_len) boundaries. Continuous across a boundary when prev/cur/next shift along.
static inline float tone_mix(void)
{
    const digi_tab_t *t = s_rt;
    return s_cur + (s_cur - s_prev) * (phi_at(t, s_u) - 1.0f) + (s_next - s_cur) * phi_at(t, s_u - s_sym_len);
}

// ================================================================================================= mode tables
static void build_table(digi_tab_t *t, int mode)
{
    float bt = 2.0f; int steps = 256, step_us = 64;   // Nano setDigDefaults() gTranSteps (<= 256) / gTranMicrosPerStep
    t->mode = mode;
    t->symbol_count = 0;
    switch (mode) {
    case DIGI_MODE_FT8:  t->spacing_hz = FT8_TONE_SPACING_HZ;  t->tone_duration_ms = FT8_tone_duration;
                         t->symbol_count = FT8_SYMBOL_COUNT; bt = s_cfg.bt_ft8; steps = 256; break;
    case DIGI_MODE_FT4:  t->spacing_hz = FT4_TONE_SPACING_HZ;  t->tone_duration_ms = FT4_tone_duration;
                         t->symbol_count = FT4_SYMBOL_COUNT; bt = s_cfg.bt_ft4; steps = 96; break;
    case DIGI_MODE_FT2:  t->spacing_hz = FT2_TONE_SPACING_HZ;  t->tone_duration_ms = FT2_tone_duration;
                         t->symbol_count = FT2_SYMBOL_COUNT; bt = s_cfg.bt_ft2; steps = 96; break;
    case DIGI_MODE_JTTY: t->spacing_hz = JTTY_TONE_SPACING_HZ; t->tone_duration_ms = JTTY_tone_duration;
                         bt = s_cfg.bt_jtty; steps = 64; break;
    case DIGI_MODE_RTTY: {
        // ESP32 2026-11-12: optional Gaussian edges for RTTY (DIGI_BT_RTTY > 0; 0 = hard keyed as the Nano). Same erf
        // step, BT relative to the 22 ms bit; boundaries fall between ticks, so the table is read with interpolation.
        t->spacing_hz = 0; t->tone_duration_ms = 0; t->nsym = 0; t->symbol_count = 0;
        t->W = 0; t->phi[0] = 0;
        if (s_cfg.bt_rtty > 0.0f) {
            const double dt = 1.0 / (double)s_cfg.tick_hz;
            double Tb = BIT_DURATION_US * 1e-6;
            double sigma = sqrt(log(2.0)) / (2.0 * M_PI * (double)s_cfg.bt_rtty) * Tb;
            int W = (int)ceil(4.5 * sigma / dt);
            int maxW = (int)floor(Tb / dt) - 1;                // only neighbouring boundaries are summed
            if (W > maxW) W = maxW;
            if (W > DIGI_MAX_W) W = DIGI_MAX_W;
            t->W = W;
            for (int x = -W; x <= W; x++)
                t->phi[x + W] = (float)(0.5 * (1.0 + erf((x * dt) / (sigma * sqrt(2.0)))));
        }
        return;
    }
    default:             t->spacing_hz = 0; t->tone_duration_ms = 0; t->W = 0; t->nsym = 0; t->phi[0] = 0; return;
    }
    t->nsym = ticks_of_ms(t->tone_duration_ms);
    const double dt = 1.0 / (double)s_cfg.tick_hz;

    if (s_cfg.shape == 0) {
        // Nano curve as it ran: gaussianTransition() interpolation in integers, one DDS write per 64 us step,
        // started at the boundary minus the Nano's integer "half transition" in ms.
        static double vals[256];                               // static: loop() has a small stack
        int f = steps / 32, n = 0;
        for (int g = 1; g <= 32; g++) {
            uint16_t cur = Gaussian_G_Scaled[g], prv = Gaussian_G_Scaled[g - 1];
            uint16_t incr = (uint16_t)((uint16_t)(cur - prv) / f);
            for (int h = 1; h <= f; h++) vals[n++] = (double)(uint16_t)(prv + incr * h) / 65536.0;
        }
        long half_ms = (mode == DIGI_MODE_JTTY) ? (long)((steps * step_us) / 2000)            // JTTY.ino halfTranMs
                                                : (long)(steps / (1000 / step_us) / 2);       // DigitalTXmsg waitFor
        double start_s = -(double)half_ms / 1000.0, len_s = steps * step_us * 1e-6;
        int W = (int)ceil(fmax(-start_s, start_s + len_s) / dt) + 1;
        if (W > DIGI_MAX_W) W = DIGI_MAX_W;
        t->W = W;
        for (int x = -W; x <= W; x++) {
            double tau = x * dt - start_s;                      // seconds since the transition started
            double v;
            if (tau < 0) v = 0.0;
            else if (tau >= len_s) v = 1.0;
            else { int k = (int)(tau / (step_us * 1e-6)); if (k >= n) k = n - 1; v = vals[k]; }
            t->phi[x + W] = (float)v;
        }
    } else {
        // WSJT-X GFSK: Gaussian frequency filter, sigma = sqrt(ln 2) / (2 pi BT) symbols; a tone change is the step
        // response 0.5 (1 + erf(t / (sigma sqrt 2))). +/-4.5 sigma kept (the residual step is ~3e-6 of a tone).
        double T = t->tone_duration_ms / 1000.0;
        double sigma = sqrt(log(2.0)) / (2.0 * M_PI * (double)bt) * T;
        int W = (int)ceil(4.5 * sigma / dt);
        if (W > DIGI_MAX_W) W = DIGI_MAX_W;
        if (W >= t->nsym) W = t->nsym - 1;                     // only neighbouring boundaries are summed
        t->W = W;
        for (int x = -W; x <= W; x++)
            t->phi[x + W] = (float)(0.5 * (1.0 + erf((x * dt) / (sigma * sqrt(2.0)))));
    }
}

static void publish_table(int mode)
{
    int nb = s_tab_pub ^ 1;
    build_table(&s_tab[nb], mode);
    fence_release();
    s_tab_pub = nb;
}

// ======================================================================================================== RTTY
// Nano RTTY.ino getBaudotCode(), unchanged (ITA2 as MMTTY)
typedef struct { int code; bool requires_figures_shift; } BaudotEntry;
static BaudotEntry getBaudotCode(char c)
{
    if (c >= 'a' && c <= 'z') c = (char)(c - 'a' + 'A');      // Nano: toupper(c)
    if (c == '\r' || c == 8) return {CR_CODE, false};
    if (c == '\n' || c == 2) return {LF_CODE, false};
    if (c == ' ') return {SPACE_CODE, false};
    switch (c) {
        case 'E': return {1, false};  case 'A': return {3, false};  case 'S': return {5, false};
        case 'I': return {6, false};  case 'U': return {7, false};  case 'D': return {9, false};
        case 'R': return {10, false}; case 'J': return {11, false}; case 'N': return {12, false};
        case 'F': return {13, false}; case 'C': return {14, false}; case 'K': return {15, false};
        case 'T': return {16, false}; case 'Z': return {17, false}; case 'L': return {18, false};
        case 'W': return {19, false}; case 'H': return {20, false}; case 'Y': return {21, false};
        case 'P': return {22, false}; case 'Q': return {23, false}; case 'O': return {24, false};
        case 'B': return {25, false}; case 'G': return {26, false}; case 'M': return {28, false};
        case 'X': return {29, false}; case 'V': return {30, false};
        case '3': return {1, true};   case '-': return {3, true};   case '8': return {6, true};
        case '7': return {7, true};   case '$': return {9, true};   case '4': return {10, true};
        case '\'': return {11, true}; case ',': return {12, true};  case '!': return {13, true};
        case ':': return {14, true};  case '(': return {15, true};  case '5': return {16, true};
        case '"': return {17, true};  case ')': return {18, true};  case '2': return {19, true};
        case '6': return {21, true};  case '0': return {22, true};  case '1': return {23, true};
        case '9': return {24, true};  case '?': return {25, true};  case '&': return {26, true};
        case '.': return {28, true};  case '/': return {29, true};  case ';': return {30, true};
        case 0:   return {NULL_CODE, false};
        default:  return {SPACE_CODE, false};
    }
}

// Nano rtty_queue.ino (enqueue in loop context, dequeue in the tick; clearQueue() is a request the tick applies)
static bool enqueueMessage(const char *newText)
{
    int next_head = (s_q_head + 1) % MAX_QUEUE_MESSAGES;
    if (next_head == s_q_tail) return false;                   // Nano: full -> dropped
    strncpy(s_rtty_q[s_q_head], newText, MAX_MESSAGE_LENGTH - 1);
    s_rtty_q[s_q_head][MAX_MESSAGE_LENGTH - 1] = '\0';
    fence_release();
    s_q_head = next_head;
    return true;
}
static void clearQueue(void)
{
    s_q_clear_head = s_q_head;
    fence_release();
    s_q_clear_req = s_q_clear_req + 1;
}
static void apply_queue_clear(void)                            // tick side
{
    if (s_q_clear_seen != s_q_clear_req) {
        s_q_clear_seen = s_q_clear_req;
        fence_acquire();
        s_q_tail = s_q_clear_head;
    }
}
static bool dequeueMessage(char *out)                          // ESP32: copies, so the slot can be reused at once
{
    apply_queue_clear();
    if (s_q_head == s_q_tail) return false;
    fence_acquire();
    memcpy(out, s_rtty_q[s_q_tail], MAX_MESSAGE_LENGTH);
    s_q_tail = (s_q_tail + 1) % MAX_QUEUE_MESSAGES;
    return true;
}

// Nano transmitRttyString() as a code-at-a-time state machine. Returns the next Baudot code, or -1 at the end.
enum { RT_PRE = 0, RT_TEXT, RT_TRAIL, RT_END };
static int rtty_next_code(void)
{
    static const int pre[6] = { 27, 31, 27, 31, 8, 2 };        // FIGS LTRS FIGS LTRS CR LF
    for (;;) {
        switch (s_rt_phase) {
        case RT_PRE:
            if (s_rt_idx == 4) s_rt_ltrs = true;               // Nano: current_shift_LTRS = true after the 4th
            if (s_rt_idx < 6) return pre[s_rt_idx++];
            s_rt_phase = RT_TEXT; s_rt_pos = 0;
            break;
        case RT_TEXT: {
            char c = s_rt_text[s_rt_pos];
            if (c && s_digi_tx_on) {                           // Nano: while (*text && DigiTxOn)
                BaudotEntry e = getBaudotCode(c);
                if (e.requires_figures_shift && s_rt_ltrs) { s_rt_ltrs = false; return FIGURES_SHIFT_CODE; }
                if (!e.requires_figures_shift && !s_rt_ltrs) { s_rt_ltrs = true; return LETTERS_SHIFT_CODE; }
                s_rt_pos++;
                return e.code;
            }
            if (dequeueMessage(s_rt_text)) { s_rt_pos = 0; return SPACE_CODE; }   // Nano: a space between strings
            s_rt_phase = RT_TRAIL; s_rt_idx = 0;
            break;
        }
        case RT_TRAIL:
            if (s_rt_idx == 0) { s_rt_idx = 1; return CR_CODE; }
            if (s_rt_idx == 1) { s_rt_idx = 2; return LF_CODE; }
            if (s_rt_idx == 2) { s_rt_idx = 3; if (!s_rt_ltrs) { s_rt_ltrs = true; return LETTERS_SHIFT_CODE; } }
            s_rt_phase = RT_END;
            break;
        default:
            return -1;
        }
    }
}

// bit b of the current code: 0 = start (space), 1..5 = data LSB first, 6 = stop (mark, two bit times)
static int  rtty_bit_value(int b)  { return b == 0 ? 0 : (b == 6 ? 1 : (s_rt_code >> (b - 1)) & 1); }
static float rtty_hz(int v)        { return (float)(v ? MARK_FREQ : SPACE_FREQ); }

// erf step at a boundary x_us away (x < 0: before it). W == 0: hard step, exactly the Nano's keying.
static float rtty_phi(double x_us)
{
    const digi_tab_t *t = s_rt;
    if (t->W == 0) return x_us >= 0.0 ? 1.0f : 0.0f;
    double xt = x_us * (double)s_cfg.tick_hz * 1e-6;
    if (xt <= -t->W) return t->phi[0];
    if (xt >= t->W) return 1.0f;
    int i = (int)floor(xt);
    float fr = (float)(xt - i);
    float a = t->phi[i + t->W];
    float b = (i + 1 >= t->W) ? 1.0f : t->phi[i + 1 + t->W];
    return a + (b - a) * fr;
}

// Tone after the current bit: the next data/stop bit, or the next character's start bit (space), or - when the
// message ends - mark held (no change).
static float rtty_next_hz(void)
{
    if (s_rt_bit < 6) return rtty_hz(rtty_bit_value(s_rt_bit + 1));
    return s_rt_pending >= 0 ? rtty_hz(0) : s_cur;
}
static long rtty_bit_len_us(int b) { return b == 6 ? STOP_BIT_DURATION_US : BIT_DURATION_US; }

// ======================================================================================================== JTTY
static void jttyPutSymbol(uint16_t d, uint8_t s)
{
    uint8_t *b = &s_jtty_buf[d >> 2];
    uint8_t sh = (uint8_t)((d & 3) * 2);
    *b = (uint8_t)((*b & ~(3 << sh)) | ((s & 3) << sh));
}
static uint8_t jttyHexVal(char c)
{
    if (c >= '0' && c <= '9') return (uint8_t)(c - '0');
    if (c >= 'A' && c <= 'F') return (uint8_t)(c - 'A' + 10);
    if (c >= 'a' && c <= 'f') return (uint8_t)(c - 'a' + 10);
    return 0xFF;
}
static uint32_t jttyTotalSymbols(void) { return (uint32_t)(s_jtty_count / JTTY_DATA_LEN) * JTTY_FRAME_LEN; }
static uint8_t jttyTone(uint32_t i)
{
    uint32_t f = i % JTTY_FRAME_LEN;
    if (f < JTTY_SYNC_LEN) return jttySync[f];
    uint32_t d = (i / JTTY_FRAME_LEN) * JTTY_DATA_LEN + (f - JTTY_SYNC_LEN);
    return (uint8_t)((s_jtty_buf[d >> 2] >> ((d & 3) * 2)) & 3);
}
static void jttyReset(void)
{
    s_jtty_count = 0;
    s_jtty_tune = false;
    s_jtty_gen = s_jtty_gen + 1;
}

// ==================================================================================================== commands
// Nano set_digital_tx_buffer(): rebuild the numeric symbol list from FTxTxMsg; all zeros = Tune (TuneChk 0)
static void set_digital_tx_buffer(void)
{
    if (s_mode == DIGI_MODE_RTTY) return;
    const digi_tab_t *t = &s_tab[s_tab_pub];
    int count = t->symbol_count;
    if (count <= 0) return;
    const char *symbols = ((int)strlen(s_msg) == count) ? s_msg : FT8TxMsgErr;   // Nano: FT8 error list either way
    int slen = (int)strlen(symbols);
    int nb = s_ftbuf_pub ^ 1;
    ftbuf_t *b = &s_ftbuf[nb];
    memset(b->sym, 0, sizeof b->sym);
    b->tunechk = 0;
    for (int i = 0; i < count; i++) {
        char c = (i < slen) ? symbols[i] : 0;                  // Nano: String.charAt() past the end = 0
        if (c >= '0' && c <= '9') { b->sym[i] = (uint8_t)(c - '0'); b->tunechk += b->sym[i]; }
    }
    fence_release();
    s_ftbuf_pub = nb;
}

void digi_init(const digi_cfg_t *cfg)
{
    if (cfg) s_cfg = *cfg;
    if (s_cfg.tick_hz == 0) s_cfg.tick_hz = 20000;
    s_mode = DIGI_MODE_FT8;                                    // Nano DEFAULT_MODE; the {M} handler sets the real one
    publish_table(s_mode);
    set_digital_tx_buffer();
}

void digi_set_mode(int mode)
{
    // ESP32: a mode change stops any digital transmission first (the Nano never changed mode mid-message).
    s_digi_tx_on = false;
    s_abort_req = true;
    s_mode = mode;
    if (!digi_mode_is_digital(mode)) return;
    publish_table(mode);                                       // Nano setDigDefaults()
    if (mode == DIGI_MODE_JTTY) { clearQueue(); jttyReset(); } // Nano setJttyDefaults()
}

void digi_cmd_V(long ms) { s_tx_delay_ms = (int)ms; }        // Nano FT8txDelay

void digi_cmd_W(long ms)
{
    s_millis_to_end = ms;                                      // Nano FTxMillisToEnd
    if (ms == 0) s_digi_tx_on = false;
}

void digi_cmd_X(long on)
{
    if (!digi_mode_is_digital(s_mode)) return;                 // ESP32: only in a digital mode
    s_digi_tx_on = (on != 0);
    if (!on) clearQueue();                                     // Nano: clear outstanding RTTY messages on {X0}
    if (!on && s_mode == DIGI_MODE_JTTY) jttyReset();          // Nano: JTTY abort
}

void digi_cmd_Y(long hz) { s_df = (int)hz; }                  // Nano FTxTxDF

void digi_cmd_Z(const char *msg)
{
    if (!msg) return;
    bool changed = false;
    if (s_mode == DIGI_MODE_RTTY) {
        enqueueMessage(msg);
    } else if (strlen(msg) == FT4_SYMBOL_COUNT || strlen(msg) == FT8_SYMBOL_COUNT) {   // FT2 uses FT4's count
        if (strcmp(msg, s_msg) != 0) { strcpy(s_msg, msg); changed = true; }
    }
    set_digital_tx_buffer();
    if (changed) { fence_release(); s_msg_chg = s_msg_chg + 1; }   // Nano FTxTxMsgChanged, after the new list is up
}

void digi_cmd_J(const char *payload)
{
    if (s_mode != DIGI_MODE_JTTY || !payload) return;          // Nano: {J..} is only special in JTTY mode
    // Nano jttyRxBegin()
    uint32_t gen = s_jtty_gen;
    bool busy = s_digi_tx_on || s_state != ST_IDLE;
    if (busy && s_jtty_count > 0) {
        s_jtty_rx_pos = s_jtty_count;                          // append behind the current message
    } else {
        s_jtty_count = 0;                                      // new message
        s_jtty_rx_pos = 0;
        s_jtty_tune = false;
    }
    s_jtty_rx_bad = false;
    bool tune = false;
    // Nano jttyRxChar()
    for (const char *p = payload; *p; p++) {
        char c = *p;
        if (s_jtty_rx_bad) break;
        if (c == 'T' && s_jtty_rx_pos == 0 && s_jtty_count == 0) { tune = true; continue; }
        if (tune) { s_jtty_rx_bad = true; break; }
        uint8_t v = jttyHexVal(c);
        if (v == 0xFF || s_jtty_rx_pos + 2 > JTTY_MAX_DATA) { s_jtty_rx_bad = true; break; }
        jttyPutSymbol(s_jtty_rx_pos++, v & 3);
        jttyPutSymbol(s_jtty_rx_pos++, (uint8_t)(v >> 2));
    }
    // Nano jttyRxEnd(): publish whole frames only. ESP32: not if the tick reset the message meanwhile.
    if (gen != s_jtty_gen) return;
    if (tune && !s_jtty_rx_bad) s_jtty_tune = true;
    if (!s_jtty_rx_bad && s_jtty_rx_pos > s_jtty_count && (s_jtty_rx_pos % JTTY_DATA_LEN) == 0) {
        fence_release();
        s_jtty_count = s_jtty_rx_pos;
    }
}

void digi_abort(void)
{
    s_digi_tx_on = false;
    s_abort_req = true;
}

// ================================================================================================ FT sequencing
// Nano AdjustStart(): first symbol to send for the time left in the period (WSJT-X late-start rule)
static int AdjustStart(long time_left_ms, int mode, int symbol_count, int tone_duration)
{
    int iRet = 0;
    long TimeLeft = time_left_ms;
    long KillTime = 5500;                 // FT8: will decode on a partial message up to this time remaining
    long TruncTime = 12650;               // FT8: minimum time left for the whole message
    if (mode == DIGI_MODE_FT4) { TimeLeft -= 1460; KillTime = 2020; TruncTime = 5040; }
    if (mode == DIGI_MODE_FT2) { TimeLeft -= 1460 / 2; KillTime = 2020 / 2; TruncTime = 5040 / 2; }
    if (TimeLeft < KillTime) {
        iRet = symbol_count;
    } else if (TimeLeft <= TruncTime) {
        iRet = (int)(symbol_count - TimeLeft / tone_duration);
        if (iRet < 0) iRet = 0;
        if (iRet > symbol_count) iRet = symbol_count;
    }
    return iRet;
}

static const ftbuf_t *ft_buf(void) { int p = s_ftbuf_pub; fence_acquire(); return &s_ftbuf[p]; }
static float ft_tone(int i)
{
    const ftbuf_t *b = ft_buf();
    int k = (i >= 0 && i < 106) ? i : 0;
    return (float)s_df_seen + (float)b->sym[k] * s_rt->spacing_hz;
}

// ============================================================================================================ tick
static void end_run(void)
{
    if (s_rt && s_rt->mode == DIGI_MODE_JTTY) jttyReset();     // Nano transmitJtty(): jttyReset() at the end
    s_digi_tx_on = false;                                      // Nano: key up (main loop does the TX hold)
    s_state = ST_IDLE;
    s_ends = s_ends + 1;
}

static void start_run(void)
{
    int p = s_tab_pub; fence_acquire();
    s_rt = &s_tab[p];
    s_run_tick = 0;
    s_u = 0;
    s_next_frozen = false;
    s_restart_pending = false;
    s_symbols_sent = 0;
    s_starts = s_starts + 1;
    s_state = ST_RUN;
    const int mode = s_rt->mode;

    if (mode == DIGI_MODE_RTTY || s_mode == DIGI_MODE_RTTY) {
        s_rt_phase = RT_PRE; s_rt_idx = 0; s_rt_ltrs = true;
        if (!dequeueMessage(s_rt_text)) { end_run(); return; }   // Nano: nothing queued -> straight back to RX
        s_rt_code = rtty_next_code();
        s_rt_pending = -1;
        s_rt_bit = 0;
        s_rt_t_us = 0;
        s_rt_bit_start_us = 0;
        s_rt_bit_end_us = (double)rtty_bit_len_us(0);
        s_symbols_sent = 1;
        s_first_symbol = 0;
        s_freq = s_cur = s_prev = rtty_hz(rtty_bit_value(0));
        s_next = rtty_next_hz();
        return;
    }
    s_sym_len = s_rt->nsym;
    if (mode == DIGI_MODE_JTTY) {
        s_df_seen = s_df;
        s_i = 0;
        s_first_symbol = 0;
        if (s_jtty_tune) { s_cur = s_prev = s_next = (float)s_df_seen; s_freq = s_cur; return; }
        s_jt_total = jttyTotalSymbols();
        if (s_jt_total == 0) { end_run(); return; }
        s_cur = s_prev = (float)s_df_seen + jttyTone(0) * s_rt->spacing_hz;
        s_next = s_cur;
        s_freq = s_cur;
        s_symbols_sent = 1;
        return;
    }
    // FT8 / FT4 / FT2 - Nano DigitalTXmsg() start
    s_local_end = s_millis_to_end;
    s_df_seen = s_df;
    s_chg_seen = s_msg_chg;
    s_i = AdjustStart(s_local_end, mode, s_rt->symbol_count, s_rt->tone_duration_ms);
    s_first_symbol = s_i;
    s_t0 = 0;
    if (s_i >= s_rt->symbol_count || s_local_end <= 0) { end_run(); return; }
    s_cur = s_prev = ft_tone(s_i);
    s_next = s_cur;
    s_freq = s_cur;
    s_symbols_sent = 1;
}

// Tick results: TK_MORE = produced this tick's frequency, carry on; TK_LAST = produced it and that was the last;
// TK_DONE = ended without producing one (abort / nothing left).
enum { TK_DONE = 0, TK_LAST, TK_MORE };

// One FT tick.
static int ft_tick(void)
{
    const digi_tab_t *t = s_rt;
    const int count = t->symbol_count;
    const bool tune = (ft_buf()->tunechk == 0);

    // Nano: a changed message or offset during TX restarts the sequence (checked in its wait loop).
    // ESP32: the new sequence starts W ticks later, so the tone change is a whole erf step centred on that point.
    if (!s_restart_pending && (s_msg_chg != s_chg_seen || s_df != s_df_seen) && s_u < s_sym_len - t->W) {
        long elapsed_ms = ms_of_ticks(s_run_tick - s_t0) + ms_of_ticks((uint32_t)t->W);
        s_local_end -= elapsed_ms;                             // Nano: _FTxMillisToEnd -= (millis() - tStart)
        s_chg_seen = s_msg_chg;
        s_df_seen = s_df;
        s_restart_ai = AdjustStart(s_local_end, t->mode, count, t->tone_duration_ms);   // Nano: i = AdjustStart()
        s_restart_pending = true;
        s_sym_len = s_u + t->W;
        int inext = (ft_buf()->tunechk == 0) ? 2 : s_restart_ai + 1;   // Nano: break; tune i = 1; for-loop i++
        s_next = (inext < count && s_local_end > 0) ? ft_tone(inext) : s_cur;
        s_next_frozen = true;
        s_restarts = s_restarts + 1;
    }
    if (!s_next_frozen) {
        if (s_u >= s_sym_len - t->W) s_next_frozen = true;
        int inext = tune ? 2 : s_i + 1;
        s_next = (inext < count) ? ft_tone(inext) : s_cur;
    }

    s_freq = tone_mix();
    s_u++;
    s_run_tick++;
    if (s_u < s_sym_len) return TK_MORE;

    // ---- symbol boundary: Nano post-symbol checks ----
    if (s_restart_pending) {
        s_restart_pending = false;
        s_t0 = s_run_tick;                                     // Nano: tStart = millis(); n = -1 ... n++
        s_i = tune ? 1 : s_restart_ai;
    } else if (tune) {
        s_i = 1;                                               // Nano: if (TuneChk == 0) i = 1
    }
    if (s_millis_to_end == 0) s_local_end = 0;                 // Nano: {W0} forces the stop
    long elapsed = ms_of_ticks(s_run_tick - s_t0);
    if (!s_digi_tx_on && elapsed > s_local_end) return TK_LAST;  // Nano: TX off and the period is over
    if (s_local_end <= 0) return TK_LAST;                        // Nano: its waits stop at once, nothing more is sent
    s_i++;
    if (s_i >= count) return TK_LAST;
    s_prev = s_cur;
    s_cur = s_next;
    s_u = 0;
    s_sym_len = t->nsym;
    s_next_frozen = false;
    s_symbols_sent = s_symbols_sent + 1;
    return TK_MORE;
}

static int jtty_tick(void)
{
    const digi_tab_t *t = s_rt;
    if (!s_digi_tx_on) return TK_DONE;                           // Nano: {X0} / lockout -> goto done
    if (s_jtty_tune) { s_freq = s_cur; s_run_tick++; return TK_MORE; }   // Nano {JT}: steady tone 0 until {X0}
    if (!s_next_frozen) {
        if (s_u >= s_sym_len - t->W) s_next_frozen = true;
        s_jt_total = jttyTotalSymbols();                       // Nano: re-read every symbol (frames appended in TX)
        s_next = ((uint32_t)s_i + 1 < s_jt_total) ? (float)s_df + jttyTone(s_i + 1) * t->spacing_hz : s_cur;
    }
    s_freq = tone_mix();
    s_u++;
    s_run_tick++;
    if (s_u < s_sym_len) return TK_MORE;
    s_i++;
    if ((uint32_t)s_i >= jttyTotalSymbols()) return TK_LAST;
    s_prev = s_cur;
    s_cur = s_next;
    s_u = 0;
    s_next_frozen = false;
    s_symbols_sent = s_symbols_sent + 1;
    return TK_MORE;
}

static int rtty_tick(void)
{
    s_rt_t_us += 1e6 / (double)s_cfg.tick_hz;
    s_run_tick++;
    while (s_rt_t_us >= s_rt_bit_end_us) {
        if (++s_rt_bit > 6) {
            s_rt_code = s_rt_pending;                          // fetched when the stop bit began
            if (s_rt_code < 0) return TK_DONE;                 // the last stop bit has been sent in full
            s_rt_pending = -1;
            s_rt_bit = 0;
            s_symbols_sent = s_symbols_sent + 1;
        }
        // ESP32: the next character is fetched at the start of the stop bit (44 ms early), so the edge into its
        // start bit can begin before the boundary. Nano: fetched after the stop bit.
        if (s_rt_bit == 6) s_rt_pending = rtty_next_code();
        s_prev = s_cur;
        s_cur = rtty_hz(rtty_bit_value(s_rt_bit));
        s_rt_bit_start_us = s_rt_bit_end_us;
        s_rt_bit_end_us += (double)rtty_bit_len_us(s_rt_bit);
        s_next = rtty_next_hz();
    }
    // mark +85 / space -85 Hz; with DIGI_BT_RTTY 0 the steps are hard (Nano), otherwise Gaussian, centred on the bit
    // boundaries as in the FT modes
    s_freq = s_cur + (s_cur - s_prev) * (rtty_phi(s_rt_t_us - s_rt_bit_start_us) - 1.0f)
                   + (s_next - s_cur) * rtty_phi(s_rt_t_us - s_rt_bit_end_us);
    return TK_MORE;
}

void DIGI_HOT digi_tick(bool requested, bool tx_ready, uint32_t keydown_count, digi_out_t *out)
{
    if (!s_keydown_init) { s_keydown_seen = keydown_count; s_keydown_init = true; }
    if (s_state == ST_IDLE) apply_queue_clear();

    if (s_abort_req) {
        s_abort_req = false;
        if (s_state != ST_IDLE) { s_aborts = s_aborts + 1; s_state = ST_IDLE; }
    }
    // ESP32: {D} or a trip while a transmission is under way ends it at once (the Nano kept sending until the end
    // of the message with the DDS powered down)
    if (!requested && (s_state == ST_DELAY || s_state == ST_RUN)) {
        s_digi_tx_on = false;
        s_aborts = s_aborts + 1;
        s_state = ST_IDLE;
    }

    switch (s_state) {
    case ST_IDLE:
        if (s_digi_tx_on) s_state = ST_WAIT;
        break;
    case ST_WAIT:
        if (!s_digi_tx_on) { s_state = ST_IDLE; break; }
        if (tx_ready) {
            bool fresh = (keydown_count != s_keydown_seen);
            s_keydown_seen = keydown_count;
            long d = 0;
            if (fresh) {                                       // Nano: TX delay only when going from RX to TX
                d = s_tx_delay_ms;
                if (s_mode == DIGI_MODE_FT4) d = d / 2;
                if (s_mode == DIGI_MODE_FT2) d = 0;            // Nano: no time for delay
                if (s_mode == DIGI_MODE_JTTY) d = 0;           // Nano: no T/R period, start now
                if (d < 0) d = 0;
            }
            s_delay_left = (uint32_t)ticks_of_ms(d);
            s_state = ST_DELAY;
        }
        break;
    case ST_DELAY:
        if (!s_digi_tx_on) { s_state = ST_IDLE; break; }      // Nano: {X0} during the delay -> no message
        if (s_delay_left > 0) { s_delay_left--; break; }
        start_run();
        break;
    default:
        break;
    }

    bool ran = false;
    if (s_state == ST_RUN) {
        int r;
        if (s_rt->mode == DIGI_MODE_RTTY)      r = rtty_tick();
        else if (s_rt->mode == DIGI_MODE_JTTY) r = jtty_tick();
        else                                   r = ft_tick();
        ran = (r != TK_DONE);
        if (r != TK_MORE) end_run();
    }

    if (out) {
        out->freq_hz = s_freq;
        out->rf_on = ran;
        out->key = s_digi_tx_on || s_state != ST_IDLE;
    }
}

void digi_get_status(digi_status_t *st)
{
    if (!st) return;
    const digi_tab_t *t = &s_tab[s_tab_pub];
    st->mode = s_mode;
    st->digi_tx_on = s_digi_tx_on;
    st->state = s_state;
    st->starts = s_starts; st->ends = s_ends; st->restarts = s_restarts; st->aborts = s_aborts;
    st->first_symbol = s_first_symbol;
    st->symbols_sent = s_symbols_sent;
    st->run_ticks = s_run_tick;
    st->tx_delay_ms = s_tx_delay_ms;
    st->df_hz = s_df;
    st->millis_to_end = s_millis_to_end;
    if (s_mode == DIGI_MODE_RTTY)      st->symbol_count = (s_q_head - s_q_tail + MAX_QUEUE_MESSAGES) % MAX_QUEUE_MESSAGES;
    else if (s_mode == DIGI_MODE_JTTY) st->symbol_count = (int)jttyTotalSymbols();
    else                               st->symbol_count = t->symbol_count;
    st->tune = (s_mode == DIGI_MODE_JTTY) ? s_jtty_tune : (s_ftbuf[s_ftbuf_pub].tunechk == 0);
    st->half_window_ticks = t->W;
}
