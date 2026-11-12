// Host test for digi_gen.cpp (2026-11-11). Build and run on a PC, never in the Arduino sketch folder:
//   g++ -std=c++17 -O2 -Wall -I.. test_digi_gen.cpp ../digi_gen.cpp -o test_digi_gen && ./test_digi_gen [dumpdir]
// Drives the generator through a model of tx_control's sequencer (5 ms relay, TX hold) at 20 kHz and checks every
// mode against independent references: the WSJT-X GFSK pulse sum (double precision), the Nano's own transition
// steps, AdjustStart cases, restarts, {W0}/{X0}/{D}, RTTY decoded back to text, JTTY sync/data/chaining.
// With a directory argument it also writes FT8/FT4 frequency traces (float32, Hz per tick) for a spectrum check.

#include "digi_gen.h"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cstdarg>
#include <string>
#include <vector>

static int fails = 0, checks = 0;
static void check(bool ok, const char *fmt, ...) __attribute__((format(printf, 2, 3)));
static void check(bool ok, const char *fmt, ...)
{
    checks++;
    if (ok) return;
    fails++;
    va_list ap; va_start(ap, fmt); printf("FAIL: "); vprintf(fmt, ap); printf("\n"); va_end(ap);
}

static const int FS = 20000;
static const int RELAY = 100;          // 5 ms
static const int HOLD = 4000;          // 200 ms

// ---- model of tx_control's sequencer for digital modes ----
static uint32_t g_kd = 0;          // key-down counter keeps counting across runs, as tx_start_count() does

struct Seq {
    bool requested = true;
    int state = 0;                     // 0 RX, 1 KEYING, 2 TX
    int in_state = 0;
    int keyup = 0;
    digi_out_t out{};
    std::vector<float> f;
    std::vector<uint8_t> rf;
    std::vector<uint8_t> key;
    void tick()
    {
        bool want = requested && (out.key || (state == 2 && keyup < HOLD));
        switch (state) {
        case 0: if (want) { state = 1; in_state = 0; g_kd++; } break;
        case 1: if (!want) state = 0; else if (++in_state >= RELAY) state = 2; break;
        case 2: if (!want) state = 0; break;
        }
        digi_tick(requested, state == 2, g_kd, &out);
        keyup = out.key ? 0 : keyup + 1;
        f.push_back(out.freq_hz);
        rf.push_back(out.rf_on && state == 2);
        key.push_back(out.key);
    }
    void run(int n) { for (int i = 0; i < n; i++) tick(); }
    void run_until_idle(int max_ticks)
    {
        for (int i = 0; i < max_ticks; i++) { tick(); if (!out.key && state == 0) break; }
    }
    int rf_first() const { for (size_t i = 0; i < rf.size(); i++) if (rf[i]) return (int)i; return -1; }
    int rf_last() const { for (size_t i = rf.size(); i-- > 0;) if (rf[i]) return (int)i; return -1; }
    int rf_count() const { int n = 0; for (auto r : rf) n += r; return n; }
};

static digi_cfg_t cfg(int shape, float bt_rtty = 0.0f) { digi_cfg_t c = { (uint32_t)FS, shape, 2.0f, 1.0f, 1.0f, 2.0f, bt_rtty }; return c; }

static std::string random_symbols(int n, int m, unsigned seed)
{
    std::string s; srand(seed);
    for (int i = 0; i < n; i++) s += (char)('0' + rand() % m);
    return s;
}

// WSJT-X GFSK reference: f(t) = DF + spacing * sum_k a_k p(t/T - k - 0.5), first/last symbols extended (as WSJT-X's
// dummy symbols), p = 0.5 (erf(c BT (x + 0.5)) - erf(c BT (x - 0.5))), c = pi sqrt(2 / ln 2).
static double gfsk_ref(const std::vector<int> &a, double T, double bt, double df, double sp, double t)
{
    const double c = M_PI * sqrt(2.0 / log(2.0));
    double x = t / T, sum = 0;
    int n = (int)a.size();
    int k0 = (int)floor(x) - 4, k1 = (int)floor(x) + 4;
    for (int k = k0; k <= k1; k++) {
        int v = a[k < 0 ? 0 : (k >= n ? n - 1 : k)];
        double y = x - k - 0.5;
        sum += v * 0.5 * (erf(c * bt * (y + 0.5)) - erf(c * bt * (y - 0.5)));
    }
    return df + sp * sum;
}

static double max_step(const Seq &s, int a, int b)
{
    double m = 0;
    for (int i = a + 1; i <= b; i++) m = fmax(m, fabs((double)s.f[i] - s.f[i - 1]));
    return m;
}

static void dump(const char *dir, const char *name, const Seq &s, int a, int b)
{
    if (!dir) return;
    std::string p = std::string(dir) + "/" + name;
    FILE *fp = fopen(p.c_str(), "wb");
    if (!fp) { printf("cannot write %s\n", p.c_str()); return; }
    fwrite(&s.f[a], sizeof(float), (size_t)(b - a + 1), fp);
    fclose(fp);
    printf("wrote %s (%d ticks)\n", p.c_str(), b - a + 1);
}

// ---------------------------------------------------------------------------------------------------- FT8 / FT4
static void test_ft_gfsk(int mode, int nsym_ticks, int count, double T, double bt, double sp, int delay_ms,
                         const char *dumpdir, const char *dumpname)
{
    digi_init(nullptr);
    { digi_cfg_t c = cfg(1); digi_init(&c); }
    digi_set_mode(mode);
    digi_cmd_V(250); digi_cmd_W(14999); digi_cmd_Y(1000);
    std::string msg = random_symbols(count, mode == DIGI_MODE_FT8 ? 8 : 4, 1234u + mode);
    digi_cmd_Z(msg.c_str());
    Seq s;
    s.run(50);                                             // idle first: nothing happens
    check(s.rf_count() == 0 && !s.out.key, "%s idle before X1", digi_mode_name(mode));
    digi_cmd_X(1);
    s.run_until_idle(20 * FS);
    int a = s.rf_first(), b = s.rf_last();
    int expect_start = 50 + 1 + RELAY + delay_ms * FS / 1000 + 1;
    check(abs(a - expect_start) <= 2, "%s RF start tick %d, expected ~%d", digi_mode_name(mode), a, expect_start);
    check(b - a + 1 == count * nsym_ticks, "%s RF length %d ticks, expected %d", digi_mode_name(mode), b - a + 1,
          count * nsym_ticks);
    digi_status_t st; digi_get_status(&st);
    check(st.first_symbol == 0 && st.symbols_sent == (uint32_t)count, "%s first %d sent %u", digi_mode_name(mode),
          st.first_symbol, st.symbols_sent);
    check(!st.digi_tx_on, "%s DigiTxOn cleared at the end (Nano)", digi_mode_name(mode));
    std::vector<int> av; for (char c : msg) av.push_back(c - '0');
    double maxerr = 0, maxmid = 0;
    for (int i = a; i <= b; i++) {
        double t = (double)(i - a) / FS;
        double e = fabs(s.f[i] - gfsk_ref(av, T, bt, 1000.0, sp, t));
        maxerr = fmax(maxerr, e);
        int k = (i - a) / nsym_ticks, u = (i - a) % nsym_ticks;
        if (u == nsym_ticks / 2) maxmid = fmax(maxmid, fabs(s.f[i] - (1000.0 + av[k] * sp)));
    }
    check(maxerr < 0.01, "%s GFSK max error vs WSJT-X pulse sum %.5f Hz", digi_mode_name(mode), maxerr);
    check(maxmid < 0.01 + (mode == DIGI_MODE_FT4 ? 0.3 : 0), "%s mid-symbol tone error %.5f Hz", digi_mode_name(mode), maxmid);
    double ms = max_step(s, a, b);
    printf("%s GFSK: start %d, %d ticks, max error %.2e Hz, mid-symbol %.2e Hz, max step/tick %.3f Hz\n",
           digi_mode_name(mode), a, b - a + 1, maxerr, maxmid, ms);
    check(fabs(s.f.back() - s.f[b]) < 1e-6, "%s frequency held after the end", digi_mode_name(mode));
    dump(dumpdir, dumpname, s, a, b);
}

static void test_ft8_nano_shape(const char *dumpdir)
{
    digi_cfg_t c = cfg(0); digi_init(&c);
    digi_set_mode(DIGI_MODE_FT8);
    digi_cmd_V(0); digi_cmd_W(14999); digi_cmd_Y(500);
    std::string msg = random_symbols(79, 8, 77u);
    digi_cmd_Z(msg.c_str());
    Seq s; digi_cmd_X(1); s.run_until_idle(20 * FS);
    int a = s.rf_first(), b = s.rf_last();
    check(b - a + 1 == 79 * 3200, "Nano shape FT8 length %d", b - a + 1);
    // Nano gaussianTransition(): 256 steps of 64 us, started 8 ms (integer maths) before the boundary
    double vals[256]; int n = 0;
    static const uint16_t L[33] = {0,518,705,948,1264,1667,2177,2815,3602,4565,5726,7110,8741,10638,12817,15288,
        18052,21102,24421,27978,31732,35630,39604,43582,47477,51203,54668,57782,60461,62631,64228,65206,65535};
    for (int g = 1; g <= 32; g++) { uint16_t inc = (uint16_t)((L[g] - L[g - 1]) / 8);
        for (int h = 1; h <= 8; h++) vals[n++] = (uint16_t)(L[g - 1] + inc * h) / 65536.0; }
    double maxerr = 0;
    for (int k = 1; k < 79; k++) {
        int d0 = msg[k - 1] - '0', d1 = msg[k] - '0';
        for (int x = -200; x < 200; x++) {
            double tau_us = x * 50.0 + 8000.0;
            double v = tau_us < 0 ? 0 : (tau_us >= 256 * 64 ? 1 : vals[(int)(tau_us / 64)]);
            double ref = 500 + 6.25 * (d0 + (d1 - d0) * v);
            maxerr = fmax(maxerr, fabs(s.f[a + k * 3200 + x] - ref));
        }
    }
    check(maxerr < 1e-3, "Nano shape vs Nano transition steps: %.2e Hz", maxerr);
    printf("FT8 Nano shape: max error vs Nano steps %.2e Hz, max step/tick %.3f Hz\n", maxerr, max_step(s, a, b));
    dump(dumpdir, "ft8_nano.f32", s, a, b);
}

static void test_ft_adjust_and_tune(void)
{
    digi_cfg_t c = cfg(1); digi_init(&c);
    // FT4 late start: W5000 -> TimeLeft 3540 <= 5040 -> first symbol 105 - 3540/48 = 32
    digi_set_mode(DIGI_MODE_FT4);
    digi_cmd_V(250); digi_cmd_W(5000); digi_cmd_Y(1200);
    digi_cmd_Z(random_symbols(105, 4, 9u).c_str());
    { Seq s; digi_cmd_X(1); s.run_until_idle(20 * FS);
      digi_status_t st; digi_get_status(&st);
      check(st.first_symbol == 32, "FT4 AdjustStart truncation: first %d, expected 32", st.first_symbol);
      check(s.rf_count() == (105 - 32) * 960, "FT4 truncated length %d", s.rf_count());
      int expect = 1 + RELAY + 125 * FS / 1000 + 1;
      check(abs(s.rf_first() - expect) <= 2, "FT4 start delay is half of {V}: %d vs %d", s.rf_first(), expect); }
    // FT4 too late: W3000 -> TimeLeft 1540 < 2020 -> nothing sent
    digi_cmd_W(3000);
    { Seq s; digi_cmd_X(1); s.run_until_idle(5 * FS);
      check(s.rf_count() == 0 && !s.out.key, "FT4 kill time: no RF (%d)", s.rf_count()); }
    // FT8 Tune: all zeros, {X0} at 5 s -> runs to the first boundary after the 14999 ms period end
    digi_set_mode(DIGI_MODE_FT8);
    digi_cmd_V(0); digi_cmd_W(14999); digi_cmd_Y(1500);
    digi_cmd_Z(std::string(79, '0').c_str());
    { Seq s; digi_cmd_X(1); s.run(5 * FS); digi_cmd_X(0); s.run_until_idle(20 * FS);
      int n = s.rf_count();
      check(n == 15040 * FS / 1000, "FT8 Tune ends at the first boundary after the period: %d ticks (%.3f s)", n,
            (double)n / FS);
      double dev = 0; for (int i = s.rf_first(); i <= s.rf_last(); i++) dev = fmax(dev, fabs(s.f[i] - 1500.0));
      check(dev < 1e-6, "FT8 Tune steady at the offset (%.2e)", dev); }
    // FT8 {W0} mid-message: stop at the end of the current symbol
    digi_cmd_Z(random_symbols(79, 8, 5u).c_str());
    digi_cmd_W(14999);
    { Seq s; digi_cmd_X(1); s.run(RELAY + 2 + 3200 * 10 + 1000); digi_cmd_W(0); s.run_until_idle(20 * FS);
      check(s.rf_count() == 11 * 3200, "FT8 {W0}: %d ticks, expected 11 symbols", s.rf_count()); }
    // FT8 {X0} mid-message (period not over): the Nano keeps sending to the end of the message
    digi_cmd_W(14999);
    { Seq s; digi_cmd_X(1); s.run(RELAY + 2 + 3200 * 10); digi_cmd_X(0); s.run_until_idle(20 * FS);
      check(s.rf_count() == 79 * 3200, "FT8 {X0} keeps sending to the end (Nano): %d", s.rf_count()); }
    // {D} / trip: stops at once
    { Seq s; digi_cmd_X(1); s.run(RELAY + 2 + 3200 * 3); s.requested = false; s.run(FS);
      digi_status_t st; digi_get_status(&st);
      check(s.rf_count() <= 3 * 3200 + 2 && !st.digi_tx_on && st.state == 0, "FT8 {D} stops at once (%d)", s.rf_count()); }
}

static void test_ft_restart(const char *dumpdir)
{
    digi_cfg_t c = cfg(1); digi_init(&c);
    digi_set_mode(DIGI_MODE_FT8);
    digi_cmd_V(0); digi_cmd_W(14999); digi_cmd_Y(1000);
    std::string m1 = random_symbols(79, 8, 21u), m2 = random_symbols(79, 8, 22u);
    digi_cmd_Z(m1.c_str());
    Seq s; digi_cmd_X(1);
    s.run(RELAY + 2 + 2 * FS + 777);                      // 2 s into the message, mid-symbol
    digi_cmd_Z(m2.c_str());
    s.run(3 * FS + 123);
    digi_cmd_Y(1100);                                      // offset change: another restart
    s.run_until_idle(20 * FS);
    digi_status_t st; digi_get_status(&st);
    check(st.restarts == 2, "FT8 restarts %u, expected 2", st.restarts);
    double ms = max_step(s, s.rf_first(), s.rf_last());
    check(ms < 0.2, "FT8 restarts are smooth: max step/tick %.3f Hz", ms);
    // after the first restart the tones are m2's, starting at AdjustStart()+1 = 1
    int a = s.rf_first();
    int restart_tick = (RELAY + 2 + 2 * FS + 777) - a;
    int u = restart_tick % 3200, W = st.half_window_ticks;
    int b1 = restart_tick + W;                             // new boundary: W ticks after the change
    (void)u;
    double e = fabs(s.f[a + b1 + 1600] - (1000 + 6.25 * (m2[1] - '0')));
    check(e < 0.01, "FT8 after restart: first new symbol is m2[1] (err %.4f)", e);
    double e2 = fabs(s.f[a + b1 + 3200 + 1600] - (1000 + 6.25 * (m2[2] - '0')));
    check(e2 < 0.01, "FT8 after restart: second new symbol is m2[2] (err %.4f)", e2);
    printf("FT8 restarts: %u, max step/tick %.3f Hz, %d ticks of RF\n", st.restarts, ms, s.rf_count());
    dump(dumpdir, "ft8_restart.f32", s, s.rf_first(), s.rf_last());
}

// ---------------------------------------------------------------------------------------------------- RTTY
static const char *ltrs = "\0E\nA SIU\rDRJNFCKTZLWHYPQOBG\x01MXV\x02";
static const char *figs = "\0003\n- \"87\r$4',!:(5\")2#6019?&\x01./;\x02";

static std::string rtty_decode(const Seq &s, std::vector<int> *codes_out)
{
    // sample each bit at its centre; bit length 22002 us, stop 44004 us
    int a = s.rf_first(), b = s.rf_last();
    double t_us = 0; std::string out; bool fig = false;
    auto at = [&](double us) { int i = a + (int)(us / 50.0); return i <= b ? s.f[i] : 0.0f; };
    while (a + (int)((t_us + 11001) / 50.0) <= b) {
        if (at(t_us + 11001) > 0) { t_us += 50; continue; }      // waiting for a start bit (space)
        int code = 0;
        for (int k = 0; k < 5; k++) if (at(t_us + 22002.0 * (k + 1) + 11001) > 0) code |= 1 << k;
        bool stop_ok = at(t_us + 22002.0 * 6 + 11001) > 0 && at(t_us + 22002.0 * 6 + 33003) > 0;
        if (!stop_ok) out += '#';
        if (codes_out) codes_out->push_back(code);
        if (code == 27) fig = true;
        else if (code == 31) fig = false;
        else { char ch = (fig ? figs : ltrs)[code]; if (code == 8) ch = '\r'; if (code == 2) ch = '\n'; if (code == 4) ch = ' ';
               if (ch) out += ch; }
        t_us += 22002.0 * 6 + 44004.0;
    }
    return out;
}

static void test_rtty(void)
{
    digi_cfg_t c = cfg(1); digi_init(&c);
    digi_set_mode(DIGI_MODE_RTTY);
    digi_cmd_V(250);
    digi_cmd_Z("cq test de g4ahn 599 73");
    digi_cmd_Z("2ND LINE");
    Seq s; digi_cmd_X(1); s.run_until_idle(30 * FS);
    std::vector<int> codes;
    std::string txt = rtty_decode(s, &codes);
    check(codes.size() >= 6 && codes[0] == 27 && codes[1] == 31 && codes[2] == 27 && codes[3] == 31 && codes[4] == 8 &&
          codes[5] == 2, "RTTY preamble FIGS LTRS FIGS LTRS CR LF");
    std::string want = "\r\nCQ TEST DE G4AHN 599 73 2ND LINE\r\n";
    check(txt == want, "RTTY decodes back: got [%s]", txt.c_str());
    int a = s.rf_first(), b = s.rf_last();
    for (int i = a; i <= b; i++) if (fabs(fabs(s.f[i]) - 85.0f) > 1e-6) { check(false, "RTTY tones are +/-85 Hz"); break; }
    check(s.f[b] == 85.0f, "RTTY ends on mark");
    // timing: the first start bit (space) lasts one bit (22002 us = 440.04 ticks)
    int n0 = 0; while (a + n0 <= b && s.f[a + n0] < 0) n0++;
    check(abs(n0 - 440) <= 1, "RTTY first bit %d ticks", n0);
    printf("RTTY: %zu codes, %d ticks of RF, text [%s]\n", codes.size(), s.rf_count(), "CQ TEST DE G4AHN 599 73 / 2ND LINE");
    // {X0} part way: current character finishes, then CR LF (and LTRS if needed)
    digi_cmd_Z("ABCDEFGHIJKLMNOPQRSTUVWXYZ");
    Seq s2; digi_cmd_X(1); s2.run(1 + RELAY + 5000 + 3 * FS); digi_cmd_X(0); s2.run_until_idle(30 * FS);
    std::string t2 = rtty_decode(s2, nullptr);
    check(t2.size() > 4 && t2.substr(t2.size() - 2) == "\r\n" && t2.size() < 26, "RTTY {X0}: stops early with CR LF [%s]",
          t2.c_str());
    // empty queue: no RF at all
    Seq s3; digi_cmd_X(1); s3.run_until_idle(5 * FS);
    check(s3.rf_count() == 0 && !s3.out.key, "RTTY with nothing queued: no RF");
}

static int rtty_len_hard = 0;
static void test_rtty_shaped(float bt, const char *dumpdir, const char *name)
{
    digi_cfg_t c = cfg(1, bt); digi_init(&c);
    digi_set_mode(DIGI_MODE_RTTY);
    digi_cmd_V(0);
    digi_cmd_Z("cq test de g4ahn 599 73");
    digi_cmd_Z("2ND LINE");
    Seq s; digi_cmd_X(1); s.run_until_idle(30 * FS);
    std::string txt = rtty_decode(s, nullptr);
    check(txt == "\r\nCQ TEST DE G4AHN 599 73 2ND LINE\r\n", "RTTY BT %.1f decodes back: got [%s]", bt, txt.c_str());
    int a = s.rf_first(), b = s.rf_last();
    double sigma = sqrt(log(2.0)) / (2 * M_PI * bt) * 0.022002 * FS;          // ticks
    double lim = 170.0 / (sigma * sqrt(2 * M_PI));
    double ms = max_step(s, a, b);
    check(ms <= lim * 1.02, "RTTY BT %.1f smooth: max step %.3f Hz/tick, limit %.3f", bt, ms, lim);
    // bit centres sit on the tones; the whole thing is as long as the hard-keyed version of the same text
    double dmax = 0; double t_us = 0;
    for (int k = 0; k < 40; k++) { int i = a + (int)((t_us + 11001) / 50.0); dmax = fmax(dmax, fabs(fabs(s.f[i]) - 85.0)); t_us += 22002; }
    check(dmax < 0.05, "RTTY BT %.1f bit centres on +/-85 Hz (%.4f)", bt, dmax);
    check(fabs(s.f[b] - 85.0f) < 1e-3, "RTTY BT %.1f ends on mark", bt);
    check(abs(s.rf_count() - rtty_len_hard) <= 1, "RTTY BT %.1f same length as hard keying (%d vs %d)", bt, s.rf_count(), rtty_len_hard);
    printf("RTTY BT %.1f: %d ticks of RF, max step %.3f Hz/tick (limit %.3f), bit-centre error %.2e Hz\n", bt, s.rf_count(), ms, lim, dmax);
    dump(dumpdir, name, s, a, b);
}

static void test_rtty_hard_len(const char *dumpdir)
{
    digi_cfg_t c = cfg(1, 0.0f); digi_init(&c);
    digi_set_mode(DIGI_MODE_RTTY);
    digi_cmd_V(0);
    digi_cmd_Z("cq test de g4ahn 599 73");
    digi_cmd_Z("2ND LINE");
    Seq s; digi_cmd_X(1); s.run_until_idle(30 * FS);
    rtty_len_hard = s.rf_count();
    dump(dumpdir, "rtty_hard.f32", s, s.rf_first(), s.rf_last());
}

// ---------------------------------------------------------------------------------------------------- JTTY
static std::string hexframe(unsigned seed) { std::string s; srand(seed); for (int i = 0; i < 23; i++) s += "0123456789ABCDEF"[rand() % 16]; return s; }

static void test_jtty(void)
{
    digi_cfg_t c = cfg(1); digi_init(&c);
    digi_set_mode(DIGI_MODE_JTTY);
    digi_cmd_V(250); digi_cmd_Y(800);
    std::string f1 = hexframe(1), f2 = hexframe(2);
    digi_cmd_J(f1.c_str());
    Seq s; digi_cmd_X(1);
    s.run(RELAY + 2 + 640 * 20);
    digi_cmd_J(f2.c_str());                                // appended during TX (gapless chaining)
    s.run_until_idle(20 * FS);
    int a = s.rf_first();
    check(abs(a - (1 + RELAY + 1)) <= 2, "JTTY starts without the {V} delay (%d)", a);
    check(s.rf_count() == 118 * 640, "JTTY two frames chained: %d ticks, expected %d", s.rf_count(), 118 * 640);
    static const int sync[13] = {0,2,2,3,0,0,3,2,1,3,1,2,0};
    std::vector<int> tones;
    for (int fr = 0; fr < 2; fr++) {
        for (int k = 0; k < 13; k++) tones.push_back(sync[k]);
        const std::string &h = fr ? f2 : f1;
        for (char ch : h) { int v = (ch <= '9') ? ch - '0' : ch - 'A' + 10; tones.push_back(v & 3); tones.push_back(v >> 2); }
    }
    double maxe = 0;
    for (int k = 0; k < 118; k++) maxe = fmax(maxe, fabs(s.f[a + k * 640 + 320] - (800 + 31.25 * tones[k])));
    check(maxe < 0.01, "JTTY sync + data tones at symbol centres (err %.4f)", maxe);
    // steepest GFSK slope: 3 tones x 31.25 Hz / (sigma sqrt(2 pi)), sigma = sqrt(ln2)/(2 pi 2) x 32 ms = 42.4 ticks
    double jmax = 3 * 31.25 / (sqrt(log(2.0)) / (4 * M_PI) * 0.032 * FS * sqrt(2 * M_PI));
    double jst = max_step(s, a, s.rf_last());
    check(jst <= jmax * 1.01, "JTTY smooth: max step %.3f Hz/tick, GFSK limit %.3f", jst, jmax);
    digi_status_t st; digi_get_status(&st);
    check(st.symbol_count == 0, "JTTY message cleared at the end (Nano jttyReset)");
    // tune
    digi_cmd_J("T");
    Seq s2; digi_cmd_X(1); s2.run(FS); digi_cmd_X(0); s2.run_until_idle(5 * FS);
    double dev = 0; for (int i = s2.rf_first(); i <= s2.rf_last(); i++) dev = fmax(dev, fabs(s2.f[i] - 800));
    check(s2.rf_count() > FS - 200 && s2.rf_count() < FS + 10 && dev < 1e-6, "JTTY {JT} tune until {X0} (%d ticks)", s2.rf_count());
    // bad payload rejected, partial frame not published
    digi_cmd_J("12345");
    digi_get_status(&st);
    check(st.symbol_count == 0, "JTTY partial frame not published");
    // {X0} aborts at once
    digi_cmd_J(f1.c_str());
    Seq s3; digi_cmd_X(1); s3.run(RELAY + 2 + 640 * 5); digi_cmd_X(0); s3.run_until_idle(5 * FS);
    check(s3.rf_count() <= 640 * 5 + 2, "JTTY {X0} aborts at once (%d)", s3.rf_count());
    printf("JTTY: chained 118 symbols, tone error %.2e Hz\n", maxe);
}

static void test_ft2_and_modes(void)
{
    digi_cfg_t c = cfg(1); digi_init(&c);
    digi_set_mode(DIGI_MODE_FT2);
    digi_cmd_V(250); digi_cmd_W(14999); digi_cmd_Y(1000);
    digi_cmd_Z(random_symbols(105, 4, 3u).c_str());
    Seq s; digi_cmd_X(1); s.run_until_idle(10 * FS);
    check(s.rf_count() == 105 * 480, "FT2 105 x 24 ms (%d)", s.rf_count());
    check(abs(s.rf_first() - (1 + RELAY + 1)) <= 2, "FT2 no start delay (%d)", s.rf_first());
    // wrong-length list for the mode -> Nano error list (FT8 "ERROR G4AHN" 79 chars, zeros after)
    digi_set_mode(DIGI_MODE_FT4);
    digi_cmd_Z(random_symbols(79, 8, 4u).c_str());
    digi_status_t st; digi_get_status(&st);
    check(!st.tune, "FT4 with a 79-symbol list uses the Nano error list (not Tune)");
    // {X1} ignored outside digital modes; mode change aborts
    digi_set_mode(32);
    digi_cmd_X(1); digi_get_status(&st);
    check(!st.digi_tx_on, "{X1} ignored in USB");
    digi_set_mode(DIGI_MODE_FT8);
    digi_cmd_W(14999); digi_cmd_Z(random_symbols(79, 8, 6u).c_str());
    Seq s2; digi_cmd_X(1); s2.run(RELAY + 2 + 3200 * 2); digi_set_mode(DIGI_MODE_FT4); s2.run(FS);
    digi_get_status(&st);
    check(st.state == 0 && !st.digi_tx_on && s2.rf_count() <= 3200 * 2 + 2, "mode change aborts the transmission");
}

int main(int argc, char **argv)
{
    const char *dumpdir = argc > 1 ? argv[1] : nullptr;
    test_ft_gfsk(DIGI_MODE_FT8, 3200, 79, 0.160, 2.0, 6.25, 250, dumpdir, "ft8_gfsk.f32");
    test_ft_gfsk(DIGI_MODE_FT4, 960, 105, 0.048, 1.0, 12000.0 / 576.0, 125, dumpdir, "ft4_gfsk.f32");
    test_ft8_nano_shape(dumpdir);
    test_ft_adjust_and_tune();
    test_ft_restart(dumpdir);
    test_rtty();
    test_rtty_hard_len(dumpdir);
    test_rtty_shaped(2.0f, dumpdir, "rtty_bt2.f32");
    test_rtty_shaped(1.0f, dumpdir, "rtty_bt1.f32");
    test_jtty();
    test_ft2_and_modes();
    printf("%d checks, %d failed\n", checks, fails);
    return fails ? 1 : 0;
}
