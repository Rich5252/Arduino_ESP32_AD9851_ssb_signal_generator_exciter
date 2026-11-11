// Host test for slow_adc_conv.cpp (2026-11-04). Build and run on a PC, never in the Arduino sketch folder:
//   g++ -std=c++17 -O1 -Wall -I.. test_slow_adc_conv.cpp ../slow_adc_conv.cpp -o test_slow_adc_conv && ./test_slow_adc_conv
// Checks the port against a near-verbatim copy of the Nano's SendSWR()/calcNTC() maths (double precision, Nano
// 10-bit counts of 5 V), plus the field formatter.

#include "slow_adc_conv.h"
#include <cmath>
#include <cstdio>
#include <cstring>

static int fails = 0, checks = 0;
static void check(bool ok, const char *what)
{
    checks++;
    if (!ok) { fails++; printf("FAIL: %s\n", what); }
}
static bool near(double a, double b, double tol) { return fabs(a - b) <= tol * (1.0 + fabs(b)); }

// ---- Reference: TXlink4_Nano.ino SendSWR()/calcNTC(), inputs as averaged Nano counts ----
static double nano_fwd(double avgFWD)
{
    if (avgFWD < 2) avgFWD = 0;
    else { avgFWD = avgFWD / 1023 * 5 * 0.7071; avgFWD = 34.232 * avgFWD + 2.2884; }
    return pow(avgFWD, 2) / 50;
}
static double nano_ref(double avgREF, unsigned long CurFreq)
{
    bool HiReading = true;
    if (avgREF < 2) avgREF = 0;
    if (avgREF > 0 && avgREF < 100) { avgREF = avgREF * 12.549 * pow(avgREF, -0.555); HiReading = false; }
    avgREF = avgREF / 1023 * 5;
    avgREF = pow(avgREF / 0.0932, 2) / 50;
    if (HiReading && avgREF > 0) avgREF = avgREF * 0.79 + 0.59;
    if (CurFreq > 30000000) avgREF = avgREF / 2;
    return avgREF;
}
static double nano_swr(double f, double r)
{
    f = sqrt(fabs(f)); r = sqrt(fabs(r));
    return (f - r > 0) ? (f + r) / (f - r) : 0;
}
static double nano_ntc(double AI)   // 51k pull-up to 5 V, counts 0..1023
{
    double R = 51000.0 * AI / (1023.0 - AI);
    double T = 1 / ((1 / 298.0) + (1 / 3950.0) * log(R / 50000.0));
    return T - 273;
}

int main()
{
    // Scale 1.0, 5 V "supply": pin volts == Nano volts, so counts = v/5*1023 and every result must match the Nano.
    slow_adc_conv_cfg_t nano_like = { 1.0f, 1.0f, 5.0f, 51000.0f, 50000.0f, 3950.0f };

    for (double v = 0.0; v <= 3.2; v += 0.0137) {
        double counts = v / 5.0 * 1023.0;
        char msg[96];
        snprintf(msg, sizeof msg, "fwd at %.4f V", v);
        check(near(slow_adc_fwd_watts((float)v, &nano_like), nano_fwd(counts), 2e-4), msg);
        snprintf(msg, sizeof msg, "rev at %.4f V, 14 MHz", v);
        check(near(slow_adc_rev_watts((float)v, 14200000u, &nano_like), nano_ref(counts, 14200000ul), 2e-4), msg);
        snprintf(msg, sizeof msg, "rev at %.4f V, 50 MHz", v);
        check(near(slow_adc_rev_watts((float)v, 50313000u, &nano_like), nano_ref(counts, 50313000ul), 2e-4), msg);
        if (v > 0.05 && v < 4.55) {
            snprintf(msg, sizeof msg, "ntc at %.4f V", v);
            check(near(slow_adc_ntc_degc((float)v, &nano_like), nano_ntc(counts), 1e-3), msg);
        }
    }
    const double fw[] = { 0, 0.5, 1.8, 2.0, 10 }, rw[] = { 0, 0.01, 0.08, 2.0, 12 };
    for (double f : fw) for (double r : rw) {
        char msg[64]; snprintf(msg, sizeof msg, "swr f=%.2f r=%.2f", f, r);
        check(near(slow_adc_swr((float)f, (float)r), nano_swr(f, r), 1e-5), msg);
    }

    // ESP32 defaults (config.h): 3.3 V NTC rail, dummy pin voltages -> the values documented there.
    slow_adc_conv_cfg_t esp = { 1.0f, 1.0f, 3.3f, 47000.0f, 50000.0f, 3950.0f };
    slow_adc_reading_t r;
    slow_adc_convert(0.300f, 0.060f, 1.701f, 14200000u, &esp, &r);
    printf("dummy defaults -> fwd %.3f W, rev %.4f W, SWR %.3f, temp %.3f degC\n", r.fwd_w, r.rev_w, r.swr, r.temp_c);
    check(r.fwd_w > 1.6f && r.fwd_w < 2.0f, "dummy fwd about 1.8 W");
    check(r.rev_w > 0.05f && r.rev_w < 0.12f, "dummy rev about 0.08 W");
    check(r.swr > 1.3f && r.swr < 1.8f, "dummy SWR about 1.5");
    check(fabsf(r.temp_c - 25.0f) < 0.6f, "dummy temp about 25 degC");
    check(slow_adc_ntc_degc(0.0f, &esp) == -99.0f, "ntc shorted -> -99");
    check(slow_adc_ntc_degc(3.3f, &esp) == -99.0f, "ntc open -> -99");
    check(slow_adc_ntc_degc(3.10f, &esp) == -99.0f, "ntc open as the ADC sees it (3.1 V cap) -> -99");
    check(slow_adc_ntc_degc(3.00f, &esp) > -25.0f && slow_adc_ntc_degc(3.00f, &esp) < -10.0f, "3.00 V still a reading (cold)");

    // Formatter
    char buf[64];
    slow_adc_reading_t f1 = { 1.8f, 0.081f, 1.54f, 25.0f };
    int n = slow_adc_format_fields(buf, sizeof buf, &f1);
    printf("format: %s\n", buf);
    check(strcmp(buf, "{F1.8}{R0.08}{S1.5}{T25.00}") == 0 && n == (int)strlen(buf), "format basic");
    slow_adc_reading_t f2 = { NAN, -1.0f, 1e9f, -300.0f };
    slow_adc_format_fields(buf, sizeof buf, &f2);
    check(strcmp(buf, "{F0.0}{R0.00}{S999.9}{T-99.00}") == 0, "format clamps / NaN");
    slow_adc_reading_t f3 = { 9999.9f, 9999.99f, 999.9f, 999.0f };
    n = slow_adc_format_fields(buf, sizeof buf, &f3);
    printf("format worst case: %s (%d chars)\n", buf, n);
    check(n > 0 && n < 48, "worst case fits in 48");
    check(slow_adc_format_fields(buf, 10, &f1) == 0 && buf[0] == 0, "too-small buffer -> empty, 0");

    // SWR trip rules (2026-11-11): Nano limits 3.0 / 1 W, REV saturation at 3000 mV
    auto T = [](float f, float r, float mv) {
        return slow_adc_swr_trip_check(f, r, slow_adc_swr(f, r), mv, 3.0f, 1.0f, 3000.0f);
    };
    check(T(81, 1.35f, 400) == SWR_TRIP_NONE, "trip: normal");
    check(T(100, 30, 2900) == SWR_TRIP_HIGH, "trip: SWR 3.4");
    check(T(100, 24, 2900) == SWR_TRIP_NONE, "trip: SWR 2.9 below limit");
    check(T(10, 12, 1500) == SWR_TRIP_REV_GE_FWD, "trip: reflected >= forward (Nano SWR would be 0)");
    check(T(0.5f, 0.8f, 500) == SWR_TRIP_NONE, "trip: below forward minimum");
    check(T(100, 18, 3050) == SWR_TRIP_REV_SATURATED, "trip: REV at ADC full scale");
    check(T(0, 0, 3100) == SWR_TRIP_REV_SATURATED, "trip: REV saturated with no forward reading");

    // temperature trip rules
    check(slow_adc_temp_trip_check(40.0f, 54.0f, true) == TEMP_TRIP_NONE, "temp: 40 < 54");
    check(slow_adc_temp_trip_check(54.0f, 54.0f, true) == TEMP_TRIP_OVER, "temp: 54 >= 54 (Nano uses >=)");
    check(slow_adc_temp_trip_check(-99.0f, 54.0f, true) == TEMP_TRIP_SENSOR_FAULT, "temp: open/short sensor trips");
    check(slow_adc_temp_trip_check(-99.0f, 54.0f, false) == TEMP_TRIP_NONE, "temp: sensor fault ignored when disabled");
    check(slow_adc_temp_trip_check(-20.0f, 54.0f, true) == TEMP_TRIP_NONE, "temp: genuine cold is not a fault");

    printf("%d checks, %d failed\n", checks, fails);
    return fails ? 1 : 0;
}
