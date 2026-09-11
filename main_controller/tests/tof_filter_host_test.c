/* Host-side verification of the VL53L0X noise filter.
 *
 * The numbers here are taken from a real bench measurement on this robot: a
 * wall at a true 80 mm read back as 83-90 mm. That is ~7 mm of spread sitting
 * on top of a ~+6 mm bias, and the two are checked separately because they
 * have separate fixes (filter vs. TOF_OFFSET_*_MM).
 *
 * Runs on the host, no hardware needed. Exit code 0 = pass. */
#include "tof_filter.h"
#include <stdio.h>
#include <math.h>

static int failures = 0;

static void check(const char *name, float got, float expect, float tol)
{
    float err = fabsf(got - expect);
    int ok = (err <= tol);
    if (!ok) failures++;
    printf("  [%s] %-40s got=%8.2f expect=%8.2f tol=%.2f\n",
           ok ? "PASS" : "FAIL", name, got, expect, tol);
}

static void check_true(const char *name, int cond, const char *detail)
{
    if (!cond) failures++;
    printf("  [%s] %-40s %s\n", cond ? "PASS" : "FAIL", name, detail);
}

/* Deterministic pseudo-random so a failure is always reproducible. */
static unsigned long rng = 2024;
static float urand(void)
{
    rng = rng * 1103515245UL + 12345UL;
    return (float)((rng >> 16) & 0x7FFF) / 32767.0f;
}

/* Sample from the measured band [lo, hi] mm. */
static uint16_t noisy(int lo, int hi)
{
    return (uint16_t)(lo + (int)(urand() * (float)(hi - lo + 1)));
}

/* Population standard deviation of a series. */
static float stddev(const float *v, int n)
{
    float mean = 0.0f, acc = 0.0f;
    for (int i = 0; i < n; i++) mean += v[i];
    mean /= (float)n;
    for (int i = 0; i < n; i++) acc += (v[i] - mean) * (v[i] - mean);
    return sqrtf(acc / (float)n);
}

int main(void)
{
    printf("\n===== VL53L0X noise filter host tests =====\n");

    /* ---- TEST 1: the headline claim -- less spread out than in ---- */
    printf("\nTEST 1: measured 83-90mm jitter is smoothed\n");
    {
        ToF_Filter_t f;
        ToF_Filter_Reset(&f);

        float raw[400], filt[400];
        int n = 0;

        /* Prime, so the comparison is of steady-state behaviour. */
        for (int i = 0; i < 20; i++) ToF_Filter_Update(&f, noisy(83, 90));

        for (int i = 0; i < 400; i++) {
            uint16_t r = noisy(83, 90);
            uint16_t o = ToF_Filter_Update(&f, r);
            raw[n] = (float)r;
            filt[n] = (float)o;
            n++;
        }

        float sd_raw = stddev(raw, n);
        float sd_filt = stddev(filt, n);
        printf("       raw sigma=%.2fmm  filtered sigma=%.2fmm  (%.1fx better)\n",
               sd_raw, sd_filt, sd_raw / sd_filt);

        check_true("filtered spread is smaller", sd_filt < sd_raw * 0.5f,
                   "at least 2x reduction");

        /* The mean must survive: a filter that smooths by dragging the value
         * off-centre has not helped, it has just moved the error. */
        float mean_raw = 0.0f, mean_filt = 0.0f;
        for (int i = 0; i < n; i++) { mean_raw += raw[i]; mean_filt += filt[i]; }
        mean_raw /= (float)n; mean_filt /= (float)n;
        check("mean preserved (mm)", mean_filt, mean_raw, 1.0f);
    }

    /* ---- TEST 2: no lie about bias ---- */
    printf("\nTEST 2: filter does NOT remove bias (that is the offset's job)\n");
    {
        ToF_Filter_t f;
        ToF_Filter_Reset(&f);

        /* Every sample high by ~6mm, as the real sensor is. */
        uint16_t last = 0;
        for (int i = 0; i < 200; i++) last = ToF_Filter_Update(&f, noisy(83, 90));

        /* Settles near the biased mean (~86), NOT the true 80. This test
         * exists to stop anyone "fixing" bias by tuning the filter. */
        check("settles at biased mean, not truth", (float)last, 86.0f, 2.5f);
        check_true("still far from true 80mm", fabsf((float)last - 80.0f) > 3.0f,
                   "bias needs TOF_OFFSET_*_MM");
    }

    /* ---- TEST 3: the case that matters for maze solving ---- */
    printf("\nTEST 3: a real wall transition is NOT smoothed into a ramp\n");
    {
        ToF_Filter_t f;
        ToF_Filter_Reset(&f);

        for (int i = 0; i < 30; i++) ToF_Filter_Update(&f, noisy(83, 90));
        uint32_t jumps_before = f.jump_count;

        /* Side wall ends: 86mm -> 250mm in one sample. */
        uint16_t after = ToF_Filter_Update(&f, 250);

        check("responds immediately to the step", (float)after, 250.0f, 15.0f);
        check_true("jump detector fired", f.jump_count == jumps_before + 1,
                   "exactly one jump recorded");
    }

    /* ---- TEST 4: jitter must not trip the detector ---- */
    printf("\nTEST 4: ordinary jitter does not trip the jump detector\n");
    {
        ToF_Filter_t f;
        ToF_Filter_Reset(&f);

        for (int i = 0; i < 10; i++) ToF_Filter_Update(&f, noisy(83, 90));
        uint32_t jumps_before = f.jump_count;

        for (int i = 0; i < 500; i++) ToF_Filter_Update(&f, noisy(83, 90));

        /* If this fails, TOF_FILTER_JUMP_THRESHOLD_MM is below the noise
         * floor and the filter is resetting instead of smoothing. */
        check_true("no jumps while target is stationary",
                   f.jump_count == jumps_before, "jump_count unchanged");
    }

    /* ---- TEST 5: single wild outlier is rejected ---- */
    printf("\nTEST 5: a lone outlier is rejected by the median stage\n");
    {
        ToF_Filter_t f;
        ToF_Filter_Reset(&f);

        for (int i = 0; i < 40; i++) ToF_Filter_Update(&f, 86);
        uint16_t before = ToF_Filter_GetLast(&f);

        /* One bad reflection, immediately back to normal. */
        ToF_Filter_Update(&f, 400);
        uint16_t during = ToF_Filter_GetLast(&f);
        ToF_Filter_Update(&f, 86);
        ToF_Filter_Update(&f, 86);
        uint16_t after = ToF_Filter_GetLast(&f);

        printf("       before=%umm  during-spike=%umm  after=%umm\n",
               before, during, after);

        /* An outlier bigger than the jump threshold DOES move the output for
         * exactly one sample -- the detector cannot tell it apart from the
         * first sample of a real transition (see the comment in
         * ToF_Filter_Update). What must hold is that it does not persist:
         * the median refuses to follow, so the very next reading pulls the
         * output back. One sample of blip is the accepted cost of never
         * being blind to a real opening. */
        check_true("outlier does not persist", after < 100,
                   "median pulls it back within two samples");
        check("recovers cleanly", (float)after, 86.0f, 3.0f);
        check_true("pre-spike value was already settled", before == 86,
                   "baseline sanity");
    }

    /* ---- TEST 6: priming must not report a bogus short distance ---- */
    printf("\nTEST 6: a fresh filter does not report a false close wall\n");
    {
        ToF_Filter_t f;
        ToF_Filter_Reset(&f);

        /* First ever sample. A zero-filled window would median to 0mm, which
         * a wall-follower reads as an imminent collision. */
        uint16_t first = ToF_Filter_Update(&f, 200);
        check("first sample tracks input", (float)first, 200.0f, 1.0f);

        uint16_t second = ToF_Filter_Update(&f, 200);
        check_true("still sane on second sample", second > 150,
                   "no dip toward zero");
    }

    /* ---- TEST 7: invalidate clears history but keeps diagnostics ---- */
    printf("\nTEST 7: Invalidate() re-seeds without blending across the gap\n");
    {
        ToF_Filter_t f;
        ToF_Filter_Reset(&f);

        for (int i = 0; i < 40; i++) ToF_Filter_Update(&f, 86);
        uint32_t jumps = f.jump_count;

        ToF_Filter_Invalidate(&f);

        /* Sensor comes back pointing at something far away. Without the
         * invalidate this would blend with the stale 86mm history. */
        uint16_t reseeded = ToF_Filter_Update(&f, 300);
        check("re-seeds from the new value", (float)reseeded, 300.0f, 1.0f);
        check_true("jump_count preserved across invalidate",
                   f.jump_count == jumps, "diagnostic counter is run-scoped");
    }

    /* ---- TEST 8: hostile inputs ---- */
    printf("\nTEST 8: degenerate inputs do not break the filter\n");
    {
        ToF_Filter_t f;
        ToF_Filter_Reset(&f);

        ToF_Filter_Update(&f, 0);
        ToF_Filter_Update(&f, 65534);
        ToF_Filter_Update(&f, 0);
        uint16_t out = ToF_Filter_Update(&f, 100);

        /* The median of the last three (65534, 0, 100) is 100, and the huge
         * value must not have leaked into the EMA on the way through. */
        check("extreme values do not corrupt the output", (float)out, 100.0f,
              5.0f);

        /* NULL must be survivable: the driver calls this on every read. */
        uint16_t passthrough = ToF_Filter_Update(NULL, 123);
        check("NULL filter passes input through", (float)passthrough, 123.0f, 0.0f);
        ToF_Filter_Reset(NULL);
        ToF_Filter_Invalidate(NULL);
        check_true("NULL handling did not crash", 1, "survived");
    }

    printf("\n===== %s (%d failure%s) =====\n\n",
           failures ? "FAILURES PRESENT" : "ALL CHECKS PASSED",
           failures, failures == 1 ? "" : "s");

    return failures ? 1 : 0;
}
