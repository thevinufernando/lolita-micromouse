#include "tof_filter.h"
#include "control_config.h"

/* A median needs an unambiguous middle element. Catch an even window at
 * compile time rather than silently returning the wrong order statistic. */
_Static_assert((TOF_FILTER_MEDIAN_WINDOW % 2U) == 1U,
               "TOF_FILTER_MEDIAN_WINDOW must be odd");
_Static_assert(TOF_FILTER_MEDIAN_WINDOW >= 3U,
               "TOF_FILTER_MEDIAN_WINDOW must be at least 3 to reject an outlier");

/* Median of the samples currently held.
 *
 * Insertion sort on a copy: the window is 3 elements, so an O(n^2) sort is
 * a handful of comparisons and beats anything cleverer in both code size and
 * actual cycles. The copy exists because the ring buffer must stay in arrival
 * order -- sorting it in place would destroy the history. */
static uint16_t ToF_Filter_Median(const ToF_Filter_t *filter, uint8_t count)
{
    uint16_t sorted[TOF_FILTER_MEDIAN_WINDOW];

    for (uint8_t i = 0; i < count; i++) {
        sorted[i] = filter->window[i];
    }

    for (uint8_t i = 1; i < count; i++) {
        uint16_t key = sorted[i];
        int j = (int)i - 1;

        while (j >= 0 && sorted[j] > key) {
            sorted[j + 1] = sorted[j];
            j--;
        }

        sorted[j + 1] = key;
    }

    return sorted[count / 2U];
}

void ToF_Filter_Reset(ToF_Filter_t *filter)
{
    if (filter == NULL) {
        return;
    }

    for (uint8_t i = 0; i < TOF_FILTER_MEDIAN_WINDOW; i++) {
        filter->window[i] = 0U;
    }

    filter->window_index = 0U;
    filter->sample_count = 0U;
    filter->ema = 0.0f;
    filter->initialised = 0U;
    filter->last_raw = 0U;
    filter->last_output = 0U;
    filter->jump_count = 0U;
}

void ToF_Filter_Invalidate(ToF_Filter_t *filter)
{
    if (filter == NULL) {
        return;
    }

    /* Drop the history but KEEP jump_count: it is a running diagnostic across
     * the whole run, and zeroing it on every dropped reading would make it
     * useless exactly when readings are marginal. The next valid sample
     * re-seeds the filter, which is why initialised is cleared. */
    filter->window_index = 0U;
    filter->sample_count = 0U;
    filter->ema = 0.0f;
    filter->initialised = 0U;
}

uint16_t ToF_Filter_Update(ToF_Filter_t *filter, uint16_t raw_mm)
{
    uint16_t median;
    float delta;

    if (filter == NULL) {
        return raw_mm;
    }

    filter->last_raw = raw_mm;

    /* --- Stage 1: median over the recent window --- */
    filter->window[filter->window_index] = raw_mm;
    filter->window_index =
        (uint8_t)((filter->window_index + 1U) % TOF_FILTER_MEDIAN_WINDOW);

    if (filter->sample_count < TOF_FILTER_MEDIAN_WINDOW) {
        filter->sample_count++;
    }

    /* While priming, take the median of only what has actually arrived. The
     * alternative -- medianing zero-filled slots -- would drag the output
     * toward 0 mm, which a wall-follower reads as an imminent collision. */
    median = ToF_Filter_Median(filter, filter->sample_count);

    /* --- Stage 2: EMA, with a step detector ---
     *
     * The jump test deliberately looks at the RAW sample, not the median.
     *
     * This ordering is load-bearing and was found by the host test. A real
     * wall transition arrives as a single new value against a window still
     * full of old ones -- {86, 86, 250} medians to 86, so the median stage
     * SUPPRESSES the very first sample of a genuine step exactly as if it
     * were an outlier. Testing the median against the threshold therefore
     * misses the transition entirely until two more samples arrive, which is
     * the failure the jump detector exists to prevent.
     *
     * A median cannot tell "one bad reflection" from "first sample of a real
     * change" -- they are the same shape. So the raw value is what gets
     * tested, and confirmation comes from the median: a lone outlier is
     * rejected because the NEXT sample returns to the old level and the
     * median never follows, while a real step is confirmed because
     * subsequent samples stay at the new level.
     *
     * The cost is that a single outlier larger than the threshold causes one
     * sample of overshoot before the median pulls the output back. Test 5
     * covers exactly this. That is the right trade: a transient one-sample
     * blip is far cheaper than being blind to an opening in the maze. */
    if (!filter->initialised) {
        /* Seed from the first sample rather than ramping up from zero. */
        filter->ema = (float)median;
        filter->initialised = 1U;
    } else {
        delta = (float)raw_mm - filter->ema;

        if (delta < 0.0f) {
            delta = -delta;
        }

        if (delta >= (float)TOF_FILTER_JUMP_THRESHOLD_MM) {
            /* A step this large is a real change in the world -- a side wall
             * ending, or an opening appearing -- not jitter. Smoothing across
             * it would report a wall that is no longer there for as long as
             * the ramp lasts, so jump straight to the new value and restart
             * the average from it.
             *
             * Snap to the RAW value: the median is still dominated by
             * pre-transition samples and would land the output between the
             * old and new distances rather than at the new one. */
            filter->ema = (float)raw_mm;
            filter->jump_count++;
        } else {
            filter->ema += TOF_FILTER_EMA_ALPHA * ((float)median - filter->ema);
        }
    }

    /* +0.5 to round rather than truncate. Truncation would bias every output
     * low by an average of half a millimetre -- small, but it is a systematic
     * error introduced by the very code meant to improve accuracy. */
    filter->last_output = (uint16_t)(filter->ema + 0.5f);

    return filter->last_output;
}

uint16_t ToF_Filter_GetLast(const ToF_Filter_t *filter)
{
    if (filter == NULL || !filter->initialised) {
        return 0U;
    }

    return filter->last_output;
}
