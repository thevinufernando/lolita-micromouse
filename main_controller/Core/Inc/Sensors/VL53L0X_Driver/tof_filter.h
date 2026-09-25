#ifndef TOF_FILTER_H
#define TOF_FILTER_H

#include <stdint.h>
#include <stddef.h> /* NULL -- reaches the host build via stdio.h, but not the
                     * bare-metal one, so include it explicitly. */

/*
 * ============================================================================
 *              NOISE FILTER FOR VL53L0X RANGE READINGS
 * ============================================================================
 *
 * PURPOSE
 * -------
 * Raw VL53L0X readings jitter. Measured on this robot: a wall at a true 80 mm
 * reads 83-90 mm, i.e. roughly +-3 mm of spread around a biased mean. That
 * jitter is not a defect to be fixed by better wiring -- it is inherent to how
 * the sensor counts returned photons, and it grows with distance, with ambient
 * light, and with darker surfaces.
 *
 * This module smooths that jitter. It does NOT fix bias -- see the note at the
 * bottom, because conflating the two leads to filters that are tuned forever
 * and never work.
 *
 * ----------------------------------------------------------------------------
 * WHY THIS PARTICULAR FILTER
 * ----------------------------------------------------------------------------
 * The obvious choice, a moving average, is the wrong one here. Its problem is
 * LAG: with an N-sample window the output trails the true distance by about
 * N/2 samples. On a wall-following robot that lag is fed straight into a
 * steering controller, which then reacts to where the wall WAS. That is a
 * textbook route to oscillation, and it gets worse exactly when you lengthen
 * the window to reduce noise.
 *
 * The VL53L0X also produces two statistically different kinds of bad sample:
 *
 *   1. Small, dense jitter around the true value -- every reading, always.
 *   2. Occasional wild outliers, tens of mm off, when a measurement catches a
 *      bad reflection or a partially-occluded target. These are rare but
 *      large, and an averaging filter SMEARS one into many output samples
 *      instead of rejecting it.
 *
 * So the filter is two stages, each aimed at one failure:
 *
 *   Stage 1: MEDIAN of the last TOF_FILTER_MEDIAN_WINDOW raw samples.
 *            A median is immune to a minority of arbitrarily-wrong values --
 *            one outlier in three samples cannot move it at all. This kills
 *            case 2 before it ever reaches the averaging stage.
 *
 *   Stage 2: EXPONENTIAL MOVING AVERAGE over the median output.
 *            Smooths case 1. An EMA is used rather than a boxcar average
 *            because it needs one stored value instead of a window, and its
 *            response is a smooth exponential decay rather than a sample that
 *            abruptly falls out of a window N steps later.
 *
 * A 3-sample median costs 3 comparisons and adds only one sample of delay,
 * which is why the window is deliberately kept short.
 *
 * ----------------------------------------------------------------------------
 * THE JUMP DETECTOR (this is the part that matters for maze solving)
 * ----------------------------------------------------------------------------
 * Any smoothing filter assumes the underlying signal is roughly constant. In a
 * maze that assumption breaks at the single most important moment: the instant
 * a side wall ends and the reading legitimately jumps from ~80 mm to ~250 mm.
 *
 * That jump is SIGNAL, not noise. An EMA would ramp across it over many
 * samples, so for that whole ramp the robot believes in a wall that is no
 * longer there -- which is precisely the reading a maze solver uses to decide
 * whether an opening exists.
 *
 * So the filter watches for a step larger than TOF_FILTER_JUMP_THRESHOLD_MM
 * and, when it sees one, RESETS to the new value instead of easing toward it.
 * Response to a real wall transition is then immediate, while ordinary jitter
 * (far below the threshold) is still smoothed. The threshold must sit well
 * above the noise spread and well below the smallest real transition; with
 * ~7 mm of spread and maze transitions of 100 mm or more, that window is wide
 * and the choice is not delicate.
 *
 * ----------------------------------------------------------------------------
 * WHAT THIS FILTER CANNOT DO
 * ----------------------------------------------------------------------------
 * It cannot remove BIAS. If a sensor reads 85 mm for a true 80 mm, every
 * sample is high, the median of high values is high, and their average is
 * high. Filtering a biased signal yields a beautifully stable wrong number.
 *
 * Bias is corrected by the per-sensor offsets in control_config.h
 * (TOF_OFFSET_*_MM), applied before this filter runs. Measure them with
 * TEST_TOF_SINGLE against a ruler. Do not attempt to compensate a constant
 * error by tuning filter constants -- that is the wrong knob and it will
 * silently make the dynamic response worse.
 * ============================================================================
 */

/* Samples held for the median stage. MUST be odd (a median needs a middle
 * element) and is asserted as such in the .c. 3 is the deliberate choice:
 * it rejects a lone outlier while adding just one sample of delay. */
#define TOF_FILTER_MEDIAN_WINDOW 3U

typedef struct {
  /* Median stage: ring buffer of the most recent raw samples. */
  uint16_t window[TOF_FILTER_MEDIAN_WINDOW];
  uint8_t window_index; /* next slot to overwrite                     */
  uint8_t sample_count; /* saturates at the window size; <window = priming */

  /* EMA stage. Held as float so repeated blending does not accumulate the
   * rounding error that integer maths would introduce at small alpha. */
  float ema;
  uint8_t initialised; /* 0 until the first accepted sample seeds the EMA */

  /* Diagnostics, for live-watch during bring-up. */
  uint16_t last_raw;    /* most recent input                          */
  uint16_t last_output; /* most recent output                         */
  uint32_t jump_count;  /* step resets; should track real wall changes */
} ToF_Filter_t;

/* Reset to the empty state. Safe to call at any time; the next sample seeds
 * the filter as if it were fresh. */
void ToF_Filter_Reset(ToF_Filter_t *filter);

/* Feed one raw sample (mm) and get the filtered distance (mm).
 *
 * While priming (fewer than TOF_FILTER_MEDIAN_WINDOW samples seen) the output
 * tracks the input closely rather than ramping up from zero, so a fresh filter
 * does not report a bogus short distance -- which on a wall-follower would
 * read as an imminent collision. */
uint16_t ToF_Filter_Update(ToF_Filter_t *filter, uint16_t raw_mm);

/* Last output without feeding a new sample. Returns 0 if never updated. */
uint16_t ToF_Filter_GetLast(const ToF_Filter_t *filter);

/* Notify the filter that the measurement stream was interrupted -- an invalid
 * reading, or a sensor that dropped out.
 *
 * This is NOT the same as Reset(). It discards the history (which is now
 * separated from the present by an unknown gap, so blending across it would
 * be meaningless) while leaving the filter ready to re-seed cleanly on the
 * next valid sample. Call it whenever a read does not return TOF_OK. */
void ToF_Filter_Invalidate(ToF_Filter_t *filter);

#endif /* TOF_FILTER_H */
