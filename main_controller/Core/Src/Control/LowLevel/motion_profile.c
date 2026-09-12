#include "motion_profile.h"
#include <math.h>

void MotionProfile_Init(MotionProfile_t *p, float total, float v_max, float accel)
{
    /* Starting from rest is the ordinary case and the only one the turn
     * controller ever wants, so it stays a named entry point rather than every
     * caller passing a zero. Sharing the implementation is deliberate: a
     * mistake in the general form shows up in the existing rest-to-rest tests
     * immediately, instead of only in the arena. */
    (void)MotionProfile_InitFrom(p, total, 0.0f, v_max, accel);
}


uint8_t MotionProfile_InitFrom(MotionProfile_t *p, float total, float v0,
                               float v_max, float accel)
{
    if (p == 0) return 0U;

    p->sign     = (total < 0.0f) ? -1.0f : 1.0f;
    p->distance = fabsf(total);
    p->accel    = accel;
    p->v_start  = 0.0f;
    p->v_peak   = 0.0f;
    p->t_ramp   = 0.0f;
    p->t_cruise = 0.0f;
    p->t_decel  = 0.0f;
    p->t_total  = 0.0f;

    /* Degenerate requests collapse to a zero-length profile rather than
     * producing NaNs or an infinite duration. Position() then reports the
     * target immediately, so a caller that does not special-case it simply
     * completes at once. */
    if (!(p->distance > 0.0f) || !(v_max > 0.0f) || !(accel > 0.0f)) {
        p->accel = 0.0f;
        return 1U;
    }

    /* Taken in the direction of travel. A v0 pointing the other way is treated
     * as rest: a trapezoid cannot describe reversing first, and pretending
     * otherwise would produce a profile whose velocity never matches the one
     * the caller actually has. */
    if (v0 * total > 0.0f) {
        p->v_start = fabsf(v0);
    }

    const float v_start  = p->v_start;
    const float d_stop   = (v_start * v_start) / (2.0f * accel);
    uint8_t     feasible = 1U;

    if (p->distance < d_stop) {
        /* Not enough room to come to rest. Build the hardest stop available
         * and report the shortfall: the profile stays self-consistent and
         * simply lands beyond what was asked, which is the honest outcome and
         * far better than a reference that reverses to make the numbers work.
         */
        p->distance = d_stop;
        feasible    = 0U;
    }

    if (v_start >= v_max) {
        /* Already at or above the limit, so there is nothing to accelerate.
         * Hold and then brake. */
        const float d_down = (v_start * v_start) / (2.0f * accel);

        p->v_peak   = v_start;
        p->t_ramp   = 0.0f;
        p->t_cruise = (p->distance - d_down) / v_start;
        p->t_decel  = v_start / accel;
    }
    else {
        /* Distance spent climbing to v_max and then braking from it. */
        const float d_up   = (v_max * v_max - v_start * v_start) / (2.0f * accel);
        const float d_down = (v_max * v_max) / (2.0f * accel);

        if (d_up + d_down <= p->distance) {
            /* Long enough to reach the speed limit: trapezoid. */
            p->v_peak   = v_max;
            p->t_ramp   = (v_max - v_start) / accel;
            p->t_cruise = (p->distance - d_up - d_down) / v_max;
            p->t_decel  = v_max / accel;
        }
        else {
            /* Too short: still accelerating when braking has to begin, so the
             * profile peaks early and never cruises. The peak is where the two
             * ramps meet --
             *
             *     (v^2 - v_start^2) / 2a  +  v^2 / 2a  =  distance
             *
             * -- and it can never come out below v_start, because that would
             * require distance < d_stop, which the branch above already took.
             */
            p->v_peak   = sqrtf((2.0f * accel * p->distance
                                 + v_start * v_start) * 0.5f);
            p->t_ramp   = (p->v_peak - v_start) / accel;
            p->t_cruise = 0.0f;
            p->t_decel  = p->v_peak / accel;
        }
    }

    p->t_total = p->t_ramp + p->t_cruise + p->t_decel;

    return feasible;
}


float MotionProfile_Position(const MotionProfile_t *p, float t)
{
    if (p == 0) return 0.0f;

    if (!(p->t_total > 0.0f)) {
        return p->sign * p->distance;
    }

    if (t <= 0.0f)       return 0.0f;
    if (t >= p->t_total) return p->sign * p->distance;

    /* Distance covered by the acceleration phase. Carries the v_start term,
     * which is the only difference from a move that begins at rest. */
    float ramp_distance = p->v_start * p->t_ramp
                          + 0.5f * p->accel * p->t_ramp * p->t_ramp;
    float s;

    if (t < p->t_ramp) {
        s = p->v_start * t + 0.5f * p->accel * t * t;
    }
    else if (t < p->t_ramp + p->t_cruise) {
        s = ramp_distance + p->v_peak * (t - p->t_ramp);
    }
    else {
        /* Measured BACK FROM THE END rather than forward from the cruise, and
         * that is load-bearing: the move always finishes at rest, so this form
         * lands exactly on `distance` instead of accumulating rounding error
         * across three segments. It is also why the deceleration needs no
         * v_start term -- it is the same ramp whatever the move began at. */
        float remaining = p->t_total - t;
        s = p->distance - 0.5f * p->accel * remaining * remaining;
    }

    return p->sign * s;
}


float MotionProfile_Velocity(const MotionProfile_t *p, float t)
{
    if (p == 0) return 0.0f;

    if (!(p->t_total > 0.0f)) return 0.0f;

    /* At and before the start the profile is moving at v_start, not stopped.
     * Returning zero here would make the feedforward lie on the very first
     * cycle of a retargeted move, which is the cycle it matters most. For a
     * move that begins at rest this is unchanged. */
    if (t <= 0.0f)       return p->sign * p->v_start;
    if (t >= p->t_total) return 0.0f;

    float v;

    if (t < p->t_ramp) {
        v = p->v_start + p->accel * t;
    }
    else if (t < p->t_ramp + p->t_cruise) {
        v = p->v_peak;
    }
    else {
        v = p->accel * (p->t_total - t);
    }

    return p->sign * v;
}


float MotionProfile_Acceleration(const MotionProfile_t *p, float t)
{
    if (p == 0) return 0.0f;

    if (!(p->t_total > 0.0f)) return 0.0f;
    if (t <= 0.0f || t >= p->t_total) return 0.0f;

    if (t < p->t_ramp)               return p->sign * p->accel;
    if (t < p->t_ramp + p->t_cruise) return 0.0f;

    return -p->sign * p->accel;
}


float MotionProfile_Duration(const MotionProfile_t *p)
{
    return (p != 0) ? p->t_total : 0.0f;
}
