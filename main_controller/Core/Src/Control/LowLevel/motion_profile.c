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
    /* Ending at rest is the ordinary case, so it keeps its own name. */
    return MotionProfile_InitFromTo(p, total, v0, 0.0f, v_max, accel);
}


uint8_t MotionProfile_InitFromTo(MotionProfile_t *p, float total, float v0,
                                 float v_end, float v_max, float accel)
{
    if (p == 0) return 0U;

    p->sign     = (total < 0.0f) ? -1.0f : 1.0f;
    p->distance = fabsf(total);
    p->accel    = accel;
    p->v_start  = 0.0f;
    p->v_end    = 0.0f;
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

    /* Both taken in the direction of travel. One pointing the other way is
     * treated as rest: a trapezoid cannot describe reversing, and pretending
     * otherwise would produce a profile whose velocity never matches the one
     * the caller actually has. */
    if (v0    * total > 0.0f) p->v_start = fabsf(v0);
    if (v_end * total > 0.0f) p->v_end   = fabsf(v_end);

    const float vs = p->v_start;
    const float ve = p->v_end;

    /* The distance the SPEED CHANGE alone needs, with no cruise and no wasted
     * motion in between. Below this the move cannot be done at all -- not
     * "done badly", but not done: there is no way to get from vs to ve inside
     * it at this acceleration. */
    const float d_min    = fabsf(vs * vs - ve * ve) / (2.0f * accel);
    uint8_t     feasible = 1U;

    if (p->distance < d_min) {
        /* Build the shortest profile that IS possible and report the
         * shortfall. The reference stays self-consistent and simply lands
         * beyond what was asked, which is far better than one that reverses to
         * make the numbers work. */
        p->distance = d_min;
        feasible    = 0U;
    }

    /* Where the two ramps meet if they meet before v_max:
     *
     *     (v^2 - vs^2) / 2a  +  (v^2 - ve^2) / 2a  =  distance
     *
     * which is never below max(vs, ve) once the distance clears d_min above,
     * so the ramp times below cannot come out negative. */
    float   v_peak  = sqrtf((2.0f * accel * p->distance + vs * vs + ve * ve) * 0.5f);
    uint8_t cruises = 0U;

    if (v_peak > v_max) { v_peak = v_max; cruises = 1U; }   /* room to cruise */

    /* A speed limit below a speed the caller already has is not a limit this
     * generator can impose -- the robot is doing it. Honour the endpoints and
     * let the cruise fall out as zero. */
    if (v_peak < vs) v_peak = vs;
    if (v_peak < ve) v_peak = ve;

    const float d_up   = (v_peak * v_peak - vs * vs) / (2.0f * accel);
    const float d_down = (v_peak * v_peak - ve * ve) / (2.0f * accel);

    p->v_peak   = v_peak;
    p->t_ramp   = (v_peak - vs) / accel;
    p->t_decel  = (v_peak - ve) / accel;

    /* WHEN THE RAMPS MEET BELOW v_max THERE IS NO CRUISE, and that is known
     * from the shape rather than from the arithmetic. Computing it instead
     * leaves a few hundred nanoseconds of "cruise" behind, because v_peak came
     * out of a square root and v_peak^2 does not land back exactly on the
     * distance it was derived from. A triangular profile must report a cruise
     * of exactly zero, not nearly zero. */
    p->t_cruise = cruises ? (p->distance - d_up - d_down) / v_peak : 0.0f;

    if (p->t_cruise < 0.0f) {
        /* Only reachable when v_max is below a speed the caller already has,
         * so the endpoints had to override it above. */
        p->t_cruise = 0.0f;
        feasible    = 0U;
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
         * that is load-bearing: it lands exactly on `distance` instead of
         * accumulating rounding error across three segments. It is also why
         * the deceleration carries v_end rather than v_start -- this ramp is
         * the same whatever the move began at, and depends only on what it is
         * heading for. */
        float remaining = p->t_total - t;
        s = p->distance - (p->v_end * remaining
                           + 0.5f * p->accel * remaining * remaining);
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
    if (t >= p->t_total) return p->sign * p->v_end;

    float v;

    if (t < p->t_ramp) {
        v = p->v_start + p->accel * t;
    }
    else if (t < p->t_ramp + p->t_cruise) {
        v = p->v_peak;
    }
    else {
        v = p->v_end + p->accel * (p->t_total - t);
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
