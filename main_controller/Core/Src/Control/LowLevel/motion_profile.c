#include "motion_profile.h"
#include <math.h>

void MotionProfile_Init(MotionProfile_t *p, float total, float v_max, float accel)
{
    if (p == 0) return;

    p->sign     = (total < 0.0f) ? -1.0f : 1.0f;
    p->distance = fabsf(total);
    p->accel    = accel;
    p->v_peak   = 0.0f;
    p->t_ramp   = 0.0f;
    p->t_cruise = 0.0f;
    p->t_total  = 0.0f;

    /* Degenerate requests collapse to a zero-length profile rather than
     * producing NaNs or an infinite duration. Position() then reports the
     * target immediately, so a caller that does not special-case it simply
     * completes at once. */
    if (!(p->distance > 0.0f) || !(v_max > 0.0f) || !(accel > 0.0f)) {
        p->accel = 0.0f;
        return;
    }

    /* Distance consumed by one full ramp to v_max. */
    float ramp_distance = (v_max * v_max) / (2.0f * accel);

    if (2.0f * ramp_distance <= p->distance) {
        /* Long enough to reach the speed limit: trapezoid. */
        p->v_peak   = v_max;
        p->t_ramp   = v_max / accel;
        p->t_cruise = (p->distance - 2.0f * ramp_distance) / v_max;
    }
    else {
        /* Too short: the robot is still accelerating when it has to start
         * braking, so the profile peaks early and never cruises. */
        p->v_peak   = sqrtf(accel * p->distance);
        p->t_ramp   = p->v_peak / accel;
        p->t_cruise = 0.0f;
    }

    p->t_total = 2.0f * p->t_ramp + p->t_cruise;
}


float MotionProfile_Position(const MotionProfile_t *p, float t)
{
    if (p == 0) return 0.0f;

    if (!(p->t_total > 0.0f)) {
        return p->sign * p->distance;
    }

    if (t <= 0.0f)       return 0.0f;
    if (t >= p->t_total) return p->sign * p->distance;

    float ramp_distance = 0.5f * p->accel * p->t_ramp * p->t_ramp;
    float s;

    if (t < p->t_ramp) {
        s = 0.5f * p->accel * t * t;
    }
    else if (t < p->t_ramp + p->t_cruise) {
        s = ramp_distance + p->v_peak * (t - p->t_ramp);
    }
    else {
        /* Mirror of the acceleration ramp, measured back from the end. This
         * is exact by construction, so the profile always lands on `distance`
         * rather than accumulating rounding error across three segments. */
        float remaining = p->t_total - t;
        s = p->distance - 0.5f * p->accel * remaining * remaining;
    }

    return p->sign * s;
}


float MotionProfile_Velocity(const MotionProfile_t *p, float t)
{
    if (p == 0) return 0.0f;

    if (!(p->t_total > 0.0f)) return 0.0f;
    if (t <= 0.0f || t >= p->t_total) return 0.0f;

    float v;

    if (t < p->t_ramp) {
        v = p->accel * t;
    }
    else if (t < p->t_ramp + p->t_cruise) {
        v = p->v_peak;
    }
    else {
        v = p->accel * (p->t_total - t);
    }

    return p->sign * v;
}


float MotionProfile_Duration(const MotionProfile_t *p)
{
    return (p != 0) ? p->t_total : 0.0f;
}
