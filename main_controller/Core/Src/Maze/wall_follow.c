#include "wall_follow.h"
#include "control_config.h"
#include <math.h>

volatile uint8_t  wf_side;
volatile float    wf_error_mm;
volatile float    wf_tilt_deg;
volatile float    wf_drift_deg;
volatile uint32_t wf_switches;

static uint8_t active_side = WALL_FOLLOW_NONE;


void WallFollow_Reset(void)
{
    active_side  = WALL_FOLLOW_NONE;
    wf_side      = WALL_FOLLOW_NONE;
    wf_error_mm  = 0.0f;
    wf_tilt_deg  = 0.0f;
    wf_drift_deg = 0.0f;
}


static uint8_t usable(const ToF_Measurement_t *m)
{
    return (m->valid && m->distance_mm != TOF_DISTANCE_INVALID &&
            m->distance_mm <= WALL_SIDE_THRESHOLD_MM) ? 1U : 0U;
}


float WallFollow_Update(const ToF_Measurement_t m[TOF_SENSOR_COUNT])
{
    uint8_t left_ok  = usable(&m[TOF_LEFT]);
    uint8_t right_ok = usable(&m[TOF_RIGHT]);

    /* Stick with the wall already being followed for as long as it is usable.
     * Swapping sides mid-corridor puts a step into the error signal and kicks
     * the robot, so the preference order only decides which one to ADOPT, not
     * whether to keep the current one. */
    uint8_t want;

    if (active_side == WALL_FOLLOW_LEFT && left_ok)        want = WALL_FOLLOW_LEFT;
    else if (active_side == WALL_FOLLOW_RIGHT && right_ok) want = WALL_FOLLOW_RIGHT;
    else if (left_ok)                                      want = WALL_FOLLOW_LEFT;
    else if (right_ok)                                     want = WALL_FOLLOW_RIGHT;
    else                                                   want = WALL_FOLLOW_NONE;

    if (want != active_side) {
        if (want != WALL_FOLLOW_NONE && active_side != WALL_FOLLOW_NONE) {
            wf_switches++;
        }
        active_side = want;
    }

    wf_side = active_side;

    if (active_side == WALL_FOLLOW_NONE) {
        /* No reference. Hold the heading target open-loop rather than
         * inventing a correction from a reading that means nothing. */
        wf_error_mm = 0.0f;
        wf_tilt_deg = 0.0f;
        return 0.0f;
    }

    float measured;
    float setpoint;
    float sign;

    if (active_side == WALL_FOLLOW_LEFT) {
        measured = (float)m[TOF_LEFT].distance_mm;
        setpoint = WALL_FOLLOW_SETPOINT_LEFT_MM;
        /* Reading below setpoint means too close to the LEFT wall, so the
         * robot must steer right, which is negative (clockwise) yaw. */
        sign = 1.0f;
    }
    else {
        measured = (float)m[TOF_RIGHT].distance_mm;
        setpoint = WALL_FOLLOW_SETPOINT_RIGHT_MM;
        /* Mirrored: too close to the RIGHT wall means steer left. */
        sign = -1.0f;
    }

    wf_error_mm = measured - setpoint;

    float tilt = sign * WALL_FOLLOW_KP_DEG_PER_MM * wf_error_mm;

    if (tilt >  WALL_FOLLOW_MAX_TILT_DEG) tilt =  WALL_FOLLOW_MAX_TILT_DEG;
    if (tilt < -WALL_FOLLOW_MAX_TILT_DEG) tilt = -WALL_FOLLOW_MAX_TILT_DEG;

    wf_tilt_deg = tilt;

    /* Drift bleed. A tilt that persists is not a position error the robot is
     * still correcting -- it is the heading estimate being wrong, because a
     * centred robot needs no tilt to stay centred. Move the heading target
     * toward it slowly enough that the lateral loop always wins in the short
     * term and this only picks up the standing component. */
    wf_drift_deg += WALL_FOLLOW_DRIFT_BLEED * tilt * CONTROL_SAMPLE_TIME_S;

    return tilt;
}


float WallFollow_GetDriftDeg(void)
{
    return wf_drift_deg;
}
