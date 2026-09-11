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
            m->distance_mm <= WALL_FOLLOW_USABLE_MAX_MM) ? 1U : 0U;
}


float WallFollow_Update(const ToF_Measurement_t m[TOF_SENSOR_COUNT], float dir)
{
    uint8_t left_ok  = usable(&m[TOF_LEFT]);
    uint8_t right_ok = usable(&m[TOF_RIGHT]);

    /* BOTH WALLS BEAT EITHER ONE, and it is not a small difference.
     *
     * With one wall the robot holds a measured distance from it, so the
     * sensor's ~27 mm close-range over-read has to be baked into the setpoint,
     * and any variation in corridor width lands straight in the error. With
     * two it holds the DIFFERENCE, where the common-mode over-read cancels
     * exactly and the width cancels with it -- only the ~1 mm mismatch between
     * the pair survives. Measured across one run, L + R came to 121-129 mm on
     * every genuine pair, so the difference is the trustworthy quantity and
     * either reading alone is not.
     *
     * The preference order below only decides what to ADOPT. A side already
     * being followed is kept while it stays usable, because swapping puts a
     * step into the error signal. Gaining the second wall is not a swap, so it
     * is always taken. */
    uint8_t want;

    if (left_ok && right_ok)                               want = WALL_FOLLOW_BOTH;
    else if (active_side == WALL_FOLLOW_LEFT && left_ok)   want = WALL_FOLLOW_LEFT;
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
         * inventing a correction from a reading that means nothing -- but
         * SLEW back to zero, because snapping there is the same step this
         * function exists to avoid, just in the other direction. Losing a wall
         * happens at every cell boundary, so it is the common case, not a
         * corner one. */
        const float ease = WALL_FOLLOW_TILT_SLEW_DPS * WALL_FOLLOW_UPDATE_S;

        wf_error_mm = 0.0f;

        if (wf_tilt_deg >  ease) wf_tilt_deg -= ease;
        else if (wf_tilt_deg < -ease) wf_tilt_deg += ease;
        else wf_tilt_deg = 0.0f;

        return wf_tilt_deg;
    }

    const float left_mm  = (float)m[TOF_LEFT].distance_mm;
    const float right_mm = (float)m[TOF_RIGHT].distance_mm;

    /* Error is POSITIVE when the robot must move LEFT, whichever reference is
     * in use, so everything downstream is direction-agnostic. */
    if (active_side == WALL_FOLLOW_BOTH) {
        /* Half the difference is the offset from centre. The setpoint term is
         * just the trim between the two sensors, about half a millimetre. */
        wf_error_mm = ((left_mm - right_mm)
                       - (WALL_FOLLOW_SETPOINT_LEFT_MM - WALL_FOLLOW_SETPOINT_RIGHT_MM))
                      * 0.5f;
    }
    else if (active_side == WALL_FOLLOW_LEFT) {
        /* Reading above setpoint means too far from the LEFT wall, so move
         * left, which is positive (anticlockwise) yaw. */
        wf_error_mm = left_mm - WALL_FOLLOW_SETPOINT_LEFT_MM;
    }
    else {
        /* Mirrored: too far from the RIGHT wall means move right. */
        wf_error_mm = -(right_mm - WALL_FOLLOW_SETPOINT_RIGHT_MM);
    }

    /* !! DIRECTION !! Backwards the loop is non-minimum-phase, because the
     * sensors sit ahead of the wheel axis and swing the wrong way first. It
     * gets its own, much smaller gain and clamp. See control_config.h. */
    const uint8_t reverse   = (dir < 0.0f) ? 1U : 0U;
    const float   kp        = reverse ? WALL_FOLLOW_KP_REVERSE_DEG_PER_MM
                                      : WALL_FOLLOW_KP_DEG_PER_MM;
    const float   max_tilt  = reverse ? WALL_FOLLOW_MAX_TILT_REVERSE_DEG
                                      : WALL_FOLLOW_MAX_TILT_DEG;

    /* The tilt that corrects this error, expressed in the TRAVEL direction.
     * Reversing, a given tilt walks the robot the opposite way, so the demand
     * is negated -- this is the one place that flip belongs, rather than in
     * every caller. */
    float tilt = dir * kp * wf_error_mm;

    if (tilt >  max_tilt) tilt =  max_tilt;
    if (tilt < -max_tilt) tilt = -max_tilt;

    /* SLEW LIMIT. The value above is where the heading target should go; this
     * decides how fast it is allowed to get there.
     *
     * Without it the demand can step by the full clamp range in one cycle --
     * when a wall ends, when the active side changes, or simply when a move
     * begins with the robot already off-centre. A step in a heading setpoint
     * asks the robot to rotate as hard as it can, which is never what a
     * centring correction wants, and it pins the inner loop's output for as
     * long as it takes. Ramping instead keeps the inner loop in its linear
     * region, where a cascade is worth having.
     *
     * Note the SLEWED value is what gets bled into the drift estimate below,
     * not the raw demand: the bleed is meant to pick up the standing tilt the
     * robot is actually holding, and it never holds the un-slewed one. */
    /* !! THIS FUNCTION IS NOT CALLED EVERY CONTROL CYCLE !! The straight-line
     * controller calls it once per ToF sweep, every STRAIGHT_TOF_DIVIDER
     * cycles, because sampling faster than the sensor produces only re-reads a
     * stale measurement. Using the control period here would make the slew
     * STRAIGHT_TOF_DIVIDER times slower than the constant says. */
    const float max_step = WALL_FOLLOW_TILT_SLEW_DPS * WALL_FOLLOW_UPDATE_S;
    float       step     = tilt - wf_tilt_deg;

    if (step >  max_step) step =  max_step;
    if (step < -max_step) step = -max_step;

    tilt = wf_tilt_deg + step;

    wf_tilt_deg = tilt;

    /* Drift bleed. A tilt that persists is not a position error the robot is
     * still correcting -- it is the heading estimate being wrong, because a
     * centred robot needs no tilt to stay centred. Move the heading target
     * toward it slowly enough that the lateral loop always wins in the short
     * term and this only picks up the standing component. */
    wf_drift_deg += WALL_FOLLOW_DRIFT_BLEED * tilt * WALL_FOLLOW_UPDATE_S;

    return tilt;
}


float WallFollow_GetDriftDeg(void)
{
    return wf_drift_deg;
}
