#include "wall_follow.h"
#include "control_config.h"
#include <math.h>

volatile uint8_t  wf_side;
volatile float    wf_error_mm;
volatile float    wf_tilt_deg;
volatile float    wf_drift_deg;
volatile uint32_t wf_switches;
volatile float    wf_integral;

static uint8_t active_side = WALL_FOLLOW_NONE;

/* What the map knows about the two cells this move touches. Zeroed means
 * neither cell has an opinion, which vetoes nothing. */
static WallFollowCells_t s_cells;

/* Travel at the previous update, for working out whether the robot is actually
 * moving. Negative means "no previous sample this move". */
static float s_last_travel_cm = -1.0f;

/* Integral of the lateral error, in degrees. It is NOT part of the tilt: it is
 * added to the heading TARGET, outside the tilt clamp, and reaches the
 * controller through WallFollow_GetDriftDeg(). wf_drift_deg holds it.
 *
 * It belongs to the ROBOT, not to a move -- see WALL_FOLLOW_KI_DEG_PER_MM_S
 * and the ceiling argument at WALL_FOLLOW_KI_LIMIT_DEG. */


/* Per-move reset. Clears what belongs to a move and DELIBERATELY KEEPS what
 * belongs to the robot.
 *
 * wf_drift_deg is a slow learner of a standing asymmetry. Clearing it here is
 * why it read exactly 0.000 in every log ever taken: it was wiped at the start
 * of all 26 moves of a run and never given long enough to become anything. A
 * term reset before it can act is not a term. */
void WallFollow_Reset(void)
{
    active_side  = WALL_FOLLOW_NONE;
    wf_side      = WALL_FOLLOW_NONE;
    wf_error_mm  = 0.0f;
    wf_tilt_deg  = 0.0f;

    /* The context belongs to a move. Keeping it would let a move that never
     * set one inherit the previous move's cells, and after a pivot those
     * describe walls that are no longer on the sides they used to be. */
    WallFollow_SetCells(0);

    /* Travel restarts at zero every move, so a value carried over from the
     * last one would make the first update of this one look like a large jump
     * backwards. */
    s_last_travel_cm = -1.0f;
}


void WallFollow_SetCells(const WallFollowCells_t *cells)
{
    if (cells) {
        s_cells = *cells;
    } else {
        s_cells.left_this  = 0U; s_cells.right_this = 0U;
        s_cells.left_next  = 0U; s_cells.right_next = 0U;
        s_cells.this_known = 0U; s_cells.next_known = 0U;
        s_cells.cross_cm   = 0.0f;
    }
}


/* Does the map contradict a reading on this side, right now?
 *
 * Only ever WITHHOLDS. A cell the map has not read has no opinion, and a wall
 * the map believes in but the sensor cannot see is not conjured into one --
 * this returns 1 only when the applicable cell has genuinely been surveyed and
 * genuinely recorded no wall there. Everything else is 0.
 *
 * Which cell is "applicable" is decided by how far into the move the robot is,
 * because the sensors lead the axle by TOF_SIDE_AHEAD_CM and cross into the
 * next cell long before the body does. */
static uint8_t vetoed(uint8_t want_left, float travelled_cm)
{
    const uint8_t looking_ahead = (travelled_cm >= s_cells.cross_cm) ? 1U : 0U;

    const uint8_t known = looking_ahead ? s_cells.next_known
                                        : s_cells.this_known;

    if (!known) {
        return 0U;
    }

    const uint8_t wall = want_left
        ? (looking_ahead ? s_cells.left_next  : s_cells.left_this)
        : (looking_ahead ? s_cells.right_next : s_cells.right_this);

    return wall ? 0U : 1U;
}


/* Full reset, including everything learned about the robot. Call once at the
 * start of a run, never between moves. */
void WallFollow_ResetBias(void)
{
    WallFollow_Reset();
    wf_integral  = 0.0f;
    wf_drift_deg = 0.0f;
}


/* How much to trust a single-wall reading, 1.0 down to
 * WALL_FOLLOW_FAR_CONF_FLOOR.
 *
 * A reading at or inside its setpoint is certain: only a wall returns a close
 * signal, so there is nothing to doubt. Past the setpoint, confidence falls
 * linearly and reaches the floor at WALL_FOLLOW_USABLE_MAX_MM, which is where
 * the reading stops being used at all. The two ends of the ramp are therefore
 * the two things already known to be true, and the middle is interpolation
 * rather than invention. */
static float far_confidence(float reading_mm, float setpoint_mm)
{
    const float far_mm = reading_mm - setpoint_mm;

    if (far_mm <= 0.0f) {
        return 1.0f;
    }

    const float span_mm = (float)WALL_FOLLOW_USABLE_MAX_MM - setpoint_mm;

    if (span_mm <= 0.0f) {
        return WALL_FOLLOW_FAR_CONF_FLOOR;   /* degenerate config */
    }

    float t = far_mm / span_mm;

    if (t > 1.0f) t = 1.0f;

    return 1.0f - t * (1.0f - WALL_FOLLOW_FAR_CONF_FLOOR);
}


static uint8_t usable(const ToF_Measurement_t *m)
{
    return (m->valid && m->distance_mm != TOF_DISTANCE_INVALID &&
            m->distance_mm <= WALL_FOLLOW_USABLE_MAX_MM) ? 1U : 0U;
}


float WallFollow_Update(const ToF_Measurement_t m[TOF_SENSOR_COUNT],
                        float dt_s, float travelled_cm)
{
    /* IS THE ROBOT ACTUALLY MOVING? Sampled here, at the top, because the
     * no-reference path below returns early -- and a travel sample skipped for
     * a stretch of cells would come back as one enormous step divided by a
     * single interval, which reads as a robot sprinting. Whether there is a
     * wall to follow has nothing to do with whether the wheels are turning.
     *
     * Lateral authority comes from leaning while travelling forward, so with
     * no forward motion there is no correction to be had, and the fact that it
     * did not arrive says nothing about the robot's asymmetry. A wedged run
     * drove the integral to -7.66 of a +/-8 limit over two seconds of grinding
     * at 3.8 cm/s. */
    uint8_t moving = 1U;

    if (s_last_travel_cm >= 0.0f && dt_s > 0.0f) {
        const float rate = fabsf(travelled_cm - s_last_travel_cm) / dt_s;

        moving = (rate >= WALL_FOLLOW_MIN_TRAVEL_CMS) ? 1U : 0U;
    }

    s_last_travel_cm = travelled_cm;

    uint8_t left_ok  = usable(&m[TOF_LEFT])  && !vetoed(1U, travelled_cm);
    uint8_t right_ok = usable(&m[TOF_RIGHT]) && !vetoed(0U, travelled_cm);

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

    /* Both readings in range is not enough -- they must also be CONSISTENT
     * with each other. Two walls of the same cell sum to its width whatever
     * the robot is doing between them, so a sum far from WALL_FOLLOW_SPAN_MM
     * means at least one of them is not the wall it is being taken for. When
     * that happens, fall through to the single-wall path, which will pick the
     * CLOSER reading -- a close return is unambiguous, a distant one is not. */
    uint8_t pair_ok = 0U;

    if (left_ok && right_ok) {
        const float span = (float)m[TOF_LEFT].distance_mm
                         + (float)m[TOF_RIGHT].distance_mm;

        pair_ok = (fabsf(span - WALL_FOLLOW_SPAN_MM) <= WALL_FOLLOW_SPAN_TOL_MM)
                  ? 1U : 0U;

        if (!pair_ok) {
            /* Keep only the nearer sensor as a reference. */
            if (m[TOF_LEFT].distance_mm <= m[TOF_RIGHT].distance_mm) right_ok = 0U;
            else                                                     left_ok  = 0U;
        }
    }

    if (pair_ok)                                           want = WALL_FOLLOW_BOTH;
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
        const float ease = WALL_FOLLOW_TILT_SLEW_DPS * dt_s;

        wf_error_mm = 0.0f;

        if (wf_tilt_deg >  ease) wf_tilt_deg -= ease;
        else if (wf_tilt_deg < -ease) wf_tilt_deg += ease;
        else wf_tilt_deg = 0.0f;

        return wf_tilt_deg;
    }

    const float left_mm  = (float)m[TOF_LEFT].distance_mm;
    const float right_mm = (float)m[TOF_RIGHT].distance_mm;

    /* HOW MUCH THE REFERENCE IS WORTH, from 1.0 (certain) downwards.
     *
     * Two walls are always 1.0: the difference cancels the common-mode error
     * and resolves the ambiguity outright. A single wall reading SHORT is also
     * 1.0, because a close return can only be a wall. A single wall reading
     * LONG is the one case that cannot tell an off-centre robot from a wall
     * that has ended, and how suspicious it deserves to be depends on HOW
     * long -- which is what the old rule got wrong. See below. */
    float conf = 1.0f;

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
        conf = far_confidence(left_mm, WALL_FOLLOW_SETPOINT_LEFT_MM);
    }
    else {
        /* Mirrored: too far from the RIGHT wall means move right. */
        wf_error_mm = -(right_mm - WALL_FOLLOW_SETPOINT_RIGHT_MM);
        conf = far_confidence(right_mm, WALL_FOLLOW_SETPOINT_RIGHT_MM);
    }

    /* A TAPER, NOT A CLIFF. The gain and the clamp both scale with how much
     * the reference is worth, so the loop acts on a reading in proportion to
     * how likely it is to mean what it says.
     *
     * The old rule was binary on the SIGN of the error: any long reading got a
     * fifth of the gain and a 2.5 degree cap. It cost a run. The robot carried
     * a 14 mm error for a full second against a right wall reading 76 mm --
     * twelve millimetres long, when an opening reads 240 and the usable gate
     * already rejects anything past 95 -- and the rule throttled a correction
     * that was entirely correct. The proof arrived a moment later: when the
     * left wall came into range it said the same thing, too close on the left
     * by 14 where the right had said too far by 12. The loop had declined for
     * a second to act on a reading that was right, and ended the cell 28 mm
     * off centre, from where the next turn jammed. */
    const float kp  = WALL_FOLLOW_KP_DEG_PER_MM  * conf;
    const float cap = WALL_FOLLOW_MAX_TILT_DEG   * conf;

    /* PROPORTIONAL ONLY. The integral is deliberately absent from this sum --
     * it goes to the heading target instead, so the clamp below bounds how
     * hard the robot may lean to fix a POSITION error and nothing else. */
    float tilt      = kp * wf_error_mm;
    uint8_t clamped = 0U;

    if (tilt >  cap) { tilt =  cap; clamped = 1U; }
    if (tilt < -cap) { tilt = -cap; clamped = 1U; }

    /* ANTI-WINDUP, and it is the ordinary kind: stop integrating while the
     * proportional term is saturated.
     *
     * Without it this term stopped being a bias estimator. A single-wall cell
     * with the robot 23 mm off centre asks the P term for 11.5 degrees against
     * a 10 degree clamp, and the integral -- seeing that same 23 mm -- moved
     * 3.3 degrees in that one cell. Over five such cells it drove itself to
     * WALL_FOLLOW_KI_LIMIT_DEG and pinned there, and because it is added to the
     * heading target it then dragged the robot 7 degrees off the maze. The
     * "growing heading error" in the second half of that run was this term's
     * own output.
     *
     * A large lateral error is a POSITION error and belongs entirely to the
     * proportional term. Only what P cannot remove is evidence of a standing
     * bias, and while P is pinned there is no such evidence to be had -- the
     * loop is already doing everything it can.
     *
     * THERE ARE THREE GATES HERE AND THEY GUARD DIFFERENT THINGS. This one is
     * about the loop asking for everything it can; `moving` is about the robot
     * not answering; `conf` is about the reference not being worth believing.
     * A term that learns a property of the ROBOT has no business updating
     * under any of the three.
     *
     * The OTHER gate, on confidence, is a different argument and both are
     * needed. The proportional term may act on a doubtful reference in
     * proportion to how doubtful it is, because it forgets immediately if the
     * reference turns out to be wrong. An integrator does not forget -- it
     * would bake the guess in permanently -- so it gets a threshold rather
     * than a taper, and stops learning entirely once the reference is not
     * clearly worth trusting. */
    if (conf >= WALL_FOLLOW_TRUST_CONF && !clamped && moving) {
        wf_drift_deg += WALL_FOLLOW_KI_DEG_PER_MM_S
                        * wf_error_mm * dt_s;

        if (wf_drift_deg >  WALL_FOLLOW_KI_LIMIT_DEG)
            wf_drift_deg =  WALL_FOLLOW_KI_LIMIT_DEG;
        if (wf_drift_deg < -WALL_FOLLOW_KI_LIMIT_DEG)
            wf_drift_deg = -WALL_FOLLOW_KI_LIMIT_DEG;
    }

    wf_integral = wf_drift_deg;

    /* SLEW LIMIT. The value above is where the heading target should go; this
     * decides how fast it is allowed to get there.
     *
     * Without it the demand can step by the full clamp range in one cycle --
     * when a wall ends, when the active side changes, or simply when a move
     * begins with the robot already off-centre. A step in a heading setpoint
     * asks the robot to rotate as hard as it can, which is never what a
     * centring correction wants, and it pins the inner loop's output for as
     * long as it takes. Ramping instead keeps the inner loop in its linear
     * region, where a cascade is worth having. */
    /* !! THIS FUNCTION IS NOT CALLED EVERY CONTROL CYCLE !! The straight-line
     * controller calls it once per ToF sweep, every STRAIGHT_TOF_DIVIDER
     * cycles, because sampling faster than the sensor produces only re-reads a
     * stale measurement. Using the control period here would make the slew
     * STRAIGHT_TOF_DIVIDER times slower than the constant says. */
    const float max_step = WALL_FOLLOW_TILT_SLEW_DPS * dt_s;
    float       step     = tilt - wf_tilt_deg;

    if (step >  max_step) step =  max_step;
    if (step < -max_step) step = -max_step;

    tilt = wf_tilt_deg + step;

    wf_tilt_deg = tilt;

    return tilt;
}


float WallFollow_GetDriftDeg(void)
{
    return wf_drift_deg;
}
