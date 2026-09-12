#ifndef MOTION_PROFILE_H
#define MOTION_PROFILE_H

#include <stdint.h>

/*
 * ============================================================================
 *                    TRAPEZOIDAL MOTION PROFILE GENERATOR
 * ============================================================================
 *
 * Given a total displacement, a speed limit and an acceleration limit, this
 * produces the reference position and velocity at any time during the move.
 *
 * UNITS ARE WHATEVER YOU PUT IN. Feed it degrees and deg/s for a turn, or cm
 * and cm/s for a straight move. It holds no state about which, which is why
 * the same generator serves turns, straights and later arcs.
 *
 * ---------------------------------------------------------------------------
 * WHY A PROFILE INSTEAD OF A STEP SETPOINT
 * ---------------------------------------------------------------------------
 * Commanding "go to 90 degrees" hands the PID a 90 degree error on cycle one.
 * It saturates, the robot accelerates as hard as it can, and the controller
 * spends the whole move recovering from a demand it could never meet. How the
 * move ends is then decided by how the saturation happens to decay.
 *
 * A profile instead asks for a position the robot can actually be at right
 * now. The error stays small throughout, the PID operates in its linear
 * region, and the deceleration is planned rather than emergent.
 *
 * The second benefit matters more here: the move has a KNOWN DURATION. A
 * tolerance-based move takes however long it takes, which on this robot has
 * meant anywhere from 0.7 to 8 seconds. A profiled move always takes
 * MotionProfile_Duration(), which makes everything downstream schedulable.
 *
 * ---------------------------------------------------------------------------
 * SHAPE
 * ---------------------------------------------------------------------------
 * Accelerate at `accel` to `v_max`, hold, decelerate at `accel` to rest:
 *
 *     v |      ______________
 *       |     /              \
 *       |    /                \
 *       +---/------------------\------> t
 *          t_ramp   t_cruise   t_ramp
 *
 * If the move is too short to reach v_max the profile degenerates to a
 * triangle, peaking at sqrt(accel * distance). That case is handled rather
 * than clamped, because short moves are the common case in a maze.
 *
 * ---------------------------------------------------------------------------
 * A MOVE MAY ALSO END AT SPEED
 * ---------------------------------------------------------------------------
 * The third corner is optional. A profile built with a non-zero `v_end` stops
 * decelerating when it gets there and hands the robot over still moving:
 *
 *     v |      ______________
 *       |     /              \____   v_end
 *       |    /
 *       +---/------------------------> t
 *
 * That is what makes continuous cell-to-cell running possible. A maze move
 * that is going to be followed by another maze move has no business braking
 * to rest in between, and a profile that always ends at zero leaves the
 * caller no way to say so. The ramps are then not symmetric, which is why
 * t_ramp and t_decel have been separate fields since v_start arrived.
 *
 * A trapezoid has a discontinuous acceleration at the corners, which shows up
 * as a jerk. That is acceptable here and an S-curve is the upgrade if the
 * chassis ever complains about it.
 * ============================================================================
 */

typedef struct {
  float sign;     /* +1 or -1; the profile itself is computed on |total| */
  float distance; /* |total|                                            */
  float accel;    /* units/s^2, positive                                */
  float v_start;  /* speed at t = 0; zero for a move that starts at rest */
  float v_end;    /* speed at t_total; zero for a move that ends at rest  */
  float v_peak;   /* actually reached; < v_max when the move is short   */
  float t_ramp;   /* ACCELERATION phase only -- zero when already fast  */
  float t_cruise; /* zero for a triangular profile                      */
  float t_decel;  /* DECELERATION phase; equals t_ramp only when v_start
                   * is zero, which is why the two are separate fields  */
  float t_total;
} MotionProfile_t;

/* Build a profile that starts from REST. `total` is signed; `v_max` and
 * `accel` must be positive. A zero or degenerate request yields a profile of
 * zero duration that reports position `total` and velocity 0 at every time, so
 * callers do not need a special case for it. */
void MotionProfile_Init(MotionProfile_t *p, float total, float v_max, float accel);

/* Build a profile that starts at speed `v0`, and say whether it fits.
 *
 * THIS EXISTS FOR RETARGETING MID-MOVE. The front-wall alignment changes a
 * move's endpoint once it can see the wall, and the new profile has to pick up
 * where the old reference actually is -- at cruise, not at rest. Rebuilding
 * from rest instead drops the velocity feedforward to zero, which commands a
 * brake and a fresh start in the middle of a move the robot is already making.
 *
 * `v0` is signed and is taken in the direction of travel; a `v0` that opposes
 * `total` is treated as zero, because a profile cannot model reversing first.
 *
 * RETURNS 0 WHEN THE MOVE DOES NOT FIT -- when `total` is shorter than
 * v0^2 / (2 * accel), the distance needed just to stop. The profile is then
 * built as the hardest stop available, which overshoots `total` and is
 * self-consistent, so a caller that ignores the return value gets a sane
 * reference rather than a reversal. The right response is usually to decline
 * the retarget. */
uint8_t MotionProfile_InitFrom(MotionProfile_t *p, float total, float v0,
                               float v_max, float accel);

/* The general form: start at `v0`, end at `v_end`, and say whether it fits.
 *
 * THIS EXISTS FOR CHAINED CELL MOVES. Stopping at every cell centre costs a
 * deceleration and an acceleration the robot did not need, and the settle that
 * follows costs more than either. A move that is going to be followed by
 * another move in the same direction should arrive still travelling, and this
 * is the only place that can be expressed.
 *
 * Both speeds are magnitudes taken in the direction of `total`; a negative one
 * is treated as zero. The two ramps are independent, so the shape is a
 * trapezoid with unequal sides, or a triangle when there is no room to reach
 * v_max in between.
 *
 * RETURNS 0 WHEN THE MOVE DOES NOT FIT -- when `total` is shorter than
 * |v0^2 - v_end^2| / (2 * accel), which is the distance the speed change alone
 * requires. The profile is then built over that minimum distance instead, so
 * it stays self-consistent and simply lands beyond what was asked. */
uint8_t MotionProfile_InitFromTo(MotionProfile_t *p, float total, float v0,
                                 float v_end, float v_max, float accel);

/* Reference position at time t, clamped to [0, total] outside the move. */
float MotionProfile_Position(const MotionProfile_t *p, float t);

/* Reference velocity at time t; zero before the start and after the end.
 * This is the FEEDFORWARD term -- the thing that lets the feedback loop stay
 * small instead of doing all the work. */
float MotionProfile_Velocity(const MotionProfile_t *p, float t);

/* Reference acceleration at time t: +accel, 0, or -accel by segment.
 *
 * The SECOND feedforward term, and the one that was missing. A velocity
 * feedforward supplies the command needed to HOLD a rate but nothing to
 * CHANGE one, so with velocity alone the robot lags through both ramps and
 * the integrator winds up covering for it. On this robot that windup was
 * still positive when the move ended, which drove it past the target and then
 * kept driving the wrong way. */
float MotionProfile_Acceleration(const MotionProfile_t *p, float t);

/* Total duration in seconds. Known before the move starts, which is the
 * whole point. */
float MotionProfile_Duration(const MotionProfile_t *p);

#endif /* MOTION_PROFILE_H */
