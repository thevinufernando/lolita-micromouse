#ifndef CONTROL_CONFIG_H
#define CONTROL_CONFIG_H

/*
 * ============================================================================
 *                          PID TUNING INTERFACE
 * ============================================================================
 *
 * All tunable gains for the low-level controllers live here. Edit the values
 * below, rebuild, and flash. Straightline and turn controllers are tuned
 * independently.
 *
 * Units:
 *   - Distance PID  : setpoint/measurement in cm, output in motor speed units
 *   - Straight PID  : setpoint/measurement in raw encoder ticks (left - right),
 *                     output is a differential speed correction
 *   - Turn PID      : setpoint/measurement in cm of wheel arc, output in motor
 *                     speed units
 *
 * Motor speed units are 0..255 (mapped to 0..MOTOR_PWM_MAX in the driver).
 *
 * ---------------------------------------------------------------------------
 * SUGGESTED TUNING ORDER
 * ---------------------------------------------------------------------------
 *  1. Set every Ki and Kd to 0.
 *  2. Raise Kp until the robot reaches the target with a small overshoot.
 *  3. Add Kd to damp the overshoot.
 *  4. Add a small Ki only if it consistently stops short of the target.
 *
 * Tune STRAIGHT_DIST_* first (drive straight, ignore heading), then
 * STRAIGHT_HEADING_* (heading hold), then the turn gains separately.
 * ============================================================================
 */

/* ------------------------- Common PID settings --------------------------- */

/* Control loop sample time in seconds (20 ms = 50 Hz) */
#define CONTROL_SAMPLE_TIME_S       0.010f

/* Derivative low-pass filter time constant in seconds.
 * Rule of thumb: keep it a few times larger than the sample time.
 * Must be > 0 or the derivative term is unfiltered. */
#define CONTROL_DERIV_TAU_S         0.020f

/* Maximum motor speed command the controllers may produce (0..255). */
#define CONTROL_MAX_SPEED           200.0f

/* Minimum speed magnitude that still overcomes gearbox stiction.
 * Commands below this (but non-zero) are boosted up to it, so the robot
 * does not stall just short of the target. Set to 0.0f to disable. */
#define CONTROL_MIN_MOVE_SPEED      35.0f


/* ====================== STRAIGHTLINE: DISTANCE PID ======================= */
/* Drives average travelled distance (cm) to the target distance.            */

#define STRAIGHT_DIST_KP            20.0f
#define STRAIGHT_DIST_KI            0.0f
#define STRAIGHT_DIST_KD            0.0f

/* Integrator clamp, in motor speed units */
#define STRAIGHT_DIST_INT_LIMIT     50.0f


/* ====================== STRAIGHTLINE: HEADING PID ======================== */
/* Holds (left_count - right_count) at zero so the robot tracks straight.    */
/* Measurement is in RAW ENCODER TICKS, so these gains are small.            */

#define STRAIGHT_HEADING_KP         0.5f
#define STRAIGHT_HEADING_KI         0.025f
#define STRAIGHT_HEADING_KD         0.00f

/* Steering authority clamp, in motor speed units */
#define STRAIGHT_HEADING_LIMIT      80.0f
#define STRAIGHT_HEADING_INT_LIMIT  20.0f


/* ============================== TURN PID ================================= */
/* Drives the differential wheel arc (cm) to the arc implied by the angle.   */

#define TURN_KP                     18.0f
#define TURN_KI                     0.0f
#define TURN_KD                     2.0f

/* Integrator clamp, in motor speed units */
#define TURN_INT_LIMIT              60.0f


/* ========================= Completion criteria =========================== */

/* How close (cm) counts as "arrived" for straightline moves. */
#define STRAIGHT_TOLERANCE_CM       0.7f

/* How close (cm of wheel arc) counts as "arrived" for turns. */
#define TURN_TOLERANCE_ARC_CM       0.20f

/* The move is only considered complete once the robot has been inside the
 * tolerance band for this many consecutive control cycles. Prevents declaring
 * success while still coasting through the target at speed. */
#define CONTROL_SETTLE_CYCLES       5

/* Safety timeout: abort a move that has not completed within this many ms. */
#define CONTROL_MOVE_TIMEOUT_MS     8000U

#endif /* CONTROL_CONFIG_H */
