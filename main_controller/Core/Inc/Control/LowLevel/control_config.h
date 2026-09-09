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
#define CONTROL_DERIV_TAU_S         0.010f

/* Maximum motor speed command the controllers may produce (0..255). */
#define CONTROL_MAX_SPEED           200.0f

/* Minimum speed magnitude that still overcomes gearbox stiction.
 * Commands below this (but non-zero) are boosted up to it, so the robot
 * does not stall just short of the target. Set to 0.0f to disable.
 *
 * Raised from 35 -> 45: a 5-cycle TEST_TURN_LEFT_90 run at 35 completed 4/5
 * turns (undershooting 0.77-0.93 deg each time) but stalled on the 5th --
 * motor audibly running, wheel not turning, stuck until CONTROL_MOVE_TIMEOUT_MS
 * fired at 2.04 deg short. That pattern (consistent small undershoot,
 * occasional stall needing the same nudge) points to 35 sitting right at this
 * motor's real low-PWM breakaway torque rather than a loose/slipping
 * coupling. TURN_KI masks this some of the time by winding up until it
 * breaks through, but isn't a guaranteed fix, hence raising the floor itself.
 * Retest both turn and straight-line moves after changing this -- it is
 * shared by both controllers. */
#define CONTROL_MIN_MOVE_SPEED      45.0f


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


/* ====================== ROBOT GEOMETRY / CALIBRATION ===================== */

/* Distance between the two wheel contact patches, in cm.
 * This converts differential wheel travel into a rotation angle, so it is the
 * single most important number for turn accuracy. If every turn is off by the
 * same RATIO (e.g. all turns come out 5% short), correct this value rather
 * than the PID gains. A turn that overshoots means the value is too small. */
#define ROBOT_WHEEL_BASE_CM         11.20f


/* ============================== TURN PID ================================= */
/*                                                                           */
/*  !! UNITS CHANGED WHEN THE IMU WAS INTEGRATED !!                          */
/*                                                                           */
/* The turn controller now closes the loop on FUSED YAW IN DEGREES from the  */
/* EKF, not on differential wheel arc in cm. Gains therefore have a          */
/* different scale than before.                                              */
/*                                                                           */
/* Conversion between the two conventions:                                   */
/*     arc_cm = (PI * ROBOT_WHEEL_BASE_CM / 360) * angle_deg                 */
/*            = 0.09774 * angle_deg          (for an 11.20 cm wheel base)    */
/*                                                                           */
/* so    K_deg = K_arc * 0.09774                                             */
/*                                                                           */
/* The defaults below were converted from the arc-based gains that were       */
/* tuned on the encoder-only controller (Kp 40.0 -> 3.9), so behaviour        */
/* should start out close to what was already working. Expect to retune:      */
/* fused yaw is a cleaner, less noisy signal than raw encoder arc, so it      */
/* will usually tolerate a higher Kp and a real Kd.                          */

#define TURN_KP                     10.9f

/* TURN_KI = 5.0 (raised from 0.0). Two floor runs at Kd=0.5 both undershot
 * (0.93 deg, then 1.94 deg) instead of overshooting, and the second one
 * TIMED OUT stuck there: basespeed pinned at CONTROL_MIN_MOVE_SPEED (35)
 * with gyro rate ~0 dps for the rest of the 8s window -- 35 units just
 * wasn't quite enough torque to break static friction from a dead stop for
 * that particular residual, even though it was enough the previous run.
 * This is exactly this file's own trigger for adding Ki ("consistently
 * stops short"): a fixed command that isn't quite enough never gets bigger
 * on its own, but an integrator winds up against a persistent, unchanging
 * error until it does, then unwinds once the wheel actually moves.
 * Sized so ~2 deg of sustained error saturates TURN_INT_LIMIT (20) in
 * roughly 1-2 s, well inside CONTROL_MOVE_TIMEOUT_MS -- not yet tested on
 * target, treat as a starting point. If it overshoots on the recovery kick,
 * lower this before touching TURN_KD. */
#define TURN_KI                     5.0f

/* TURN_KD = 0.5 (raised from 0.0). Earlier runs at Kd=0 showed a clean
 * pure-P response one time (overshoot to 94.3 deg, then got stuck there for
 * the full CONTROL_MOVE_TIMEOUT_MS -- the correction command never
 * recovered) and a floor run at Kd=0.5 landing cleanly at 89.07 deg
 * (0.93 deg undershoot, well inside TURN_TOLERANCE_DEG, zero EKF slip
 * rejects, no timeout). Consistent with this file's own tuning order:
 * damping first to kill the overshoot-and-stick failure mode.
 * Do NOT bump TURN_KP or add TURN_KI off this single clean sample -- the
 * 0.93 deg undershoot is already within tolerance. Run several more floor
 * trials at this Kd before deciding whether that undershoot is a real bias
 * (-> small Ki) or just run-to-run noise (-> leave it alone). */
#define TURN_KD                     0.5f

/* Integrator clamp, in motor speed units */
#define TURN_INT_LIMIT              20.0f


/* ============================ IMU / EKF ================================== */

/* Sign of the gyro Z axis relative to the robot's yaw convention.
 * Convention: POSITIVE yaw = anticlockwise = a LEFT turn, matching the
 * encoder convention (right wheel forward, left wheel backward).
 *
 * CALIBRATE THIS FIRST with TEST_IMU_RAW: rotate the robot anticlockwise by
 * hand and confirm tm_gyro_z_dps reads POSITIVE. If it reads negative, flip
 * this to -1.0f. Everything downstream depends on getting this right. */
#define IMU_GYRO_Z_SIGN             (+1.0f)

/* How often the EKF prediction step runs, in microseconds.
 * The gyro ODR is configured to 1 kHz, so 1000 us consumes every sample
 * exactly once. Polling faster would integrate the same sample twice and
 * inflate the rotation estimate. */
#define IMU_PREDICT_PERIOD_US       1000U

/* Stationary gyro bias calibration, performed at startup.
 * The robot MUST be completely still while this runs. */
#define IMU_GYRO_BIAS_SAMPLES       1000U
#define IMU_GYRO_BIAS_SETTLE_MS     300U

/* Reject the calibration if the robot was clearly moving during it (deg/s).
 * Guards against calibrating while the robot is being carried. */
#define IMU_GYRO_BIAS_MAX_DPS       5.0f


/* --- EKF noise parameters ---
 *
 * These trade the gyro against the encoders. Rules of thumb:
 *   - Yaw drifting steadily during a long pause  -> lower EKF_Q_BIAS or
 *     recalibrate the bias; the filter is not tracking bias fast enough.
 *   - Yaw pulled off by wheel slip during turns  -> raise EKF_R_ENCODER_YAW
 *     (trust the encoders less) or tighten EKF_INNOVATION_GATE.
 *   - Yaw noisy//jittery                          -> lower EKF_Q_YAW.
 */

/* Yaw process noise density, rad^2/s. Covers gyro white noise plus scale
 * factor error. ICM-42688-P noise density is ~0.0028 dps/sqrt(Hz), which at
 * 1 kHz is well under this; the margin absorbs modelling error. */
#define EKF_Q_YAW                   1.0e-5f

/* Gyro bias random-walk density, (rad/s)^2/s. Small: the bias drifts slowly,
 * mostly with temperature. */
#define EKF_Q_BIAS                  1.0e-7f

/* Encoder yaw measurement variance, rad^2.
 * 1.0e-2 corresponds to about 5.7 deg of 1-sigma noise, which is deliberately
 * loose. Over the ~1 s of a pivot turn the gyro is far more trustworthy than
 * the wheels; the encoders are here as a slow anchor that keeps the gyro bias
 * observable, not as the primary angle source. */
#define EKF_R_ENCODER_YAW           1.0e-2f

/* Rate-dependent slip term, rad^2 per (rad/s)^2:
 *     R_effective = EKF_R_ENCODER_YAW + EKF_R_SLIP_COEFF * yaw_rate^2
 *
 * Wheel slip is systematic and grows with rotation speed, so a fixed R lets a
 * steady slip ramp pull the estimate off (innovation gating only catches
 * sudden outliers, not gradual ones). This term makes the filter distrust the
 * wheels in proportion to how hard it is turning.
 *
 * At 180 deg/s (3.14 rad/s) this raises R from 0.01 to ~2.0, i.e. the wheels
 * are effectively ignored mid-turn while still anchoring bias at rest.
 *
 * RAISE if turns still get dragged off by slip.
 * LOWER if turns are accurate but yaw drifts during long stationary pauses.
 * Set to 0.0f for a classical fixed-R filter. */
#define EKF_R_SLIP_COEFF            2.0e-1f

/* Initial state uncertainty */
#define EKF_P0_YAW                  1.0e-4f
#define EKF_P0_BIAS                 1.0e-4f

/* Reject encoder updates further than this many sigma from the prediction.
 * This is the wheel-slip rejector. Lower = more aggressive rejection.
 * Set to 0.0f to accept every update. */
#define EKF_INNOVATION_GATE         3.0f


/* ========================= Completion criteria =========================== */

/* How close (cm) counts as "arrived" for straightline moves. */
#define STRAIGHT_TOLERANCE_CM       0.7f

/* How close (degrees of fused yaw) counts as "arrived" for turns. */
#define TURN_TOLERANCE_DEG          1.0f

/* A turn only completes when the robot is both within tolerance AND rotating
 * slower than this (deg/s). Without the rate check the controller can declare
 * success while spinning through the target. */
#define TURN_SETTLE_RATE_DPS        8.0f

/* The move is only considered complete once the robot has been inside the
 * tolerance band for this many consecutive control cycles. Prevents declaring
 * success while still coasting through the target at speed. */
#define CONTROL_SETTLE_CYCLES       5

/* Safety timeout: abort a move that has not completed within this many ms. */
#define CONTROL_MOVE_TIMEOUT_MS     8000U

#endif /* CONTROL_CONFIG_H */
