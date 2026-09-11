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

/* Control loop sample time in seconds (10 ms = 100 Hz) */
#define CONTROL_SAMPLE_TIME_S 0.010f

/* Derivative low-pass filter time constant in seconds.
 * Rule of thumb: keep it a few times larger than the sample time.
 * Must be > 0 or the derivative term is unfiltered.
 *
 * Held at 0.010 (= T), knowingly violating the rule above. 0.030 was tried
 * and REVERTED: a 5-move TEST_TURN_LEFT_90 run went 3/5 -> 0/5, every move
 * timing out 3.6-5.9 deg short. Only tau changed between those two runs.
 *
 * The theory it was testing turned out to be wrong. The robot was NOT in a
 * stick-slip limit cycle driven by derivative spikes; it was stalling dead
 * and unable to restart (turn_gyro_rate_dps ~0.15 with turn_basespeed at
 * 65.4, i.e. commanded hard and not moving). Raising tau makes the
 * derivative LAG, which kept braking torque applied for ~30 ms after motion
 * had already ceased -- helping to seat the wheel in the stall rather than
 * preventing it. See TURN_INT_LIMIT for the actual fix.
 *
 * Worth revisiting only if a derivative term is ever run at a Kd high enough
 * that transient rejection matters more than phase lag. Note tau does NOT
 * change steady-state damping (the term settles to -Kd * d(measurement)/dt
 * regardless), so raising it only ever costs phase, never authority. */
#define CONTROL_DERIV_TAU_S 0.010f

/* Maximum motor speed command the controllers may produce (0..255). */
#define CONTROL_MAX_SPEED 200.0f

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
#define CONTROL_MIN_MOVE_SPEED 45.0f

/* ====================== STRAIGHTLINE: DISTANCE PID ======================= */
/* Drives average travelled distance (cm) to the target distance.            */

#define STRAIGHT_DIST_KP 20.0f
#define STRAIGHT_DIST_KI 0.0f
#define STRAIGHT_DIST_KD 0.0f

/* Integrator clamp, in motor speed units */
#define STRAIGHT_DIST_INT_LIMIT 50.0f

/* ====================== STRAIGHTLINE: HEADING PID ======================== */
/* Holds (left_count - right_count) at zero so the robot tracks straight.    */
/* Measurement is in RAW ENCODER TICKS, so these gains are small.            */

#define STRAIGHT_HEADING_KP 0.5f
#define STRAIGHT_HEADING_KI 0.025f
#define STRAIGHT_HEADING_KD 0.00f

/* Steering authority clamp, in motor speed units */
#define STRAIGHT_HEADING_LIMIT 80.0f
#define STRAIGHT_HEADING_INT_LIMIT 20.0f

/* ====================== ROBOT GEOMETRY / CALIBRATION ===================== */

/* Distance between the two wheel contact patches, in cm.
 * This converts differential wheel travel into a rotation angle, so it is the
 * single most important number for turn accuracy. If every turn is off by the
 * same RATIO (e.g. all turns come out 5% short), correct this value rather
 * than the PID gains. A turn that overshoots means the value is too small. */
#define ROBOT_WHEEL_BASE_CM 11.20f

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

#define TURN_KP 10.9f

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
#define TURN_KI 5.0f

/* TURN_KD = 0.25 (was 0.5). Damping was originally added to kill an
 * overshoot-and-stick failure mode at Kd=0, and 0.5 did that.
 *
 * !! READ THIS BEFORE CHANGING IT !!
 * Until the PID.c derivative sign fix, only HALF the configured Kd reached
 * the output, so every result ever recorded at "0.5" was really 0.25 of
 * damping. The best run on record -- 4/5, errors 0.91-1.43 deg -- was one of
 * them. Once the fix landed and the full 0.5 took effect, the same test went
 * 3/5 with two moves braking to a dead stop 4-5 deg short and timing out.
 * 0.25 therefore is not a retreat: it is the value the robot was actually
 * tuned at, now delivered by a filter that no longer amplifies noise.
 *
 * Raising this again means accepting that the robot decelerates to a full
 * halt before reaching tolerance, which on a high-traction surface it cannot
 * always restart from. Fix the stall authority (TURN_INT_LIMIT) first. */
#define TURN_KD 0.25f

/* Integrator clamp, in motor speed units.
 *
 * Raised 20 -> 60. At 20 this clamp silently capped the controller's total
 * authority far below CONTROL_MAX_SPEED and defeated the entire purpose Ki
 * was added for. Caught red-handed in a 0/5 run: stalled 4.165 deg short,
 * gyro rate 0.147 dps, and turn_basespeed pinned at 65.396 -- which is
 * EXACTLY TURN_KP*4.165 + 20.0 = 45.40 + 20.0. The integrator was hard
 * against this clamp with nothing left to give, so a robot that stopped
 * short could never generate enough torque to break static friction again.
 *
 * 60 gives ~105 units at 4 deg of error instead of 65. If it still stalls
 * there, breakaway on that surface exceeds what the controller can reach and
 * the problem is mechanical, not tuning.
 *
 * This is now the STALL-ONLY limit; see TURN_INT_LIMIT_MOVING below. Plain
 * clamping at 60 was tried first and did exactly the predicted damage: two
 * 5-move runs went 4/5 each, and BOTH failures were overshoots to ~98 deg
 * that then failed to recover inside CONTROL_MOVE_TIMEOUT_MS. The integrator
 * only unwinds once the error changes sign, so it carried surplus command
 * straight through the target. */
#define TURN_INT_LIMIT 60.0f

/* Integrator clamp while the robot is actually moving, in motor speed units.
 *
 * The integrator has two completely different jobs here, and they want
 * opposite limits:
 *
 *   - Mid-move it should do almost nothing. Kp already saturates the output
 *     for most of the turn, so anything the integrator accumulates is pure
 *     overshoot waiting to happen.
 *   - When the robot has stopped dead short of the target, it is the ONLY
 *     term that can grow, and it needs enough authority to break static
 *     friction (see the 65.4-unit stall documented above).
 *
 * So the limit is switched: this modest value normally, TURN_INT_LIMIT once
 * the robot has been demonstrably stuck for TURN_STALL_CYCLES. Dropping the
 * limit also bleeds down an already-wound integrator, because PIDController
 * clamps on every update, so recovery authority disappears the moment the
 * wheel starts turning again.
 *
 * !! CURRENTLY SET EQUAL TO TURN_INT_LIMIT, WHICH DISABLES THE SWITCHING !!
 *
 * The scheme above is sound and it worked exactly as designed. It just
 * solved the wrong problem. Set to 20 it scored 2/5, and all three failures
 * self-reported STALLED via the flight recorder: commanded at 75-90 speed
 * units while rotating 0.03-0.15 dps, with the integrator pinned at the full
 * boosted 60 in every case. turn_stall_boosts read 2580 cycles against 24 s
 * of timeout, so the recovery path armed almost continuously and did not
 * help.
 *
 * The lesson: 90 units cannot restart this robot from a dead stop, and
 * proportional authority shrinks exactly as the error does, so the ceiling
 * near the target is far below breakaway. Holding 60 throughout instead
 * scored 8/10 -- NOT because it recovers from stalls better, but because the
 * extra command through the approach means the robot never comes to rest
 * short of the target. On this drivetrain, stopping is unrecoverable, so the
 * winning strategy is to not stop.
 *
 * The detector and its telemetry are deliberately left in place: they cost
 * nothing, turn_stall_boosts still reports when the robot got stuck, and a
 * breakaway pulse (full scale for ~30 ms, then hand back to the PID) is the
 * right consumer for it. Static friction is broken by amplitude, not by an
 * integrator patiently ramping through a range where the wheel cannot move. */
/* REVERTED 20 -> 60 after a 0/20 run. Cutting it to 20 removed the authority
 * that was rescuing moves which end the profile stopped and short: the ceiling
 * at the end of a move is P + this clamp, so at 3 deg short that fell from 93
 * units to 53, against a measured breakaway above 140. Every one of 20 moves
 * then died 1.2-4.6 deg short with the command pointing the right way and
 * simply too small. The profile makes this worse than the old scheme did,
 * because it deliberately brings the robot to REST at the end of the sweep --
 * so being short means restarting from zero, which this drivetrain cannot do.
 */
#define TURN_INT_LIMIT_MOVING 60.0f

/* Stall detector: rotation below this rate (deg/s) while still outside
 * TURN_TOLERANCE_DEG counts as "not moving".
 *
 * Deliberately below TURN_SETTLE_RATE_DPS so a normal deceleration into the
 * tolerance band does not register. A genuine stall reads ~0.15 dps, so
 * there is a wide margin either side. */
#define TURN_STALL_RATE_DPS 5.0f

/* How many consecutive cycles the stall condition must hold before the
 * integrator is granted TURN_INT_LIMIT. At CONTROL_SAMPLE_TIME_S this is a
 * dwell time, and it is the whole reason this scheme does not reintroduce
 * the overshoot: a healthy 90 deg turn completes in ~800 ms and is never
 * stationary-but-short for anything like this long, so the boost simply
 * never arms. A real stall arms it in a fifth of a second. */
#define TURN_STALL_CYCLES 20U

/* ========================= TURN MOTION PROFILE ========================== */
/*                                                                          */
/* The turn no longer chases a step setpoint of "go to 90 degrees". It      */
/* tracks a trapezoidal reference that the robot can actually be at right   */
/* now, which keeps the PID in its linear region and, more importantly,     */
/* gives the move a KNOWN DURATION. Tolerance-based turns on this robot     */
/* ranged from 0.7 to 8 seconds; a profiled one is always the same.         */

/* Peak rotation rate, deg/s. */
/* MEASURED, not chosen. A per-cycle trace of a real turn showed the robot
 * sustaining 143 deg/s with the command saturated at CONTROL_MAX_SPEED, while
 * the profile was asking for 200. It simply cannot go that fast, so the
 * reference ran away and the tracking lag peaked at 28 degrees mid-move.
 *
 * 120 leaves headroom: at TURN_FF_GAIN the feedforward alone is ~164 units at
 * cruise, so ~36 units remain for the feedback to correct with before the
 * command clips. A profile the robot cannot follow is worse than no profile,
 * because the feedback spends the whole move saturated. */
#define TURN_PROFILE_MAX_DPS 120.0f

/* Angular acceleration, deg/s^2. With the peak above, a 90 deg turn ramps
 * for 0.167 s over 16.7 deg at each end and cruises the middle 56.7 deg,
 * giving a total of 0.617 s. Raise both together to go faster; raising accel
 * alone just spends longer at peak rate. */
/* Also measured: 0 to ~145 deg/s took about 200 ms, so roughly 725 deg/s^2 is
 * all this drivetrain has. 600 keeps a margin. */
#define TURN_PROFILE_ACCEL_DPS2 600.0f

/* Feedforward gain: motor speed units per deg/s of commanded rotation.
 *
 * THIS IS THE NUMBER THAT MAKES A PROFILE WORTH HAVING. It supplies the
 * command the move needs so the feedback term only has to correct the
 * difference. Get it right and the PID output hovers near zero mid-turn.
 *
 * NOW MEASURED rather than guessed: the trace showed 200 command units
 * producing 143 deg/s sustained, so 200/143 = 1.40. The previous value of 1.0
 * under-drove every move by 40%, which the integrator then covered for.
 * Re-check it from tm_turn_trace: during cruise the fb column should hover
 * near zero rather than sitting hard one way. */
#define TURN_FF_GAIN 1.40f

/* Acceleration feedforward: motor speed units per deg/s^2.
 *
 * Velocity feedforward alone cannot accelerate the robot, only hold a rate.
 * Without this term the integrator covers both ramps and is still wound
 * POSITIVE when the move ends, which pushes the robot past the target and
 * then keeps pushing the wrong way -- observed as 2 failures in a 20-turn
 * run, both with the command pointing away from the target.
 *
 * 0.03 x the 1200 deg/s^2 ramp is ~36 speed units, which is about what the
 * integrator was winding to. Trim it from turn_fb_cmd during the ramps. */
#define TURN_FF_ACCEL_GAIN 0.10f

/* Grace period after the profile ends, in ms, to close whatever small error
 * is left. BOUNDED ON PURPOSE: this is the whole difference between a move
 * that always finishes and the old settle-forever loop that could hang for 8
 * seconds. Worst-case move time is the profile duration plus this. */
#define TURN_PROFILE_SETTLE_MS 250U

/* The stiction floor is applied only while the profile commands at least this
 * much rotation. Below it the profile is deliberately winding down, and
 * forcing CONTROL_MIN_MOVE_SPEED there would drive the robot straight through
 * the target -- which is exactly how the old scheme produced its overshoots. */
#define TURN_PROFILE_FLOOR_DPS 20.0f

/* ============================ IMU / EKF ================================== */

/* Sign of the gyro Z axis relative to the robot's yaw convention.
 * Convention: POSITIVE yaw = anticlockwise = a LEFT turn, matching the
 * encoder convention (right wheel forward, left wheel backward).
 *
 * CALIBRATE THIS FIRST with TEST_IMU_RAW: rotate the robot anticlockwise by
 * hand and confirm tm_gyro_z_dps reads POSITIVE. If it reads negative, flip
 * this to -1.0f. Everything downstream depends on getting this right. */
#define IMU_GYRO_Z_SIGN (+1.0f)

/* How often the EKF prediction step runs, in microseconds.
 * The gyro ODR is configured to 1 kHz, so 1000 us consumes every sample
 * exactly once. Polling faster would integrate the same sample twice and
 * inflate the rotation estimate. */
#define IMU_PREDICT_PERIOD_US 1000U

/* Stationary gyro bias calibration, performed at startup.
 * The robot MUST be completely still while this runs. */
#define IMU_GYRO_BIAS_SAMPLES 1000U

/* Settle time before sampling starts, in ms.
 * Raised 300 -> 1000: pressing the reset button physically jolts the robot,
 * and on compliant tyres the chassis was still rocking when sampling began.
 * The acceptance test is the worst single sample out of IMU_GYRO_BIAS_SAMPLES,
 * so one leftover wobble anywhere in the sweep rejected the whole thing --
 * observed rejected in 2 of 3 consecutive test runs. */
#define IMU_GYRO_BIAS_SETTLE_MS 1000U

/* How many times to retry a calibration that was rejected for motion before
 * giving up. Each attempt costs SETTLE_MS + SAMPLES ms, so the worst case
 * startup cost is ATTEMPTS * (SETTLE_MS + SAMPLES) ms.
 * A rejected calibration is not harmless -- the EKF then runs with an
 * unestimated gyro bias -- so it is worth a few seconds at boot to get it. */
#define IMU_GYRO_BIAS_MAX_ATTEMPTS 3U

/* Reject the calibration if the robot was clearly moving during it (deg/s).
 * Guards against calibrating while the robot is being carried. */
#define IMU_GYRO_BIAS_MAX_DPS 5.0f

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
#define EKF_Q_YAW 1.0e-5f

/* Gyro bias random-walk density, (rad/s)^2/s. Small: the bias drifts slowly,
 * mostly with temperature. */
#define EKF_Q_BIAS 1.0e-7f

/* Encoder yaw measurement variance, rad^2.
 * 1.0e-2 corresponds to about 5.7 deg of 1-sigma noise, which is deliberately
 * loose. Over the ~1 s of a pivot turn the gyro is far more trustworthy than
 * the wheels; the encoders are here as a slow anchor that keeps the gyro bias
 * observable, not as the primary angle source. */
#define EKF_R_ENCODER_YAW 1.0e-2f

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
#define EKF_R_SLIP_COEFF 2.0e-1f

/* Initial state uncertainty */
#define EKF_P0_YAW 1.0e-4f
#define EKF_P0_BIAS 1.0e-4f

/* Reject encoder updates further than this many sigma from the prediction.
 * This is the wheel-slip rejector. Lower = more aggressive rejection.
 * Set to 0.0f to accept every update. */
#define EKF_INNOVATION_GATE 3.0f

/* ==================== VL53L0X ToF ranging sensors ======================== */

/* Which TCA9548A channel each sensor hangs off. The mux has 8 channels and
 * the PCB breaks out five ToF footprints; only three are populated for wall
 * detection. Verify against the Main PCB schematic before trusting a reading
 * -- a swapped pair here produces perfectly valid distances attributed to the
 * wrong direction, which is far harder to spot than a dead sensor. */
#define TOF_CHANNEL_FRONT 0U
#define TOF_CHANNEL_LEFT 3U
#define TOF_CHANNEL_RIGHT 4U

/* Factory default 7-bit address, shifted to the 8-bit form both the ST API
 * and the HAL expect. Every sensor keeps this address; the mux is what makes
 * them individually addressable. */
#define TOF_I2C_ADDR_DEFAULT 0x52

/* Measurement timing budget, microseconds. This is the master speed/accuracy
 * knob: longer budget = less noise and more range, at a lower sample rate.
 *
 *   20000  (20 ms) - ST's fastest preset, noticeably noisier
 *   33000  (33 ms) - ST's default, ~30 Hz
 *   200000 (200 ms) - high accuracy preset
 *
 * 33 ms is the starting point here. For a moving micromouse the sample rate
 * matters more than the last millimetre, so if wall following turns out to
 * lag, drop this before touching anything else. */
#define TOF_TIMING_BUDGET_US 33000U

/* Inter-measurement period for CONTINUOUS mode, milliseconds. Must be >= the
 * timing budget in ms, otherwise the sensor cannot keep up and simply runs
 * back-to-back. The margin over the budget covers the sensor's own overhead. */
#define TOF_INTER_MEASUREMENT_MS 40U

/* VCSEL pulse periods, in PCLKs. These set the range/ambient-immunity
 * trade-off and only accept specific values: pre-range 12/14/16/18,
 * final-range 8/10/12/14. ST's default profile is 14/10.
 *
 * Longer periods extend range (the long-range profile uses 18/14) at the cost
 * of ambient light immunity. Maze walls are close -- under 20 cm -- so the
 * default is kept; there is no reason to reach for range the robot will never
 * use and pay for it in noise. */
#define TOF_VCSEL_PERIOD_PRE_RANGE 14U
#define TOF_VCSEL_PERIOD_FINAL_RANGE 10U

/* Signal rate limit, MCPS, as a float converted to the API's 16.16 fixed
 * point at the call site. Readings weaker than this are rejected as noise.
 * ST's default is 0.25; the long-range profile lowers it to 0.1.
 * RAISE to reject more marginal readings, LOWER to see darker/further walls. */
#define TOF_SIGNAL_RATE_LIMIT_MCPS 0.25f

/* Sigma (standard deviation) limit, millimetres. Rejects readings the sensor
 * itself considers imprecise. ST's default is 18 mm. */
#define TOF_SIGMA_LIMIT_MM 18.0f

/* How long to wait for a measurement to complete before giving up, ms.
 * Must comfortably exceed the timing budget -- this is a stuck-sensor
 * detector, not a pacing mechanism. */
#define TOF_DATA_READY_TIMEOUT_MS 100U

/* How long ToF_StopContinuous() waits for the sensor to finish stopping, ms.
 * Only a runaway escape: the stop completes in a millisecond or two. */
#define TOF_STOP_TIMEOUT_MS 100U

/* ========================== WALL DETECTION ============================== */
/* Distances to booleans. See Core/Inc/Maze/wall_sense.h for the reasoning.  */

/* A front reading at or below this means the current cell has a wall ahead.
 * Robot centred, a front wall sits ~77 mm away as read; with no wall the
 * nearest surface is the far side of the NEXT cell at ~250 mm. The gap is
 * enormous, which is why the sensors' +27 mm close-range over-read is
 * irrelevant here and the offset constants are not needed for detection. */
#define WALL_FRONT_THRESHOLD_MM 150U

/* Same for the sides: a wall reads ~62 mm, an opening 300 mm and up. */
#define WALL_SIDE_THRESHOLD_MM 120U

/* Samples majority-voted by WallSense_ReadCell() at a cell centre.
 * Keep this ODD so there is never a tie. Worth the ~1 s it costs: the
 * algorithm never clears a wall once set, so a single bad reflection writing
 * a phantom wall closes a corridor permanently. */
#define WALL_SENSE_SAMPLES 5U

/* Extra distance a latched wall must recede before it counts as gone, used
 * only by the moving/hysteretic path. Without a separate exit level the
 * decision chatters at every cell boundary, which is exactly where the wall
 * really does end and where a wrong answer is most expensive. */
#define WALL_SENSE_HYSTERESIS_MM 40U

/* Consecutive cycles a changed answer must hold before it is latched. */
#define WALL_SENSE_CONFIRM 3U

/* ===================== WALL FOLLOWING (LATERAL) ========================= */
/*                                                                          */
/* Holds the robot centred in a corridor using ONE side wall, whichever is  */
/* currently in range. Two walls would let the common-mode sensor bias      */
/* cancel in the difference; with one it does not, which is why the         */
/* setpoints below are MEASURED READINGS rather than true distances.        */

/* What each sensor reads with the robot centred in a cell. NOT the true gap.
 *
 * These absorb the sensors' ~27 mm close-range over-read without needing
 * TOF_OFFSET_*_MM: if the setpoint is whatever the sensor says when the robot
 * is where you want it, a constant bias cancels exactly. Measure them by
 * centring the robot by hand and reading tm_tof_left_mm / _right_mm.
 *
 * From the maze-cell measurement: left 62.7, right 64.2, with the robot
 * roughly but not exactly centred. Re-measure properly before trusting them. */
#define WALL_FOLLOW_SETPOINT_LEFT_MM 63.0f
#define WALL_FOLLOW_SETPOINT_RIGHT_MM 64.0f

/* Lateral error (mm) -> commanded heading offset (deg).
 *
 * This is a CASCADE, not a second steering term added alongside the heading
 * loop. Lateral position is two integrations from steering, so summing two
 * independent corrections is undamped; feeding lateral error into the heading
 * SETPOINT makes the inner loop supply the derivative term for free.
 *
 * 0.25 deg per mm means a 10 mm offset asks for a 2.5 degree tilt. */
#define WALL_FOLLOW_KP_DEG_PER_MM 0.25f

/* Hard cap on that tilt. Bounds how sharply the robot ever turns to correct
 * sideways, which is the other thing summing the terms would not give. */
#define WALL_FOLLOW_MAX_TILT_DEG 12.0f

/* Slowly bleed the steady-state tilt back into the heading estimate.
 *
 * THIS IS THE DRIFT CORRECTOR, and it falls out of the cascade for free. If
 * the gyro has drifted, holding the wall at its setpoint requires a persistent
 * non-zero tilt -- so the standing output of the lateral loop IS the drift.
 * Bleeding it into the heading target bounds the drift with no magnetometer
 * and no differentiating of wall distance.
 *
 * Deliberately tiny: this must be far slower than the lateral loop, or the two
 * fight and the robot weaves. Degrees of correction per second of held tilt. */
#define WALL_FOLLOW_DRIFT_BLEED 0.02f

/* ============ STRAIGHTLINE: FUSED HEADING PID (degrees) ================= */
/* Used by runForwardFused(). Distinct from STRAIGHT_HEADING_*, which holds  */
/* (left - right) encoder TICKS and is blind to wheel slip -- if a wheel     */
/* slips, that loop steers to correct a rotation that never happened. This   */
/* one closes on fused yaw, which is the only signal that can tell the two   */
/* apart. Output is a differential speed correction.                          */

#define STRAIGHT_YAW_KP 8.0f
#define STRAIGHT_YAW_KI 0.0f
#define STRAIGHT_YAW_KD 0.20f

/* Steering authority clamp, motor speed units. Steering takes PRIORITY over
 * forward speed when the two together would clip: the base speed is reduced
 * to make room. Clipping each wheel independently instead turns a pure
 * steering command into a net speed change, which is how the existing
 * straight-line path loses steering authority exactly when it is fastest. */
#define STRAIGHT_YAW_LIMIT 60.0f
#define STRAIGHT_YAW_INT_LIMIT 20.0f

/* Control cycles between ToF sweeps during a fused move.
 * 4 at CONTROL_SAMPLE_TIME_S is 25 Hz, which already outruns the sensors'
 * TOF_INTER_MEASUREMENT_MS. Polling every cycle would just spend I2C time
 * re-reading the same measurement. */
#define STRAIGHT_TOF_DIVIDER 4U

/* =================== STRAIGHTLINE MOTION PROFILE ======================== */
/* Same reasoning as the turn profile, and the same generator.             */
/*                                                                         */
/* The first arena run overshot an 18 cm cell by 2.07 cm and then TIMED    */
/* OUT trying to come back -- the distance loop was still chasing a step   */
/* setpoint, so it ran at the stiction floor until it was already past.    */
/* A profile decelerates on a plan instead of on saturation decay.         */

/* MEASURED from a per-cycle trace, not guessed. The robot sustains 20.1 cm/s
 * with the command saturated at 200, and reaches it in about 600 ms.
 *
 * The first values here were 25 cm/s and 50 cm/s^2, and they failed exactly
 * the way the turn profile failed at 200 deg/s: the command sat at 200 for a
 * full second, the reference ran 7.75 cm ahead, and the profile declared the
 * move over while the robot was still at 14.5 cm doing full speed. It then
 * coasted to 20.2 and could not reverse back.
 *
 * 10 cm/s leaves headroom: feedforward alone is ~80 units at cruise, so ~120
 * remain for the feedback to correct with before the command clips. */
#define STRAIGHT_PROFILE_MAX_CMS 10.0f
#define STRAIGHT_PROFILE_ACCEL_CMS2 20.0f

/* WHY SO SLOW: this robot's braking distance, not its top speed, sets the
 * cell time. Measured coasting from 18.7 cm/s to rest took 8.5 cm -- half a
 * maze cell -- and it cannot reverse out of an overshoot because that needs
 * breaking static friction from a standstill.
 *
 * So the plan must be one the robot can actually stop from. At 10 cm/s and
 * 20 cm/s^2 the braking ramp is 2.5 cm, which is both what the profile
 * budgets and what the robot physically does. 18 cm then takes 2.3 s.
 *
 * Faster is available only after the stopping problem is solved properly,
 * either with real braking authority or by not requiring the robot to stop
 * at every cell. */

/* Motor speed units per cm/s.
 *
 * Lowered 9.9 -> 8.0. The 9.9 came from dividing a SATURATED command by the
 * speed it produced, which measures the top of the curve rather than its
 * slope -- this motor is already speed-limited by about 130 units, so 139 and
 * 200 both give ~19 cm/s. Using that as a gain made the feedforward command
 * 139 units for a requested 14 cm/s and the robot ran 34% fast.
 *
 * The feedforward sets the speed almost on its own here: at cruise it was 139
 * of a 133 total command, with the feedback trimming by only -5. So this gain
 * IS the cruise speed, and getting it wrong is not something the loop
 * quietly absorbs. */
#define STRAIGHT_FF_GAIN 8.0f

/* ---------------------- Noise filtering (tof_filter.c) ------------------- */

/* EMA smoothing factor, 0..1. This is the speed/smoothness trade-off:
 *
 *   1.0  = filter disabled, output follows the median stage exactly
 *   0.3  = light smoothing, fast response
 *   0.2  = the default here
 *   0.05 = very smooth but sluggish
 *
 * Roughly, the output reaches ~63% of a step after 1/alpha samples, so 0.2 is
 * about 5 samples -- at TOF_INTER_MEASUREMENT_MS = 40 ms that is ~200 ms to
 * settle. Note the jump detector below bypasses this entirely for large steps,
 * so this constant governs how hard SMALL jitter is smoothed, not how fast the
 * robot notices a wall appearing or disappearing.
 *
 * LOWER if readings are still too noisy to steer on.
 * RAISE if wall-following feels laggy or starts to oscillate. */
#define TOF_FILTER_EMA_ALPHA 0.2f

/* A median-stage change at least this large (mm) is treated as a real step
 * and the filter snaps to it instead of easing across it.
 *
 * Must sit ABOVE the noise spread and BELOW the smallest genuine transition.
 * Measured spread on this robot is ~7 mm; a side wall ending changes the
 * reading by 100 mm or more, so 30 mm sits comfortably between the two.
 *
 * LOWER and ordinary jitter starts tripping it, defeating the smoothing
 * (watch tm_tof_jumps climbing while the robot sits still -- it should not
 * move at all when nothing is moving).
 * RAISE and real wall transitions get smoothed into a slow ramp. */
#define TOF_FILTER_JUMP_THRESHOLD_MM 30U

/* ------------------------- Per-sensor offsets ---------------------------- */

/* Signed millimetres ADDED to each sensor's raw reading before filtering.
 *
 * This corrects BIAS -- a constant over- or under-read -- which no amount of
 * filtering can fix: averaging biased samples just produces a stable wrong
 * number. Bias comes from cover glass, mounting depth and the module's own
 * calibration, so it is per-sensor and must be measured per-sensor.
 *
 * HOW TO MEASURE: run TEST_TOF_SINGLE, put a flat target at a known distance
 * (80-100 mm is representative for maze walls), read the FILTERED value once
 * it settles, and set the offset to (true - measured). If the sensor reads
 * 86 mm at a true 80 mm, the offset is -6.
 *
 * Measure at a distance you actually care about. VL53L0X error is not
 * perfectly constant with range, so a single offset is a linear fix to a
 * mildly nonlinear problem -- calibrating at 80 mm and driving at 80 mm is
 * accurate, calibrating at 500 mm and driving at 80 mm is not. */
/* MEASURED 2026-09-11 against a ruler, robot stationary, 200 samples each.
 *
 *   sensor   true    reads   error
 *   front    60 mm   86.4    +26.4
 *   left     36 mm   62.7    +26.7
 *   right    35 mm   64.2    +29.2
 *
 * Three independent sensors agreeing to within 3 mm at the distances the
 * robot actually operates at.
 *
 * !! LEFT AT 0 ON PURPOSE -- the measurement is recorded here, not applied. !!
 * The values that would cancel the error are front -26, left -27, right -29.
 * They are held back until the rest of the robot needs them, so that ToF
 * readings stay raw while other subsystems are being brought up and there is
 * one less transform between sensor and number when something looks wrong.
 *
 * TWO HYPOTHESES WERE TESTED AND KILLED before landing here, both cheaper to
 * re-read than to re-derive:
 *
 *   ANGLED MOUNTING was proposed because the side sensors read 1.8x the ruler
 *   distance at 35 mm, implying a ~55 deg tilt. Dead: a 55 deg tilt predicts
 *   523 mm where the left sensor actually read 319 mm at a true 300 mm. The
 *   sensors are perpendicular.
 *
 *   A PERFECTLY CONSTANT OFFSET is close but not exact. At longer range the
 *   over-read shrinks (left +19.0 at 300 mm, right +8.4 at 150 mm), which is
 *   the VL53L0X's known near-field behaviour rather than a fault.
 *
 * WHEN THEY ARE APPLIED, they will be tuned for SHORT RANGE and will make
 * mid-range WORSE -- the right sensor's 150 mm reading goes from +8 to about
 * -21. That is the correct trade for a micromouse: side walls sit ~35 mm away
 * and a front wall matters at 60-90 mm when deciding to stop. Nothing needs
 * accuracy at 150 mm. Detecting a wall two cells ahead is a binary call with
 * hundreds of mm of margin, so a 20 mm error there changes nothing.
 *
 * NOTE the left/right pair also carries the differential trim. Centring
 * between walls steers on (left - right), so the common-mode over-read
 * cancels and only the 2.5 mm mismatch between the two sensors matters. The
 * -27/-29 split removes it: at the measured maze position they correct to
 * 35.7 and 35.2 against a true 36 and 35.
 *
 * If absolute short-range accuracy is ever genuinely needed, the real fix is
 * ST's VL53L0X_PerformOffsetCalibration() and XTalk calibration, which
 * ToF_InitSensor() does not currently call. */
#define TOF_OFFSET_FRONT_MM 0 /* measured: -26 */
#define TOF_OFFSET_LEFT_MM 0  /* measured: -27 */
#define TOF_OFFSET_RIGHT_MM 0 /* measured: -29 */

/* ========================= Completion criteria =========================== */

/* How close (cm) counts as "arrived" for straightline moves. */
/* Widened 0.7 -> 1.5.
 *
 * There is NO SECOND CHANCE on a straight move: the profile brings the robot
 * to rest, and from rest it cannot restart at the command a sub-centimetre
 * error produces. So the move has to land inside the band first time, and a
 * band narrower than the landing scatter just guarantees a timeout. */
#define STRAIGHT_TOLERANCE_CM 1.5f

/* How close (degrees of fused yaw) counts as "arrived" for turns. */
/* Widened 1.0 -> 2.0.
 *
 * Not a retreat: it is the tolerance that was failing moves, not the motion.
 * Across the last three runs every single failure landed between 1.0 and 1.4
 * degrees -- the controller's real repeatability is about +/-1.4, and a 1.0
 * band sits inside its own noise.
 *
 * 2.0 is also what the architecture downstream actually needs. Heading error
 * after a turn is absorbed by the wall-following straight that follows it, and
 * over one 18 cm cell 2 degrees is ~6 mm of lateral drift against ~35 mm of
 * clearance. Demanding better from the turn buys nothing the next move does
 * not already provide. */
#define TURN_TOLERANCE_DEG 2.0f

/* A turn only completes when the robot is both within tolerance AND rotating
 * slower than this (deg/s). Without the rate check the controller can declare
 * success while spinning through the target. */
#define TURN_SETTLE_RATE_DPS 8.0f

/* The move is only considered complete once the robot has been inside the
 * tolerance band for this many consecutive control cycles. Prevents declaring
 * success while still coasting through the target at speed. */
#define CONTROL_SETTLE_CYCLES 5

/* Safety timeout: abort a move that has not completed within this many ms. */
#define CONTROL_MOVE_TIMEOUT_MS 8000U

#endif /* CONTROL_CONFIG_H */
