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
 * !! 120 WAS STILL TOO FAST. Lowered to 90 after it failed a run. !!
 *
 * 36 units of headroom is not headroom. A per-cycle trace of the failing turn:
 * feedforward pinned at -168 through the whole cruise, feedback asking for a
 * further -20 to -60, so the total command sat at -188 to -228 against a
 * +/-200 clamp. Saturated, for the entire middle of the move.
 *
 * Two things followed, and the second one ended the run:
 *
 *   THE ROBOT COULD NOT HOLD 120. At a command clipped to 200 it managed
 *   113.5 dps, so the reference ran away and the robot tracked 5-7 deg behind
 *   for the whole cruise. (The 143 dps recorded earlier was a different
 *   surface; treat that number as the ceiling on a good day, not a spec.)
 *
 *   IT THEREFORE OVERSHOT, WHICH LOOKS BACKWARDS AND IS NOT. Lagging in
 *   POSITION does not mean lagging in SPEED. When the profile finished its
 *   deceleration ramp the robot was still 5 deg short and still doing ~114
 *   dps, and shedding that at TURN_PROFILE_ACCEL_DPS2 needs 10.8 deg -- so it
 *   sailed 5.26 deg PAST the target. From there it had to reverse from rest,
 *   which this drivetrain cannot do, and the move timed out.
 *
 * 90 dps puts feedforward at 126 units and leaves 74 for the feedback, and it
 * is comfortably under the 113 the robot actually delivers. A profile the
 * robot cannot follow is worse than no profile, because the feedback spends
 * the whole move saturated and the end of the ramp is a guess. */
/* LOWERED AGAIN, 90 -> 80, and for the same reason as 120 -> 90: the number
 * that matters is not the speed, it is what is left over for the feedback
 * after the feedforward has taken its share. With TURN_FF_GAIN at its
 * corrected 2.00:
 *
 *     90 deg/s -> feedforward 180 of 200, leaving 20 for feedback
 *     80 deg/s -> feedforward 160 of 200, leaving 40
 *
 * 20 units is not enough to absorb a feedforward that is wrong by tens of
 * units, and it will be wrong, because the gain above moves with surface and
 * battery. 40 units is. Braking from 80 also costs 5.3 degrees against 6.8
 * from 90, so the end of the move is more forgiving as well.
 *
 * The cost is 0.1 s per turn. */
#define TURN_PROFILE_MAX_DPS 80.0f

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
 * RAISED 1.40 -> 2.00, and the old value is worth keeping in view because it
 * was also measured. 200 units once produced 143 deg/s, giving 1.40. A later
 * trace on the arena floor showed 176 units producing 87.9 deg/s, giving 2.00.
 * Same robot, same firmware, 43% apart.
 *
 * SO THIS NUMBER IS NOT A CONSTANT OF THE ROBOT. It moves with surface and
 * with battery state, and a fixed feedforward will always be somewhat wrong.
 * What matters is that the FEEDBACK has enough headroom to absorb the error
 * rather than clipping, which is why TURN_PROFILE_MAX_DPS was lowered
 * alongside this rather than left where it was.
 *
 * Under-driving is not harmless, and it does not merely make the turn slow.
 * At 1.40 the feedforward supplied 126 units while the move needed 176, so
 * the feedback carried a steady -49 and the robot tracked 5 to 7 degrees
 * BEHIND the reference for the entire cruise. It then arrived at the target
 * still doing nearly full rate, because the controller was pushing it to catch
 * up, instead of decelerating into the target with the profile. Shedding that
 * takes 6.75 degrees, so the move finished 4.25 degrees past and outside
 * TURN_TOLERANCE_DEG, and from there recovery needs breaking static friction
 * from rest, which this drivetrain cannot do.
 *
 * HOW TO RE-MEASURE: read tm_turn_trace and look at the fb column during
 * cruise. It should hover near zero. If it sits hard one way, scale this gain
 * by (ff + fb) / ff -- the offline reader prints that factor directly. */
#define TURN_FF_GAIN 2.00f

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
 * roughly but not exactly centred. Re-measure properly before trusting them.
 *
 * NOTE these now matter far less than they used to. When BOTH walls are in
 * range the follower centres on the DIFFERENCE, (L - R)/2, and only the ~1 mm
 * mismatch between the pair survives -- the common-mode over-read cancels
 * exactly, and so does any error in the corridor width. These full values are
 * used only on the single-wall path, where nothing cancels. Measured over one
 * run, L + R came to 121-129 mm with a mean of 124 across every genuine pair,
 * which is what makes the difference trustworthy. */
#define WALL_FOLLOW_SETPOINT_LEFT_MM 63.0f
#define WALL_FOLLOW_SETPOINT_RIGHT_MM 64.0f

/* What the two side readings SUM to when both are walls of the robot's own
 * cell, in mm. Measured, like the setpoints, and for the same reason.
 *
 * This is the cell's inner width as the sensors see it, and it does not depend
 * on where the robot sits between the walls -- move 10 mm left and one reading
 * falls by 10 while the other rises by 10. That makes it the one quantity that
 * can tell a same-cell wall from something further away, which an absolute
 * distance threshold cannot: a reading of 90 mm is a perfectly ordinary wall
 * if the other side reads 34, and is not a wall at all if the other side reads
 * 239.
 *
 * Measured over ten two-wall cells in one run: 117 to 128, mean 124. */
#define WALL_FOLLOW_SPAN_MM 124.0f

/* How far the sum may stray before the pair is called inconsistent.
 *
 * Covers the spread above with room to spare, and still rejects the failures:
 * a (41, 108) pair from an earlier run sums to 149, which is 25 mm out and is
 * the beam catching a surface a cell away through an opening. */
#define WALL_FOLLOW_SPAN_TOL_MM 20.0f

/* Largest side reading the FOLLOWER will centre on, in mm.
 *
 * DELIBERATELY TIGHTER THAN WALL_SIDE_THRESHOLD_MM, and the difference is the
 * point. Detection asks "is there a wall?", and it can afford to be generous
 * because the two cases are hundreds of mm apart. Following asks "how far am I
 * from THIS wall?", and a marginal reading there does not produce a marginal
 * response -- it produces a full-scale steering command in whatever direction
 * the number implies.
 *
 * Every genuine side reading in a 17-cell run fell between 51 and 87 mm, with
 * a median of 62. The two that did not were 34 (robot hard over) and 113. At
 * 120 the 113 was accepted, read as 50 mm of error against the setpoint,
 * saturated the tilt, and steered the robot hard at something that was not the
 * wall it thought it was -- which is what a junction looks like from the
 * sensor's point of view: the beam passing an opening and catching a surface a
 * cell away.
 *
 * 95 leaves room for a robot 30 mm off centre against a real wall and refuses
 * everything beyond. A refused reading is not a failure; the follower simply
 * holds heading open-loop, which is the right answer when it cannot see a wall
 * it trusts. */
#define WALL_FOLLOW_USABLE_MAX_MM 95U

/* ONE-SIDED SINGLE-WALL CORRECTION.
 *
 * With both walls the difference says where the robot is and nothing is
 * ambiguous. With one wall it does not, and the two directions of error are
 * not equally trustworthy:
 *
 *   READING BELOW THE SETPOINT is unambiguous. Something IS there and it IS
 *   close. Whether the robot is off-centre or the corridor is narrow does not
 *   matter -- moving away is right either way, and this is also the dangerous
 *   case, because being hard against a wall is what jams the next pivot.
 *
 *   READING ABOVE THE SETPOINT is ambiguous, and at a junction it is usually
 *   wrong. The robot might be off-centre away from the wall, or the wall might
 *   have ended and the beam is catching an edge or a surface beyond it. Acting
 *   on it steers the robot hard TOWARDS something that may not be there.
 *
 * SO IT IS ASYMMETRIC, NOT ONE-SIDED. A hard one-sided rule was tried first
 * and it over-corrected the problem: across five consecutive junction cells it
 * zeroed four of the five available corrections, and the robot drifted about
 * 4 mm per cell with nothing pushing back. It entered that stretch 5.5 mm off
 * centre and left it 18.5 mm off, then reached 28 mm two cells later.
 *
 * The fix is to keep the asymmetry but make the weak direction weak rather
 * than absent. Pushing AWAY from a close wall keeps full authority. Pulling
 * TOWARDS a distant one runs a smaller gain and a much tighter tilt cap, so it
 * bleeds off drift without ever lunging at something that might not be there.
 *
 * At the cap, one cell of travel still buys 192*sin(2.5) = 8.4 mm of
 * correction, comfortably more than the 4 mm per cell the robot was losing. */
/* Integral term on the LATERAL error, in degrees of tilt per mm-second.
 *
 * THE LOOP WAS PROPORTIONAL-ONLY, AND IT SHOWS. Seven consecutive cells with
 * both walls visible and full correction running still walked from -1.5 mm to
 * +8.0 mm, about 1.4 mm per cell. That is not the loop failing, it is what a
 * P-only loop does against a standing disturbance: at +8 mm it holds 4 degrees
 * of tilt, and that tilt BALANCES the disturbance rather than removing it.
 * Steady-state error is the definition of the thing.
 *
 * The disturbance is some persistent asymmetry -- a wheel slightly larger, a
 * motor slightly stronger, a gyro bias the EKF has not caught. Which one does
 * not matter to this term: an integrator nulls a standing error whatever
 * causes it, which is exactly why it is the right tool here and why guessing
 * the cause first is not necessary.
 *
 * SIZED to supply the 4 degrees the P term is currently holding, over roughly
 * five seconds of driving: 8 mm * 0.1 * 5 s = 4 deg. Slow on purpose. It must
 * be far slower than the proportional part or the two fight and the robot
 * weaves, which is the same rule the drift bleed follows.
 *
 * IT PERSISTS ACROSS MOVES. The asymmetry belongs to the robot, not to one
 * cell, so resetting it every move would mean re-learning it every move and
 * never converging -- which is precisely what has been happening to
 * wf_drift_deg, wiped at the start of all 26 moves of every run and reading
 * exactly 0.000 in every log because of it. Only WallFollow_ResetBias() clears
 * it, once at the start of a run. */
#define WALL_FOLLOW_KI_DEG_PER_MM_S 0.10f

/* Clamp on the integral, in degrees.
 *
 * NOT bounded by WALL_FOLLOW_MAX_TILT_DEG, because the integral is no longer
 * part of the tilt. It is added to the HEADING TARGET instead, outside the
 * tilt clamp, and that distinction is the whole fix.
 *
 * Inside the clamp it could not do its job. The failure is easy to state: if
 * the yaw estimate is off from the maze by more than WALL_FOLLOW_MAX_TILT_DEG,
 * the robot cannot recover. Suppose the estimate reads 8 degrees low. To drive
 * straight the loop must command +8, but the clamp stops it at +6, so the
 * robot keeps turning -- slower, but in the same wrong direction, for as long
 * as the wall lasts. The proportional term saturates first and the integral,
 * sharing the same 6 degree budget, then has nothing left to give. A run was
 * observed doing exactly this: yawed left with the left wall in view and no
 * correction arriving.
 *
 * Outside the clamp there is no such ceiling. The proportional term keeps its
 * full tilt budget for position, and the integral separately shifts the
 * reference the inner loop holds -- which is the correct place for it, since
 * what it is learning IS a heading reference error.
 *
 * 8 degrees bounds it well past any accumulated gyro-to-maze misalignment a
 * run should ever produce, and still far short of a wrong turn. */
#define WALL_FOLLOW_KI_LIMIT_DEG 8.0f

#define WALL_FOLLOW_SINGLE_FAR_KP 0.20f
#define WALL_FOLLOW_SINGLE_FAR_TILT_DEG 2.5f

/* Lateral error (mm) -> commanded heading offset (deg).
 *
 * This is a CASCADE, not a second steering term added alongside the heading
 * loop. Lateral position is two integrations from steering, so summing two
 * independent corrections is undamped; feeding lateral error into the heading
 * SETPOINT makes the inner loop supply the derivative term for free.
 *
 * RAISED 0.25 -> 0.50 after a jammed turn.
 *
 * THE ROBOT IS LARGE FOR THE CELL, so this gain is not a comfort setting --
 * it is what decides whether a pivot fits. Measured across one 22-cell run:
 *
 *   every cell that turned successfully   within  8.5 mm of centre
 *   both in-place 180s that worked        within  2.0 mm
 *   the 180 that jammed on a wall        24.0 mm off, wall at 34 mm
 *
 * The chassis does fit. It just needs about a centimetre of centring accuracy
 * to do it, and single-wall cells were not delivering that.
 *
 * The loop is proportional, so the error decays exponentially along a cell
 * rather than closing linearly, and the fraction removed over one 19.2 cm cell
 * is 1 - exp(-CELL * KP * pi/180):
 *
 *     KP 0.25   57% removed   a 27 mm error leaves 11.7 mm
 *     KP 0.50   81% removed   a 27 mm error leaves  5.1 mm
 *     KP 0.75   92% removed   a 27 mm error leaves  2.2 mm
 *
 * 0.50 brings a worst-case single-wall error back inside the budget in one
 * cell. 0.75 is tempting and is the next thing to try, but this is a cascade
 * whose inner heading loop has its own lag, so raise it one step at a time and
 * watch for weaving. At 0.50 a 24 mm error already asks for the full
 * WALL_FOLLOW_MAX_TILT_DEG, so beyond here the clamp is doing the limiting,
 * not the gain. */
#define WALL_FOLLOW_KP_DEG_PER_MM 0.50f

/* Hard cap on that tilt.
 *
 * !! IT MUST NOT EXCEED THE INNER LOOP'S LINEAR RANGE. Lowered 12 -> 6. !!
 *
 * This is the outer half of a cascade, and its output is the inner loop's
 * SETPOINT. The inner loop here is STRAIGHT_YAW_*, which has gain 8 and clamps
 * its output at STRAIGHT_YAW_LIMIT = 60, so it saturates at 60/8 = 7.5 degrees
 * of heading error. Asking for 12 is therefore asking for something the inner
 * loop can only answer with a pinned output -- the cascade stops being a
 * cascade and becomes bang-bang.
 *
 * That is exactly how a reverse out of a dead end failed. The robot entered it
 * 10 deg off heading and 30 mm off centre, the lateral loop asked for a further
 * 12, steering pinned at +60 on the very first cycle and stayed there for 1.2
 * seconds, and the robot rotated about 40 degrees -- far more than the 22 it
 * was asked for -- then slammed to -60 coming back and jammed against the wall
 * at 15.4 cm of a 19.9 cm move.
 *
 * 6 keeps the demand inside the inner loop's linear range with margin, and it
 * still buys 192*sin(6) = 20 mm of lateral correction per cell, which covers
 * the errors actually seen. Raising STRAIGHT_YAW_LIMIT would raise this
 * ceiling too, but steering authority is taken out of forward speed, so widen
 * the inner loop first and only then this. */
#define WALL_FOLLOW_MAX_TILT_DEG 6.0f

/* Fastest the tilt demand may CHANGE, in degrees per second.
 *
 * A clamp bounds where the heading target can go; this bounds how fast it gets
 * there. They are different failures. The wall follower can legitimately jump
 * its output in one cycle -- a wall ending, the active side changing, a robot
 * arriving off-centre -- and a step in a heading setpoint asks the robot to
 * rotate as hard as it can, which is never what centring wants.
 *
 * 30 deg/s crosses the full clamp range in 0.4 s, comfortably inside a 2.4 s
 * cell, so the correction still completes while the inner loop only ever sees
 * a ramp it can track. */
#define WALL_FOLLOW_TILT_SLEW_DPS 30.0f

/* ---- WHY THERE IS NO REVERSE MOVE. Not a tuning choice. ----
 *
 * The side sensors are mounted at the VERY FRONT of the chassis. That makes
 * them a long way from the wheel axis, and it decides which direction of
 * travel they can be used for:
 *
 *   FORWARDS  to move right you tilt the nose right, and the sensors -- being
 *             at the leading end -- swing right with it. The reading improves
 *             immediately and keeps improving. The measurement LEADS the body,
 *             which is free phase lead and it is what makes the cascade work.
 *
 *   BACKWARDS to move right you tilt the nose LEFT, and the sensors swing LEFT,
 *             hard, because of that same long arm. The reading says the error
 *             got worse, so the loop tilts further, and the body only catches
 *             up later. It is non-minimum-phase, and with the sensors this far
 *             forward the wrong-way excursion is large enough that one cell of
 *             travel is not long enough to recover inside.
 *
 * Reducing the gain does not fix this, it only makes the loop too slow to do
 * anything useful before the move ends. Measured: backing out of a dead end
 * 15 mm off centre, the robot came out 26 mm off. The correction moved it the
 * wrong way and the move finished inside that window.
 *
 * SO THE ROBOT DOES NOT REVERSE AT ALL, and there is no reverse move in the
 * firmware. Backing out of a dead end was built and then removed. The whole
 * point of it was to buy a cell of lateral correction before pivoting, and
 * without a usable lateral loop it bought nothing: the pivot would have
 * happened at exactly the offset it would have had anyway, one cell further
 * back. Turning in place first is strictly better, because the forward move
 * that follows DOES correct laterally.
 *
 * If side sensors are ever added further back on the chassis, or a rear-facing
 * pair appears, this is the note to revisit -- the objection is entirely about
 * where the sensors are, not about reversing. */

/* WALL_FOLLOW_DRIFT_BLEED IS GONE, folded into the integral above.
 *
 * The drift corrector and the lateral integral were two integrators doing one
 * job, in series, on the same error. The bleed integrated the TILT, and the
 * tilt contained the integral, so the pair wound each other with no clamp on
 * the outer one -- badly conditioned at best, and guaranteed to fight, since
 * two integrators in series on one error have no way to agree on which of
 * them owns the correction.
 *
 * One integrator now does both jobs, and it is the drift corrector: it
 * integrates the lateral ERROR and is added to the heading target. The
 * reasoning that motivated the bleed still holds exactly as written -- a tilt
 * the robot must hold forever is the gyro being wrong, because a centred robot
 * needs no tilt to stay centred -- it just arrives one integration earlier.
 *
 * See WALL_FOLLOW_KI_DEG_PER_MM_S and WALL_FOLLOW_KI_LIMIT_DEG. */

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

/* Seconds between WallFollow_Update() calls. It runs once per ToF sweep, not
 * once per control cycle, and both the slew limit and the drift bleed are
 * rates -- so they need the interval they are actually integrated over. The
 * bleed had been using the control period and was therefore running
 * STRAIGHT_TOF_DIVIDER times slower than its constant claimed. */
#define WALL_FOLLOW_UPDATE_S                                                   \
  (CONTROL_SAMPLE_TIME_S * (float)STRAIGHT_TOF_DIVIDER)

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

/* ------------------- FRONT-WALL ALIGNMENT (longitudinal) ---------------- */
/*                                                                          */
/* The side walls close the loop on the robot's SIDEWAYS position. Nothing  */
/* closed it on the FORWARD position until now -- that was odometry only,  */
/* and odometry has no opinion about where the cell boundaries are.         */
/*                                                                          */
/* WHY THAT MATTERS MORE THAN IT SOUNDS. A pivot swaps the two axes: after  */
/* a 90 degree turn, the error you had along the direction of travel        */
/* becomes the error across it, which is what decides whether the chassis   */
/* clears the walls. So an uncontrolled forward axis shows up one move      */
/* later as a clearance problem, and that is why back-to-back turns were    */
/* so much worse than corridors. Measured over one 24-cell run:             */
/*                                                                          */
/*    off-centre in the corridor stretch        worst  6.5 mm               */
/*    off-centre in the turn-dense stretch      worst 23.0 mm               */
/*    front-wall gap wherever a wall was ahead  50 to 110 mm, spread 60     */
/*                                                                          */
/* That 60 mm of scatter in where the robot stops is the same number as the */
/* lateral error that follows a turn. It is one quantity seen from two      */
/* directions, and this section is what removes it.                         */

/* What the FRONT sensor reads with the robot at a cell centre and a wall on
 * the far side of that cell.
 *
 * A MEASURED READING, not a true distance -- same convention as the
 * WALL_FOLLOW_SETPOINT pair, and for the same reason: if the target is
 * whatever the sensor says when the robot is where you want it, the sensor's
 * close-range over-read cancels exactly and TOF_OFFSET_FRONT_MM stays at 0.
 *
 * 87 is the mean of the ten walled stops in that run, which is a reasonable
 * starting point because the robot was aiming at cell centres. RE-MEASURE IT
 * PROPERLY: put the robot at a cell centre by hand with a wall ahead and read
 * tm_tof_front_mm. It is worth getting right; everything above depends on it.
 */
#define WALL_FRONT_ALIGN_MM 75.0f

/* Only align when the front reading is at or below this.
 *
 * Sits deliberately between the two cases. Starting a move into a cell that
 * HAS a far wall, the sensor reads about 87 + 192 = 279 mm. Into a cell that
 * does not, the nearest wall is another cell further and reads about 471. At
 * 350 the first engages and the second does not, with wide margin either
 * side, so no extra logic is needed to decide whether the wall is in the cell
 * the robot is entering. */
#define WALL_FRONT_ALIGN_RANGE_MM 350U

/* Largest retarget the alignment may apply, in cm.
 *
 * A second guard behind the range test. A genuine correction is the size of
 * the odometry scatter, a few cm at most; anything larger means the reading
 * was not the wall this move is aiming at, or the sensor is lying. Refusing
 * to act on it leaves the move on plain odometry, which is where it started,
 * rather than steering it somewhere confidently wrong. */
#define WALL_FRONT_ALIGN_MAX_CM 4.0f

/* ------------------------- BREAKAWAY PULSE ------------------------------ */
/*                                                                          */
/* The oldest unsolved problem on this robot, finally addressed.            */
/*                                                                          */
/* Static friction here needs more than 140 command units to break. Once    */
/* moving, far less sustains it. A proportional controller cannot deliver   */
/* that near the target, because its output shrinks with exactly the error  */
/* it is trying to close -- so a move that comes to rest slightly short can */
/* never restart, and simply waits out CONTROL_MOVE_TIMEOUT_MS.             */
/*                                                                          */
/* Caught red-handed: a move froze at 17.40 of 19.20 cm with the command    */
/* sitting at 36.0 units, which is exactly STRAIGHT_DIST_KP * 1.80 cm. It   */
/* held that for five and a half seconds and went nowhere. The stiction     */
/* floor did not apply, because it is gated on reference velocity and the   */
/* profile had already finished -- and at 45 units it would not have helped */
/* anyway.                                                                  */
/*                                                                          */
/* AMPLITUDE BREAKS STATIC FRICTION, NOT PATIENCE. An integrator ramping    */
/* through a range where the wheel cannot move just arrives late. A short   */
/* full-scale pulse gets the wheel over the hump, and the ordinary feedback */
/* then has only kinetic friction to fight.                                 */

/* Travel below this rate (cm/s) counts as "not moving" for the detector. */
#define STRAIGHT_BREAKAWAY_RATE_CMS 0.4f

/* Consecutive cycles the stall must hold before a pulse is fired. At
 * CONTROL_SAMPLE_TIME_S this is a dwell, and it is what stops the detector
 * arming during the ordinary deceleration into the target. */
#define STRAIGHT_BREAKAWAY_CYCLES 12U

/* How long a pulse lasts, in ms. Long enough to break the wheel free, short
 * enough that it adds little momentum -- this is a nudge, not a move. Too
 * long and the robot lurches past the target and has to come back, which on
 * this drivetrain is the same problem mirrored. */
#define STRAIGHT_BREAKAWAY_MS 40U

/* Most pulses one move may fire. A move that needs several is not suffering
 * from stiction, it is jammed against something, and hammering it at full
 * scale will not help. Let the timeout report the failure instead. */
#define STRAIGHT_BREAKAWAY_MAX 3U

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
/* HELD AT 1.5. It was briefly taken to 2.5 alongside the breakaway pulse and
 * put back deliberately, and the reasoning is worth keeping.
 *
 * Widening the band would have made the last failure pass. It stopped 1.80 cm
 * short, so at 2.5 it simply completes. But it completes because the band was
 * moved to fit the failure, not because anything got better -- the robot is
 * still stuck, and the pulse that exists to unstick it never fires, because
 * being outside tolerance is precisely what arms it. A wider band does not
 * solve the problem, it hides the evidence that the problem is still there.
 *
 * TURN_TOLERANCE_DEG was widened on the opposite reasoning and that difference
 * matters. There the robot had ALREADY finished the move under control and
 * simply stopped a few degrees out, with proven machinery downstream to absorb
 * it. Here the robot has stopped dead and cannot restart. One is a landing
 * slightly off the mark; the other is a failure to move at all.
 *
 * So this stays tight enough that a stuck robot is reported as stuck. If the
 * pulse works, these moves complete on their own; if it does not, that shows
 * up as a timeout with sl_breakaway_count at its limit, which is the honest
 * answer and the one that says what to fix next.
 *
 * Previous note, from when it went 0.7 -> 1.5:
 *
 * There is NO SECOND CHANCE on a straight move: the profile brings the robot
 * to rest, and from rest it cannot restart at the command a sub-centimetre
 * error produces. So the move has to land inside the band first time, and a
 * band narrower than the landing scatter just guarantees a timeout. */
#define STRAIGHT_TOLERANCE_CM 1.5f

/* How close (degrees of fused yaw) counts as "arrived" for turns. */
/* WIDENED AGAIN, 2.0 -> 5.0, and this time on the architecture rather than on
 * the controller's repeatability.
 *
 * THE HEADING TARGET IS ABSOLUTE AND CONTINUOUS. Every turn aims at an exact
 * multiple of 90 in a heading that accumulates across the whole run, so a turn
 * that finishes 4 degrees short does not push the next one 4 degrees off -- the
 * next move inherits the gap as an ordinary setpoint error and the straight
 * line's own yaw loop closes it. Errors do not compound here; they are handed
 * forward and paid off. That was true when the band was 2.0 as well, but there
 * was no evidence yet that the machinery downstream actually worked. There is
 * now: fused heading hold, two-wall lateral centring, front-wall alignment and
 * the drift bleed all measurably do their job.
 *
 * WHAT IT COSTS. An inherited heading error becomes lateral drift over the
 * following cell, 192*sin(e), of which the lateral loop removes about 80%:
 *
 *     2 deg ->  6.7 mm of drift, ~1.3 mm surviving
 *     5 deg -> 16.7 mm of drift, ~3.2 mm surviving
 *     8 deg -> 26.7 mm of drift, ~5.1 mm surviving
 *
 * Against roughly 35 mm of side clearance, 5 degrees is comfortable and 8 is
 * not somewhere to go.
 *
 * WHAT IT MUST STILL CATCH. Every turn miss recorded on this robot falls into
 * two clearly separated groups: 3.67, 4.25 and 5.26 degrees for turns that
 * completed their profile and simply stopped outside the band, against 9.53 and
 * 31.25 for turns that physically jammed against a wall. A 5 degree band
 * accepts the first group and still rejects the second, which is the
 * distinction worth making -- an overshoot is absorbed downstream, a jam means
 * the robot is not where the map says and the run must stop.
 *
 * Note this does NOT relax the rate check: TURN_SETTLE_RATE_DPS still has to be
 * satisfied, so a robot swinging through the band at speed cannot claim the
 * move. Widening the band only forgives where it stops, never how fast.
 *
 * Previous note, still relevant, from the 1.0 -> 2.0 change: */
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
#define TURN_TOLERANCE_DEG 5.0f

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
