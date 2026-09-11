#include "turn_controller.h"
#include "motion_profile.h"

#define DEG_TO_RAD_F (PI / 180.0f)
#define RAD_TO_DEG_F (180.0f / PI)

//Turning PID structure variable
static PIDController turn_pid;

//Initialise turn state variable
static TurnState_t state;

static uint32_t pid_last_time;
static uint32_t turn_start_time;
static float pid_sample_time_s;
static uint16_t settle_counter;

/* Consecutive cycles spent stationary while still short of the target.
 * Gates the integrator's stall-recovery authority; see TURN_INT_LIMIT_MOVING. */
static uint16_t stall_counter;

/* Reference trajectory for the move in progress. */
static MotionProfile_t turn_profile;

/* Yaw the current profile was built from; the profile is relative to it. */
static float turn_profile_start_deg;

/* Debugging / live-watch.
 *
 * These exist only to be observed from outside the firmware (ST-Link live
 * watch, or a raw SWD memory read), so they are volatile: without it the
 * compiler is entitled to keep them in registers and elide the stores, which
 * costs nothing at Debug -O0 but can hand back stale values in Release. */
volatile float turn_target_yaw_deg;
volatile float turn_yaw_error_deg;   /* control error: target - fused, deg   */
volatile float turn_basespeed;

/* Integrator state, exposed so a stalled or overshooting move can be told
 * apart from the outside. turn_int_limit shows which of the two clamps is
 * currently in force, and turn_stall_boosts counts cycles spent in the
 * raised one -- 0 across a whole run means the stall path never armed. */
volatile float    turn_integrator;
volatile float    turn_int_limit;
volatile uint32_t turn_stall_boosts;

/* Profile tracking. turn_profile_err_deg is the error the PID actually sees
 * (reference minus actual), which is NOT the same as turn_yaw_error_deg --
 * that one is distance from the FINAL target and is legitimately large
 * mid-move. Watch the first to judge tracking, the second to judge the result.
 *
 * turn_ff_cmd and turn_fb_cmd split the command into its feedforward and
 * feedback halves. During the cruise phase fb should hover near zero; if it
 * sits consistently one way, TURN_FF_GAIN is wrong. */
volatile float turn_profile_ref_deg;
volatile float turn_profile_err_deg;
volatile float turn_ff_cmd;
volatile float turn_fb_cmd;
volatile float turn_profile_duration_s;

/* Where the robot is SUPPOSED to be pointing, accumulated across the whole
 * run. Moves by exactly +/-90 per turn and is never reset, so it stays an
 * exact multiple of 90 forever while the estimate drifts around it.
 *
 * The difference between this and yaw is the accumulated heading error, and
 * making it visible is the entire reason yaw is no longer zeroed per move.
 * A turn that finishes 2 degrees short leaves that 2 degrees here, where the
 * next move inherits it as an ordinary setpoint error instead of losing it. */
volatile TurnTrace_t tm_turn_trace[TURN_TRACE_CAPACITY];
volatile uint32_t    tm_turn_trace_count;

volatile float turn_heading_target_deg;
volatile float turn_heading_error_deg;


/* ---- Estimator forwarders ----
 * Kept so existing callers (main.c, the test harness) do not have to care that
 * the estimator moved out. New code should call YawEstimator_* directly. */

uint8_t TurnController_CalibrateGyroBias(void)
{
    return YawEstimator_CalibrateGyroBias();
}


uint8_t TurnController_IsBiasCalibrated(void)
{
    return YawEstimator_IsBiasCalibrated();
}


uint8_t TurnController_Init(void)
{
    //Initialise dependencies
    Encoders_Init();
    MotorDriver_Enable();
    DWT_Timer_Init();

    //Turn PID: fused yaw error (degrees) -> turn speed
    turn_pid.Kp        = TURN_KP;
    turn_pid.Ki        = TURN_KI;
    turn_pid.Kd        = TURN_KD;
    turn_pid.tau       = CONTROL_DERIV_TAU_S;
    turn_pid.T         = CONTROL_SAMPLE_TIME_S;
    turn_pid.limMin    = -CONTROL_MAX_SPEED;
    turn_pid.limMax    =  CONTROL_MAX_SPEED;
    /* Starting clamp only. updateControl() switches between
     * TURN_INT_LIMIT_MOVING and TURN_INT_LIMIT every cycle. */
    turn_pid.limMinInt = -TURN_INT_LIMIT_MOVING;
    turn_pid.limMaxInt =  TURN_INT_LIMIT_MOVING;

    pid_sample_time_s = turn_pid.T;

    PIDController_Init(&turn_pid);

    /* Bring up the shared yaw estimator. This is where the IMU comes up and
     * the stationary bias calibration runs, so THE ROBOT MUST BE STILL.
     * Failure is not fatal: it falls back to encoder-only. */
    uint8_t imu_ok = YawEstimator_Init();

    state = TURN_IDLE;

    return imu_ok;
}


uint8_t TurnController_IsImuOk(void)
{
    return YawEstimator_IsImuOk();
}


float TurnController_GetYawDeg(void)
{
    return YawEstimator_GetYawDeg();
}


float TurnController_GetGyroBiasDps(void)
{
    return YawEstimator_GetGyroBiasDps();
}


void TurnController_ResetYaw(void)
{
    /* Encoders FIRST, then the filter. The estimator's measurement is absolute
     * differential travel since the encoder reset, so the two must move
     * together -- see the pairing rule in yaw_estimator.h.
     *
     * This is the START-FRESH entry point and it discards accumulated heading
     * on purpose. Per-move boundaries must NOT come through here; they use
     * YawEstimator_RebaseEncoders() instead, which keeps the heading. */
    Encoders_Reset();
    YawEstimator_Reset();

    turn_heading_target_deg = 0.0f;
    turn_heading_error_deg  = 0.0f;
}


//Helper function to reset PID values
static void resetPID(void)
{
    PIDController_Init(&turn_pid);

    /* !! Seed the derivative's history with the CURRENT yaw. !!
     *
     * PIDController_Init() zeroes prevMeasurement, which was harmless when yaw
     * was reset to 0 at every move. Now that yaw is continuous the measurement
     * starts at whatever the run has accumulated, so the first cycle saw a step
     * of the entire heading and the derivative term slammed to its limit. The
     * trace caught it: the very first sample of a move at 1710 degrees showed
     * a feedback command of -200, full reverse, fighting the start of the move
     * before decaying over the next few samples. */
    turn_pid.prevMeasurement = YawEstimator_GetYawDeg();
    turn_pid.prevError       = 0.0f;

    pid_last_time = 0;
    settle_counter = 0;
    stall_counter = 0;

    /* Every move starts assumed-moving, so a stall must be re-earned. */
    turn_pid.limMaxInt =  TURN_INT_LIMIT_MOVING;
    turn_pid.limMinInt = -TURN_INT_LIMIT_MOVING;
}


//Apply the stiction floor without changing the sign of the request
static float applyMinSpeed(float speed)
{
    if (CONTROL_MIN_MOVE_SPEED <= 0.0f) return speed;

    if (speed > 0.0f && speed < CONTROL_MIN_MOVE_SPEED) {
        return CONTROL_MIN_MOVE_SPEED;
    }
    if (speed < 0.0f && speed > -CONTROL_MIN_MOVE_SPEED) {
        return -CONTROL_MIN_MOVE_SPEED;
    }

    return speed;
}


/* One iteration of the control loop.
 *
 * `target_yaw_deg` is the FINAL signed target: positive for a left turn,
 * negative for right. The instantaneous setpoint comes from the profile, not
 * from this value. */
static void updateControl(float target_yaw_deg)
{
    //Check if controller is running
    if (state != TURN_RUNNING) {
        return;
    }

    /* --- fast path: EKF prediction from the gyro, ~1 kHz --- */
    YawEstimator_Predict();

    uint32_t current_time = HAL_GetTick();
    float    elapsed_s    = (float)(current_time - turn_start_time) * 0.001f;
    float    duration_s   = MotionProfile_Duration(&turn_profile);

    /* Hard bound on the move: the profile plus a fixed grace period. This
     * replaces CONTROL_MOVE_TIMEOUT_MS as the escape hatch, and unlike an 8
     * second timeout it is reached in the normal course of events rather than
     * only on failure. */
    if (elapsed_s > duration_s + (float)TURN_PROFILE_SETTLE_MS * 0.001f) {

        state = (fabsf(target_yaw_deg - YawEstimator_GetYawDeg()) < TURN_TOLERANCE_DEG)
                    ? TURN_COMPLETED : TURN_TIMEOUT;
        Motor_Brake();
        return;
    }

    /* --- slow path: encoder correction, PID and actuation --- */
    if (current_time - pid_last_time < (uint32_t)(pid_sample_time_s * 1000.0f)) {
        return;
    }

    pid_last_time = current_time;

    YawEstimator_Correct();
    YawEstimator_PublishTelemetry();

    float fused_yaw_deg = YawEstimator_GetYawDeg();

    /* The setpoint the robot is chased toward right now, and the rate the
     * profile says it should be turning at. */
    float ref_pos = turn_profile_start_deg
                    + MotionProfile_Position(&turn_profile, elapsed_s);
    float ref_vel = MotionProfile_Velocity(&turn_profile, elapsed_s);
    float ref_acc = MotionProfile_Acceleration(&turn_profile, elapsed_s);

    float track_error = ref_pos - fused_yaw_deg;
    float final_error = target_yaw_deg - fused_yaw_deg;

    turn_target_yaw_deg  = target_yaw_deg;
    turn_yaw_error_deg   = final_error;
    turn_profile_ref_deg = ref_pos;
    turn_profile_err_deg = track_error;

    uint8_t profile_done    = (elapsed_s >= duration_s);
    uint8_t within_tolerance = (fabsf(final_error) < TURN_TOLERANCE_DEG);
    uint8_t settled = YawEstimator_IsImuOk()
                        ? (fabsf(yaw_gyro_rate_dps) < TURN_SETTLE_RATE_DPS) : 1U;

    /* Finish early only once the profile has actually delivered the rotation.
     * Completing mid-profile would mean stopping short of a move the caller
     * asked for, even if yaw happens to be within tolerance at that instant. */
    if (profile_done && within_tolerance && settled) {

        settle_counter++;

        if (settle_counter >= CONTROL_SETTLE_CYCLES) {

            state = TURN_COMPLETED;
            Motor_Brake();
            return;
        }
    }
    else {
        settle_counter = 0;
    }

    /* ---------------- stall-gated integral authority ----------------
     * Gated on the PROFILE error, not the final-target error. Early in a move
     * the robot is legitimately far from the final target while tracking the
     * reference perfectly, and judging a stall by that distance would arm the
     * boost on every healthy turn. Being stationary while the profile says to
     * move is the actual definition of stalled.
     *
     * Lowering a clamp also SHRINKS an already-wound integrator, because
     * PIDController_Update clamps after integrating, so recovery authority
     * evaporates the moment the wheel starts turning again. */
    if (YawEstimator_IsImuOk() &&
        fabsf(track_error) > TURN_TOLERANCE_DEG &&
        fabsf(yaw_gyro_rate_dps) < TURN_STALL_RATE_DPS) {

        if (stall_counter < TURN_STALL_CYCLES) stall_counter++;
    }
    else {
        stall_counter = 0;
    }

    float int_limit = TURN_INT_LIMIT_MOVING;

    if (stall_counter >= TURN_STALL_CYCLES) {
        int_limit = TURN_INT_LIMIT;
        turn_stall_boosts++;
    }

    turn_pid.limMaxInt =  int_limit;
    turn_pid.limMinInt = -int_limit;
    turn_int_limit     =  int_limit;

    /* Feedforward supplies the command the move needs; feedback only corrects
     * the difference. Without the feedforward this is just a PID chasing a
     * moving target, which is strictly worse than chasing a fixed one. */
    /* Acceleration feedforward, ON THE WAY UP ONLY.
     *
     * The plant-inverse model says less command is needed while decelerating,
     * and mathematically that is right -- but it assumes the motor is the only
     * thing slowing the robot down. On this drivetrain friction is enormous
     * (breakaway is above 140 units), so friction alone brakes harder than the
     * profile asks for, and subtracting command on top of that stalls the
     * robot early.
     *
     * The trace showed it plainly: at the end of the decel ramp the velocity
     * term wanted +27 and the acceleration term wanted -60, for a net
     * feedforward of -33 -- commanding reverse. The tracking lag, flat at
     * ~3 deg through cruise, grew 3.0 -> 5.8 over exactly that stretch. */
    float acc_ff = TURN_FF_ACCEL_GAIN * ref_acc;

    if (ref_acc * ref_vel < 0.0f) {
        acc_ff = 0.0f;
    }

    float ff = TURN_FF_GAIN * ref_vel + acc_ff;
    float fb = PIDController_Update(&turn_pid, ref_pos, fused_yaw_deg);

    turn_integrator = turn_pid.integrator;
    turn_ff_cmd     = ff;
    turn_fb_cmd     = fb;

    if (tm_turn_trace_count < TURN_TRACE_CAPACITY) {
        volatile TurnTrace_t *tr = &tm_turn_trace[tm_turn_trace_count];
        tr->t_s     = elapsed_s;
        tr->ref_deg = ref_pos - turn_profile_start_deg;
        tr->act_deg = fused_yaw_deg - turn_profile_start_deg;
        tr->ff      = ff;
        tr->fb      = fb;
        tm_turn_trace_count++;
    }

    float basespeed = ff + fb;

    /* Terminal deadband, only once the profile is finished. Applying it
     * mid-profile would stop the robot every time it happened to pass through
     * the target on its way to the end of the move. */
    if (profile_done && within_tolerance) {

        turn_basespeed = 0.0f;
        Motor_Brake();

        return;
    }

    /* Stiction floor, only while the profile is genuinely asking for rotation.
     * Below TURN_PROFILE_FLOOR_DPS the profile is winding down on purpose, and
     * forcing the floor there drives the robot through the target -- the exact
     * mechanism behind the old overshoots. */
    if (fabsf(ref_vel) > TURN_PROFILE_FLOOR_DPS || !profile_done) {
        basespeed = applyMinSpeed(basespeed);
    }

    if (basespeed >  CONTROL_MAX_SPEED) basespeed =  CONTROL_MAX_SPEED;
    if (basespeed < -CONTROL_MAX_SPEED) basespeed = -CONTROL_MAX_SPEED;

    turn_basespeed = basespeed;

    //Pivot in place: wheels counter-rotate. Positive basespeed spins the
    //robot anticlockwise (right wheel forward, left wheel backward).
    Motor_runSignedSpeed(-basespeed, basespeed);
}


//Helper to reset the state
static void resetTurnState(void)
{
    /* Encoders FIRST, then re-base -- see the pairing rule in yaw_estimator.h.
     *
     * REBASE, NOT RESET. Zeroing yaw here would throw away the heading error
     * this move inherited, which is precisely the information the next move
     * needs in order to correct it. The wheels restart from zero; the heading
     * estimate does not. */
    Encoders_Reset();
    YawEstimator_RebaseEncoders();
    resetPID();

    tm_turn_trace_count = 0U;   /* the trace holds the most recent move */

    turn_start_time = HAL_GetTick();

    //Set running state
    state = TURN_RUNNING;
}


//Shared blocking runner. Returns 1 on success, 0 on timeout.
static uint8_t runTurn(float angle_deg, float direction)
{
    //Check whether the controller is in idle state
    if (state != TURN_IDLE) {

        return 0;
    }

    /* Absolute heading bookkeeping. The commanded rotation moves the target by
     * exactly the requested amount; what the robot must actually turn is the
     * distance from where it currently believes it is to there, which folds in
     * any error left over from the last move. */
    float start_yaw_deg = YawEstimator_GetYawDeg();

    turn_heading_target_deg += angle_deg * direction;

    float target_yaw_deg = turn_heading_target_deg;
    float sweep_deg      = target_yaw_deg - start_yaw_deg;

    turn_heading_error_deg = sweep_deg - (angle_deg * direction);

    /* Build the trajectory BEFORE resetting the clock, so elapsed time and the
     * profile share an origin. */
    MotionProfile_Init(&turn_profile, sweep_deg,
                       TURN_PROFILE_MAX_DPS, TURN_PROFILE_ACCEL_DPS2);

    turn_profile_start_deg = start_yaw_deg;

    turn_profile_duration_s = MotionProfile_Duration(&turn_profile);

    resetTurnState();

    //Run until the profile completes (plus its bounded grace period)
    while (1) {

        //Check whether the controller is finished
        if (state == TURN_COMPLETED) {

            state = TURN_IDLE;
            return 1;
        }

        if (state == TURN_TIMEOUT) {

            state = TURN_IDLE;
            return 0;
        }

        updateControl(target_yaw_deg);
    }
}


float TurnController_GetHeadingTargetDeg(void)
{
    return turn_heading_target_deg;
}


void TurnController_SetHeadingTargetDeg(float deg)
{
    turn_heading_target_deg = deg;
}


uint8_t turnLeftAngle(float angle_deg)
{
    return runTurn(angle_deg, 1.0f);
}


uint8_t turnRightAngle(float angle_deg)
{
    return runTurn(angle_deg, -1.0f);
}


void TurnController_ObserveYaw(uint32_t duration_ms)
{
    Motor_Brake();

    TurnController_ResetYaw();

    uint32_t start = HAL_GetTick();
    uint32_t last_correct = start;

    while ((HAL_GetTick() - start) < duration_ms)
    {
        YawEstimator_Predict();

        uint32_t now = HAL_GetTick();

        if (now - last_correct >= (uint32_t)(pid_sample_time_s * 1000.0f))
        {
            last_correct = now;

            YawEstimator_Correct();

            /* PublishTelemetry() now computes yaw_fusion_gap_deg itself, so
             * this loop no longer has to. That gap is the slip measurement and
             * it is wanted on every move, not just this observation routine. */
            YawEstimator_PublishTelemetry();
        }
    }
}
