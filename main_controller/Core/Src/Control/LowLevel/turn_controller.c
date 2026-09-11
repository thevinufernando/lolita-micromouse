#include "turn_controller.h"

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
     * together -- see the pairing rule in yaw_estimator.h. */
    Encoders_Reset();
    YawEstimator_Reset();
}


//Helper function to reset PID values
static void resetPID(void)
{
    PIDController_Init(&turn_pid);

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
 * `target_yaw_deg` is signed: positive for a left turn, negative for right. */
static void updateControl(float target_yaw_deg)
{
    //Check if controller is running
    if (state != TURN_RUNNING) {
        return;
    }

    /* --- fast path: EKF prediction from the gyro, ~1 kHz --- */
    YawEstimator_Predict();

    uint32_t current_time = HAL_GetTick();

    //Safety timeout so a stalled robot cannot spin here forever
    if (current_time - turn_start_time >= CONTROL_MOVE_TIMEOUT_MS) {

        state = TURN_TIMEOUT;
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
    float error_deg = target_yaw_deg - fused_yaw_deg;

    turn_target_yaw_deg = target_yaw_deg;
    turn_yaw_error_deg  = error_deg;

    /* Completion needs BOTH proximity and low rotational speed, so the
     * controller cannot declare success while coasting through the target.
     * Without an IMU there is no rate signal, so fall back to position only. */
    uint8_t within_tolerance = (fabsf(error_deg) < TURN_TOLERANCE_DEG);
    uint8_t settled = YawEstimator_IsImuOk()
                    ? (fabsf(yaw_gyro_rate_dps) < TURN_SETTLE_RATE_DPS) : 1U;

    if (within_tolerance && settled) {

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
     * The integrator is wanted for one job only: growing the command until
     * a robot that has stopped short breaks static friction again. It is NOT
     * wanted during the turn proper, where Kp already saturates the output
     * and anything the integrator banks comes back as overshoot.
     *
     * So it gets a small clamp normally, and the large one only after the
     * robot has been measurably stationary AND outside tolerance for
     * TURN_STALL_CYCLES in a row. A healthy turn never satisfies that, so
     * the big limit simply never arms.
     *
     * Lowering a clamp also SHRINKS an already-wound integrator, because
     * PIDController_Update clamps after integrating. Recovery authority
     * therefore evaporates the moment the wheel starts turning, which is the
     * property that stops this from reintroducing the overshoot. */
    if (!within_tolerance && YawEstimator_IsImuOk() &&
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

    //Turn PID on fused yaw. Output is signed by the error: positive drives
    //the robot anticlockwise, negative clockwise.
    //
    //Always evaluated, even inside the deadband below, so the integrator and
    //the derivative filter stay in step with the real error instead of seeing
    //a discontinuity when driving resumes.
    float basespeed = PIDController_Update(&turn_pid, target_yaw_deg, fused_yaw_deg);

    turn_integrator = turn_pid.integrator;

    /* ---------------- terminal deadband ----------------
     * Once inside the tolerance band, stop driving and brake instead.
     *
     * Without this, applyMinSpeed() floors the command to
     * +/-CONTROL_MIN_MOVE_SPEED, so the controller kept kicking the robot at
     * full stiction-breaking torque for the whole CONTROL_SETTLE_CYCLES
     * window it was supposed to be settling in. That impulse is far coarser
     * than the tolerance band, so it would routinely knock the robot straight
     * back out of the band it had just reached -- a limit cycle, felt as
     * vibration and seen in the logs as moves that sat just outside tolerance
     * until CONTROL_MOVE_TIMEOUT_MS fired.
     *
     * Braking here also helps satisfy the TURN_SETTLE_RATE_DPS half of the
     * completion test instead of fighting it. */
    if (within_tolerance) {

        turn_basespeed = 0.0f;
        Motor_Brake();

        return;
    }

    //Overcome gearbox stiction near the target
    basespeed = applyMinSpeed(basespeed);

    turn_basespeed = basespeed;

    //Pivot in place: wheels counter-rotate. Positive basespeed spins the
    //robot anticlockwise (right wheel forward, left wheel backward).
    Motor_runSignedSpeed(-basespeed, basespeed);
}


//Helper to reset the state
static void resetTurnState(void)
{
    //Reset encoders and PID controllers. The EKF's yaw is zeroed but the
    //learned gyro bias is deliberately carried over from previous moves.
    //Encoders FIRST, then the filter -- see yaw_estimator.h.
    Encoders_Reset();
    YawEstimator_Reset();
    resetPID();

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

    resetTurnState();

    float target_yaw_deg = angle_deg * direction;

    //Run untill the angle is reached
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
