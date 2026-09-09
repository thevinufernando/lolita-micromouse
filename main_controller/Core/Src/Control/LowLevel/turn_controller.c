#include "turn_controller.h"

#define DEG_TO_RAD_F (PI / 180.0f)
#define RAD_TO_DEG_F (180.0f / PI)

//Turning PID structure variable
static PIDController turn_pid;

//Yaw estimator
static EKF_t yaw_ekf;

//Initialise turn state variable
static TurnState_t state;

static uint32_t pid_last_time;
static uint32_t turn_start_time;
static float pid_sample_time_s;
static uint16_t settle_counter;

//DWT cycle stamp of the last EKF prediction
static uint32_t last_predict_cycles;

//Set once at init; drives the encoder-only fallback
static uint8_t imu_ok = 0U;

/* Debugging / live-watch.
 *
 * These exist only to be observed from outside the firmware (ST-Link live
 * watch, or a raw SWD memory read), so they are volatile: without it the
 * compiler is entitled to keep them in registers and elide the stores, which
 * costs nothing at Debug -O0 but can hand back stale values in Release. */
volatile float turn_target_yaw_deg;
volatile float turn_fused_yaw_deg;
volatile float turn_encoder_yaw_deg;
volatile float turn_yaw_error_deg;   /* control error: target - fused, deg   */
volatile float turn_fusion_gap_deg;  /* fused - encoder, deg (ObserveYaw)    */
volatile float turn_gyro_rate_dps;
volatile float turn_gyro_bias_dps;
volatile float turn_basespeed;
volatile uint32_t turn_predict_count;
volatile uint32_t turn_update_count;
volatile uint32_t turn_reject_count;

/* Gyro reads that failed mid-flight. Each one silently costs the prediction
 * step an integration interval, so a non-zero value here means fused yaw is
 * under-integrating -- which looks exactly like encoder over-read from wheel
 * slip. Check this before blaming slip for a fused/encoder disagreement. */
volatile uint32_t turn_imu_fail_count;

/* Outcome of the startup gyro bias calibration. See TurnBiasCalStatus_t. */
volatile TurnBiasCalStatus_t turn_bias_cal_status = TURN_BIAS_NOT_RUN;

/* How many sweeps the calibration needed (1 = clean first try) and the worst
 * single sample seen on the last sweep. If peak sits just above
 * IMU_GYRO_BIAS_MAX_DPS the threshold is too tight; if it is far above, the
 * robot really was moving. */
volatile uint32_t turn_bias_cal_attempts;
volatile float    turn_bias_cal_peak_dps;


/* ------------------------------------------------------------------------
 * Yaw from wheel odometry.
 *
 * For a pivot, the wheels counter-rotate: the right wheel sweeps +d and the
 * left -d, so the differential travel is (right - left) and the rotation is
 * that divided by the wheel base. Units cancel, giving radians.
 * Positive = anticlockwise = left turn, matching the gyro sign convention.
 * ---------------------------------------------------------------------- */
static float encoderYawRad(void)
{
    return (Encoder_getRightDistance() - Encoder_getLeftDistance()) / ROBOT_WHEEL_BASE_CM;
}


/* Read the gyro and run one EKF prediction step, using the true elapsed time
 * measured from the DWT cycle counter rather than an assumed period. */
static void predictStep(void)
{
    if (!imu_ok) return;

    /* Only consume a new sample once per gyro output period. Polling faster
     * would integrate the same reading twice and inflate the rotation. */
    if (DWT_ElapsedUs(last_predict_cycles) < IMU_PREDICT_PERIOD_US) {
        return;
    }

    float dt = (float)DWT_ElapsedUs(last_predict_cycles) * 1.0e-6f;
    last_predict_cycles = DWT_GetCycles();

    float gz_dps = 0.0f;

    if (ICM42688_ReadGyroZ(&gz_dps) != IMU_OK) {
        /* NOTE: last_predict_cycles was already advanced above, so this
         * interval's rotation is dropped rather than carried into the next
         * prediction. Counted here so the loss is at least visible; if this
         * ever reads non-zero, fix the ordering (read the gyro before closing
         * the interval) rather than just watching the counter grow. */
        turn_imu_fail_count++;
        return;
    }

    /* Apply the mounting sign, then convert to rad/s for the filter. */
    float gz_rads = gz_dps * IMU_GYRO_Z_SIGN * DEG_TO_RAD_F;

    EKF_Predict(&yaw_ekf, gz_rads, dt);

    turn_gyro_rate_dps = yaw_ekf.last_rate * RAD_TO_DEG_F;
}


/* Read the encoders and apply the EKF correction step. */
static void correctStep(void)
{
    Encoders_Update();

    float yaw_enc = encoderYawRad();

    turn_encoder_yaw_deg = yaw_enc * RAD_TO_DEG_F;

    EKF_UpdateEncoderYaw(&yaw_ekf, yaw_enc);
}


/* Copy filter diagnostics out for the live-watch panel. */
static void publishTelemetry(void)
{
    turn_fused_yaw_deg  = EKF_GetYawDeg(&yaw_ekf);
    turn_gyro_bias_dps  = EKF_GetGyroBiasDps(&yaw_ekf);
    turn_predict_count  = yaw_ekf.predict_count;
    turn_update_count   = yaw_ekf.update_count;
    turn_reject_count   = yaw_ekf.reject_count;
}


uint8_t TurnController_CalibrateGyroBias(void)
{
    if (!imu_ok) {
        turn_bias_cal_status = TURN_BIAS_IMU_ERROR;
        return 0U;
    }

    /* Retry a motion-rejected sweep rather than giving up on the first one.
     * A single leftover wobble (from setting the robot down, or from the
     * reset button being pressed on the chassis itself) is enough to trip the
     * worst-sample test, and silently falling back to an unestimated bias is
     * worse than spending a few more seconds at boot. */
    for (uint32_t attempt = 0U; attempt < IMU_GYRO_BIAS_MAX_ATTEMPTS; attempt++)
    {
        turn_bias_cal_attempts = attempt + 1U;

        /* Let the chassis stop vibrating before sampling. */
        HAL_Delay(IMU_GYRO_BIAS_SETTLE_MS);

        float sum = 0.0f;
        float max_abs = 0.0f;

        for (uint32_t i = 0U; i < IMU_GYRO_BIAS_SAMPLES; i++)
        {
            float gz_dps = 0.0f;

            if (ICM42688_ReadGyroZ(&gz_dps) != IMU_OK) {
                /* A dead bus will not fix itself, so do not burn the
                 * remaining attempts on it. */
                turn_imu_fail_count++;
                turn_bias_cal_status = TURN_BIAS_IMU_ERROR;
                return 0U;
            }

            sum += gz_dps;

            float abs_dps = fabsf(gz_dps);
            if (abs_dps > max_abs) max_abs = abs_dps;

            /* Pace the sampling to the gyro output rate so we average distinct
             * samples rather than re-reading one value many times. */
            DWT_DelayUs(IMU_PREDICT_PERIOD_US);
        }

        /* If anything moved during calibration the average is meaningless. */
        if (max_abs > IMU_GYRO_BIAS_MAX_DPS) {
            turn_bias_cal_peak_dps = max_abs;   /* what tripped it, for tuning */
            continue;                           /* settle longer and try again */
        }

        float mean_dps = sum / (float)IMU_GYRO_BIAS_SAMPLES;

        /* Store in the same sign convention the prediction step uses. */
        float bias_rads = mean_dps * IMU_GYRO_Z_SIGN * DEG_TO_RAD_F;

        /* A 1000-sample average is a confident estimate, so seed a small
         * variance and let the filter refine it from there. */
        EKF_SetGyroBias(&yaw_ekf, bias_rads, 1.0e-6f);

        turn_gyro_bias_dps     = EKF_GetGyroBiasDps(&yaw_ekf);
        turn_bias_cal_peak_dps = max_abs;
        turn_bias_cal_status   = TURN_BIAS_OK;

        return 1U;
    }

    /* Every attempt saw motion. */
    turn_bias_cal_status = TURN_BIAS_MOVING;

    return 0U;
}


uint8_t TurnController_IsBiasCalibrated(void)
{
    return (turn_bias_cal_status == TURN_BIAS_OK) ? 1U : 0U;
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
    turn_pid.limMinInt = -TURN_INT_LIMIT;
    turn_pid.limMaxInt =  TURN_INT_LIMIT;

    pid_sample_time_s = turn_pid.T;

    PIDController_Init(&turn_pid);

    //Yaw estimator
    EKF_Config_t ekf_cfg;
    EKF_GetDefaultConfig(&ekf_cfg);
    EKF_Init(&yaw_ekf, &ekf_cfg);

    //Bring up the IMU. Failure is not fatal: fall back to encoder-only.
    imu_ok = (ICM42688_Init() == IMU_OK) ? 1U : 0U;

    if (imu_ok) {
        /* A rejected calibration is NOT fatal but is also not harmless: the
         * EKF keeps running with gyro_bias = 0, which quietly degrades every
         * subsequent turn. The outcome lands in turn_bias_cal_status so the
         * caller (and live-watch) can tell the difference between "IMU up and
         * calibrated" and "IMU up but flying blind on bias". */
        (void)TurnController_CalibrateGyroBias();
    }

    last_predict_cycles = DWT_GetCycles();

    publishTelemetry();

    state = TURN_IDLE;

    return imu_ok;
}


uint8_t TurnController_IsImuOk(void)
{
    return imu_ok;
}


float TurnController_GetYawDeg(void)
{
    return EKF_GetYawDeg(&yaw_ekf);
}


float TurnController_GetGyroBiasDps(void)
{
    return EKF_GetGyroBiasDps(&yaw_ekf);
}


void TurnController_ResetYaw(void)
{
    Encoders_Reset();
    EKF_Reset(&yaw_ekf, 0.0f);

    last_predict_cycles = DWT_GetCycles();

    publishTelemetry();
}


//Helper function to reset PID values
static void resetPID(void)
{
    PIDController_Init(&turn_pid);

    pid_last_time = 0;
    settle_counter = 0;
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
    predictStep();

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

    correctStep();
    publishTelemetry();

    float fused_yaw_deg = EKF_GetYawDeg(&yaw_ekf);
    float error_deg = target_yaw_deg - fused_yaw_deg;

    turn_target_yaw_deg = target_yaw_deg;
    turn_yaw_error_deg  = error_deg;

    /* Completion needs BOTH proximity and low rotational speed, so the
     * controller cannot declare success while coasting through the target.
     * Without an IMU there is no rate signal, so fall back to position only. */
    uint8_t within_tolerance = (fabsf(error_deg) < TURN_TOLERANCE_DEG);
    uint8_t settled = imu_ok ? (fabsf(turn_gyro_rate_dps) < TURN_SETTLE_RATE_DPS) : 1U;

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

    //Turn PID on fused yaw. Output is signed by the error: positive drives
    //the robot anticlockwise, negative clockwise.
    //
    //Always evaluated, even inside the deadband below, so the integrator and
    //the derivative filter stay in step with the real error instead of seeing
    //a discontinuity when driving resumes.
    float basespeed = PIDController_Update(&turn_pid, target_yaw_deg, fused_yaw_deg);

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
    Encoders_Reset();
    EKF_Reset(&yaw_ekf, 0.0f);
    resetPID();

    last_predict_cycles = DWT_GetCycles();

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
        predictStep();

        uint32_t now = HAL_GetTick();

        if (now - last_correct >= (uint32_t)(pid_sample_time_s * 1000.0f))
        {
            last_correct = now;

            correctStep();
            publishTelemetry();

            /* Deliberately NOT turn_yaw_error_deg: that one means "target -
             * fused" during a commanded turn. Overloading it here made the
             * same live-watch variable mean two different things depending on
             * which routine last wrote it. */
            turn_fusion_gap_deg = turn_fused_yaw_deg - turn_encoder_yaw_deg;
        }
    }
}
