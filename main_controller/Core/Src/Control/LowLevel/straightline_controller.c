#include "straightline_controller.h"
#include "yaw_estimator.h"
#include "turn_controller.h"
#include "wall_follow.h"
#include "tof_sensors.h"
#include "motion_profile.h"

static Controller_t controller;

static uint32_t pid_last_time = 0;
static uint32_t move_start_time = 0;
static float pid_sample_time_s;
static uint16_t settle_counter = 0;

//Debugging
int32_t left_count;
int32_t right_count;
int16_t left_delta_count;
int16_t right_delta_count;
float left_distance;
float right_distance;
float basespeed;
float steering;

//Initialse the controller using the gains from control_config.h
void StraightlineController_Init(void) {

    //Initialise dependencies
    Encoders_Init();
    MotorDriver_Enable();

    //Distance PID: average travelled distance (cm) -> base speed
    controller.distance_pid.Kp        = STRAIGHT_DIST_KP;
    controller.distance_pid.Ki        = STRAIGHT_DIST_KI;
    controller.distance_pid.Kd        = STRAIGHT_DIST_KD;
    controller.distance_pid.tau       = CONTROL_DERIV_TAU_S;
    controller.distance_pid.T         = CONTROL_SAMPLE_TIME_S;
    controller.distance_pid.limMin    = -CONTROL_MAX_SPEED;
    controller.distance_pid.limMax    =  CONTROL_MAX_SPEED;
    controller.distance_pid.limMinInt = -STRAIGHT_DIST_INT_LIMIT;
    controller.distance_pid.limMaxInt =  STRAIGHT_DIST_INT_LIMIT;

    //Heading PID: (left - right) encoder ticks -> steering correction
    controller.straight_pid.Kp        = STRAIGHT_HEADING_KP;
    controller.straight_pid.Ki        = STRAIGHT_HEADING_KI;
    controller.straight_pid.Kd        = STRAIGHT_HEADING_KD;
    controller.straight_pid.tau       = CONTROL_DERIV_TAU_S;
    controller.straight_pid.T         = CONTROL_SAMPLE_TIME_S;
    controller.straight_pid.limMin    = -STRAIGHT_HEADING_LIMIT;
    controller.straight_pid.limMax    =  STRAIGHT_HEADING_LIMIT;
    controller.straight_pid.limMinInt = -STRAIGHT_HEADING_INT_LIMIT;
    controller.straight_pid.limMaxInt =  STRAIGHT_HEADING_INT_LIMIT;

    pid_sample_time_s = controller.distance_pid.T;

    //Initialise PID instances
    PIDController_Init(&controller.straight_pid);
    PIDController_Init(&controller.distance_pid);

    controller.state = STRAIGHTLINE_IDLE;
}

//Helper function to reset PID values
static void resetPID(void) {

    PIDController_Init(&controller.straight_pid);
    PIDController_Init(&controller.distance_pid);

    pid_last_time = 0;
    settle_counter = 0;
}

//Apply the stiction floor without changing the sign of the request
static float applyMinSpeed(float speed) {

    if (CONTROL_MIN_MOVE_SPEED <= 0.0f) return speed;

    if (speed > 0.0f && speed < CONTROL_MIN_MOVE_SPEED) {
        return CONTROL_MIN_MOVE_SPEED;
    }
    if (speed < 0.0f && speed > -CONTROL_MIN_MOVE_SPEED) {
        return -CONTROL_MIN_MOVE_SPEED;
    }

    return speed;
}

//Helper function to update PID controllers
static void updatePID(float target_distance, float direction) {

    //Check if controller is running
    if (controller.state != STRAIGHTLINE_RUNNING) {
        return;
    }

    uint32_t current_time = HAL_GetTick();

    //Safety timeout so a stalled robot cannot spin here forever
    if (current_time - move_start_time >= CONTROL_MOVE_TIMEOUT_MS) {

        controller.state = STRAIGHTLINE_TIMEOUT;
        Motor_Brake();
        return;
    }

    if (current_time - pid_last_time >= (uint32_t)(pid_sample_time_s * 1000.0f)) {

        //Update the last time
        pid_last_time = current_time;

        // Update encoder readings
        Encoders_Update();

        //Signed target: negative when reversing
        float signed_target = target_distance * direction;
        float measured = Encoder_getAverageDistance();

        //Check whether target distance is reached, and stay there a few
        //cycles so we do not declare success while coasting through it
        uint8_t within_tolerance =
            (fabsf(measured - signed_target) < DISTANCE_TOLERANCE_CM);

        if (within_tolerance) {

            settle_counter++;

            if (settle_counter >= CONTROL_SETTLE_CYCLES) {

                //Set state to completed
                controller.state = STRAIGHTLINE_COMPLETED;

                //Stop motors completely
                Motor_Brake();

                return;
            }
        }
        else {
            settle_counter = 0;
        }

        //Debugging
        left_count = Encoder_getLeftCount();
        right_count = Encoder_getRightCount();
        left_delta_count = Encoder_getLeftDeltaCount();
        right_delta_count = Encoder_getRightDeltaCount();
        left_distance = Encoder_getLeftDistance();
        right_distance = Encoder_getRightDistance();

        //Distance PID calculations(update)
        basespeed = PIDController_Update(&controller.distance_pid, signed_target, measured);

        //Straightline PID calculations, measurement in raw encoder ticks
        float straightline_measurement = (float)(Encoder_getLeftCount() - Encoder_getRightCount());

        //Straightline PID update
        steering = PIDController_Update(&controller.straight_pid, 0.0f, straightline_measurement);

        /* ---------------- terminal deadband ----------------
         * Once inside the tolerance band, stop driving and brake instead.
         *
         * Without this, applyMinSpeed() below floors the command to
         * +/-CONTROL_MIN_MOVE_SPEED, so the controller kept shoving the robot
         * at full stiction-breaking torque for the whole CONTROL_SETTLE_CYCLES
         * window it was supposed to be settling in. That impulse is far
         * coarser than STRAIGHT_TOLERANCE_CM, so it routinely knocked the
         * robot straight back out of the band it had just reached -- a limit
         * cycle that ends in CONTROL_MOVE_TIMEOUT_MS rather than a completed
         * move.
         *
         * This is the same defect that was removed from turn_controller.c,
         * where it was the single biggest cause of failed moves. The ratio is
         * worse here: 45 speed units against a 0.7 cm band.
         *
         * Both PIDs are still evaluated above, deliberately, so the
         * integrator and the derivative filter stay in step with the real
         * error instead of seeing a discontinuity when driving resumes. */
        if (within_tolerance) {

            basespeed = 0.0f;
            steering  = 0.0f;
            Motor_Brake();

            return;
        }

        //Overcome gearbox stiction near the target
        float commanded = applyMinSpeed(basespeed);

        // Calculate motor speeds. Steering is differential, so it is added to
        // one wheel and subtracted from the other.
        float left_speed = commanded + steering;
        float right_speed = commanded - steering;

        // Set motor speeds. The signed interface resolves per-wheel direction,
        // so a negative command reverses that wheel instead of wrapping around.
        Motor_runSignedSpeed(left_speed, right_speed);
    }
}

//Helper to reset the state
static void resetStraightlineState(void) {

    //Reset encoders and PID controllers
    Encoders_Reset();
    resetPID();

    move_start_time = HAL_GetTick();

    //Set running state
    controller.state = STRAIGHTLINE_RUNNING;
}

//Shared blocking runner. Returns 1 on success, 0 on timeout.
static uint8_t runDistance(float distance_cm, float direction) {

    //Check whether the controller is in idle state
    if (controller.state != STRAIGHTLINE_IDLE) {

        return 0;
    }

    resetStraightlineState();

    //Run untill the distance is reached
    while (1) {

        //Check whether the controller is finished
        if (controller.state == STRAIGHTLINE_COMPLETED) {

            controller.state = STRAIGHTLINE_IDLE;
            return 1;
        }

        if (controller.state == STRAIGHTLINE_TIMEOUT) {

            controller.state = STRAIGHTLINE_IDLE;
            return 0;
        }

        updatePID(distance_cm, direction);
    }
}

uint8_t runForwardDistance(float distance_cm) {

    return runDistance(distance_cm, 1.0f);
}

uint8_t runBackwardDistance(float distance_cm) {

    return runDistance(distance_cm, -1.0f);
}


/* ==========================================================================
 *            FUSED FORWARD MOVE: gyro heading + one-wall centring
 * ======================================================================== */

static PIDController yaw_pid;

volatile float    sl_yaw_target_deg;
volatile float    sl_yaw_error_deg;
volatile float    sl_steering;
volatile float    sl_basespeed;
volatile uint32_t sl_sat_cycles;
volatile float    sl_ref_cm;
volatile float    sl_align_delta_cm;
volatile uint8_t  sl_align_applied;
volatile uint32_t sl_breakaway_count;

volatile StraightTrace_t tm_sl_trace[SL_TRACE_CAPACITY];
volatile uint32_t        tm_sl_trace_count;


/* Steering priority allocation.
 *
 * base + steer can exceed the motor range, and letting the driver clip each
 * wheel independently silently converts a pure steering command into a net
 * speed change -- the robot stops turning as hard as it was told to, exactly
 * when it is going fastest and needs it most. Reducing the COMMON MODE
 * instead preserves the differential, so the robot gives up speed rather
 * than giving up steering. */
static void allocate(float base, float steer, float *left, float *right)
{
    if (steer >  STRAIGHT_YAW_LIMIT) steer =  STRAIGHT_YAW_LIMIT;
    if (steer < -STRAIGHT_YAW_LIMIT) steer = -STRAIGHT_YAW_LIMIT;

    float headroom = CONTROL_MAX_SPEED - fabsf(steer);

    if (headroom < 0.0f) headroom = 0.0f;

    if (base >  headroom) { base =  headroom; sl_sat_cycles++; }
    if (base < -headroom) { base = -headroom; sl_sat_cycles++; }

    sl_basespeed = base;
    sl_steering  = steer;

    /* !! SIGN !! Positive steer means turn LEFT (anticlockwise, +yaw), and by
     * this project's convention that is right wheel forward, left wheel back.
     * So steer SUBTRACTS from the left wheel. Getting this backwards turns the
     * heading loop into positive feedback: a robot drifting clockwise gets
     * steered further clockwise. Observed as 25-30 degrees of runaway in the
     * first arena run. Cross-check against turn_controller's
     * Motor_runSignedSpeed(-basespeed, +basespeed). */
    *left  = base - steer;
    *right = base + steer;
}


/* Shared implementation of every fused straight move.
 *
 * `front_target_mm` is the front-sensor reading the move should END at, or 0
 * to run on odometry alone. See the FRONT-WALL ALIGNMENT block in
 * control_config.h for what it is for. */
static uint8_t runFused(float distance_cm, float front_target_mm)
{
    if (controller.state != STRAIGHTLINE_IDLE) {
        return 0;
    }

    /* Encoders FIRST, then re-base the estimator onto the current yaw. REBASE,
     * not reset: the heading this move inherits from the last turn is exactly
     * what it exists to correct. See yaw_estimator.h. */
    Encoders_Reset();
    YawEstimator_RebaseEncoders();
    WallFollow_Reset();

    PIDController_Init(&controller.distance_pid);
    PIDController_Init(&yaw_pid);

    yaw_pid.Kp = STRAIGHT_YAW_KP;
    yaw_pid.Ki = STRAIGHT_YAW_KI;
    yaw_pid.Kd = STRAIGHT_YAW_KD;
    yaw_pid.tau = CONTROL_DERIV_TAU_S;
    yaw_pid.T = CONTROL_SAMPLE_TIME_S;
    yaw_pid.limMin = -STRAIGHT_YAW_LIMIT;
    yaw_pid.limMax =  STRAIGHT_YAW_LIMIT;
    yaw_pid.limMinInt = -STRAIGHT_YAW_INT_LIMIT;
    yaw_pid.limMaxInt =  STRAIGHT_YAW_INT_LIMIT;

    /* Same trap as the turn controller: yaw is continuous, so a zeroed
     * derivative history makes the first cycle see a step of the whole
     * accumulated heading. */
    yaw_pid.prevMeasurement = YawEstimator_GetYawDeg();

    settle_counter    = 0;
    sl_sat_cycles     = 0;
    tm_sl_trace_count = 0U;

    MotionProfile_t dist_profile;
    MotionProfile_Init(&dist_profile, distance_cm,
                       STRAIGHT_PROFILE_MAX_CMS, STRAIGHT_PROFILE_ACCEL_CMS2);

    /* The endpoint, which front-wall alignment may move once. align_offset_cm
     * translates the profile by the same amount so its ramps still land on the
     * new endpoint; translation leaves velocity and acceleration untouched. */
    float   target_cm        = distance_cm;
    float   align_offset_cm  = 0.0f;
    uint8_t align_tried      = (front_target_mm > 0.0f) ? 0U : 1U;

    sl_align_delta_cm = 0.0f;
    sl_align_applied  = 0U;

    /* Breakaway detector state. See the BREAKAWAY PULSE block in
     * control_config.h for why a proportional controller cannot restart this
     * drivetrain on its own. */
    float    prev_measured   = 0.0f;
    uint32_t stall_cycles    = 0;
    uint32_t pulse_until_ms  = 0;

    sl_breakaway_count = 0U;

    uint32_t start_ms = HAL_GetTick();
    uint32_t last_pid = 0;
    uint32_t tof_div  = 0;
    float    tilt_deg = 0.0f;

    controller.state = STRAIGHTLINE_RUNNING;

    while (1) {

        YawEstimator_Predict();

        uint32_t now = HAL_GetTick();

        if (now - start_ms >= CONTROL_MOVE_TIMEOUT_MS) {
            Motor_Brake();
            controller.state = STRAIGHTLINE_IDLE;
            return 0;
        }

        if (now - last_pid < (uint32_t)(CONTROL_SAMPLE_TIME_S * 1000.0f)) {
            continue;
        }

        last_pid = now;

        YawEstimator_Correct();
        YawEstimator_PublishTelemetry();

        float measured   = Encoder_getAverageDistance();
        float elapsed_s  = (float)(now - start_ms) * 0.001f;
        float ref_pos    = MotionProfile_Position(&dist_profile, elapsed_s)
                           + align_offset_cm;
        float ref_vel    = MotionProfile_Velocity(&dist_profile, elapsed_s);

        sl_ref_cm = ref_pos;

        if (fabsf(measured - target_cm) < DISTANCE_TOLERANCE_CM) {
            settle_counter++;
            if (settle_counter >= CONTROL_SETTLE_CYCLES) {
                Motor_Brake();
                controller.state = STRAIGHTLINE_IDLE;
                return 1;
            }
        }
        else {
            settle_counter = 0;
        }

        /* ToF at a fraction of the control rate. Sampling faster than the
         * sensor produces just spends I2C time on a stale measurement, and
         * that time is taken out of the 1 kHz gyro loop. */
        if (++tof_div >= STRAIGHT_TOF_DIVIDER) {
            tof_div = 0;
            ToF_Measurement_t m[TOF_SENSOR_COUNT];
            (void)ToF_ReadAll(m);

            tilt_deg = WallFollow_Update(m);

            /* FRONT-WALL ALIGNMENT, attempted only in the first quarter of the
             * move and applied at most once.
             *
             * Early on purpose. The correction translates the profile, so the
             * reference steps by up to WALL_FRONT_ALIGN_MAX_CM the moment it
             * lands. During the opening ramp the command is large and clipped
             * anyway and the step vanishes into it; applied near the end it
             * would arrive after the robot has already braked, and closing a
             * few cm from rest is exactly what this drivetrain cannot do. */
            if (!align_tried
                && fabsf(measured) < 0.25f * fabsf(target_cm)) {

                uint16_t f = m[TOF_FRONT].distance_mm;

                if (m[TOF_FRONT].valid
                    && f != TOF_DISTANCE_INVALID
                    && f <= WALL_FRONT_ALIGN_RANGE_MM) {

                    /* Signed travel still needed for the sensor to read the
                     * target. Works in BOTH directions unchanged: driving at a
                     * wall the reading falls, so this is positive; backing away
                     * it rises, so this is negative. */
                    float to_go_cm   = ((float)f - front_target_mm) * 0.1f;
                    float new_target = measured + to_go_cm;
                    float delta      = new_target - target_cm;

                    if (fabsf(delta) <= WALL_FRONT_ALIGN_MAX_CM) {
                        target_cm        += delta;
                        align_offset_cm  += delta;
                        sl_align_delta_cm = delta;
                        sl_align_applied  = 1U;
                        align_tried       = 1U;
                    }
                    /* Out of range: leave align_tried clear and look again on
                     * the next sweep. The wall may simply not be the one this
                     * move is aiming at yet. */
                }
            }
        }

        /* The cascade: lateral error tilts the HEADING TARGET, and the
         * heading loop closes on that. The drift bleed rides along on the
         * same signal. */
        float heading_target = TurnController_GetHeadingTargetDeg()
                               + WallFollow_GetDriftDeg();

        sl_yaw_target_deg = heading_target + tilt_deg;

        float yaw = YawEstimator_GetYawDeg();

        sl_yaw_error_deg = sl_yaw_target_deg - yaw;

        /* Track the profile, not the endpoint. Feedforward supplies the
         * command the move needs; feedback only trims. */
        float base = STRAIGHT_FF_GAIN * ref_vel
                     + PIDController_Update(&controller.distance_pid, ref_pos, measured);
        float steer = PIDController_Update(&yaw_pid, sl_yaw_target_deg, yaw);

        /* Stiction floor, NEVER during deceleration.
         *
         * The floor exists to raise a command too small to move the robot. It
         * must not raise a command that is deliberately small because the
         * profile is braking. The trace caught it doing exactly that: at
         * t=1512 the distance PID wanted to slow down and applyMinSpeed forced
         * the command back up to +45, driving the robot straight through the
         * target to 21.6 cm.
         *
         * Gating on reference ACCELERATION rather than velocity is what makes
         * this correct: velocity is still large during the braking ramp, which
         * is precisely when flooring is most harmful. turn_controller already
         * guards this with TURN_PROFILE_FLOOR_DPS; the straight path never got
         * the same gate. */
        float ref_acc = MotionProfile_Acceleration(&dist_profile, elapsed_s);

        /* The rule is "floor the command unless the profile is braking", and
         * braking is exactly when reference velocity and acceleration disagree
         * in sign. Written that way rather than as `ref_acc >= 0` because the
         * two are equivalent going forwards and only one of them stays true if
         * this ever has to run a move in the other direction. */
        if (fabsf(measured - target_cm) < DISTANCE_TOLERANCE_CM) {
            base = 0.0f;
        }
        else if (ref_acc * ref_vel >= 0.0f && fabsf(ref_vel) > 1.0f) {
            base = applyMinSpeed(base);
        }

        /* BREAKAWAY. Armed only once the profile has finished: while it is
         * still running a slow patch is the controller's problem to solve, and
         * a full-scale pulse mid-move would wreck the tracking. After it ends,
         * a robot that is outside tolerance and not moving is stuck, and no
         * amount of waiting fixes that.
         *
         * The command is applied at full scale in the direction of the
         * remaining error, briefly. It is a nudge to get the wheel over static
         * friction, after which the ordinary feedback has only kinetic
         * friction to work against. */
        const float travelled = fabsf(measured - prev_measured);
        const float still_threshold =
            STRAIGHT_BREAKAWAY_RATE_CMS * CONTROL_SAMPLE_TIME_S;

        prev_measured = measured;

        const uint8_t profile_done = (elapsed_s >= MotionProfile_Duration(&dist_profile));
        const uint8_t short_of_it  = (fabsf(measured - target_cm) >= DISTANCE_TOLERANCE_CM);

        if (profile_done && short_of_it && travelled < still_threshold) {
            stall_cycles++;
        }
        else {
            stall_cycles = 0;
        }

        if (stall_cycles >= STRAIGHT_BREAKAWAY_CYCLES
            && sl_breakaway_count < STRAIGHT_BREAKAWAY_MAX) {

            pulse_until_ms = now + STRAIGHT_BREAKAWAY_MS;
            stall_cycles   = 0;
            sl_breakaway_count++;
        }

        if (now < pulse_until_ms) {
            base = (target_cm > measured) ? CONTROL_MAX_SPEED : -CONTROL_MAX_SPEED;
        }

        float left, right;
        allocate(base, steer, &left, &right);

        Motor_runSignedSpeed(left, right);

        if (tm_sl_trace_count < SL_TRACE_CAPACITY) {
            volatile StraightTrace_t *tr = &tm_sl_trace[tm_sl_trace_count];
            tr->t_s     = elapsed_s;
            tr->ref_cm  = ref_pos;
            tr->act_cm  = measured;
            tr->base    = sl_basespeed;
            tr->steer   = sl_steering;
            tr->yaw_err = sl_yaw_error_deg;
            tm_sl_trace_count++;
        }
    }
}


uint8_t runForwardFused(float distance_cm)
{
    /* Aims to finish WALL_FRONT_ALIGN_MM from a wall ahead, when there is one.
     * With no wall in range the alignment never fires and the move is exactly
     * what it was before: odometry against a trapezoidal profile. */
    return runFused(distance_cm, WALL_FRONT_ALIGN_MM);
}
