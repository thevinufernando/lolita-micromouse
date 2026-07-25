#include "straightline_controller.h"

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
        if (fabsf(measured - signed_target) < DISTANCE_TOLERANCE_CM) {

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
