#include "turn_controller.h"

//Turning PID structure variable
static PIDController turn_pid;

//Initialise turn state variable
static TurnState_t state;

static uint32_t pid_last_time;
static uint32_t turn_start_time;
static float pid_sample_time_s;
static uint16_t settle_counter;

//Debugging
float turn_target_distance;
float turn_current_distance;
float turn_basespeed;

//Initialise the controller using the gains from control_config.h
void TurnController_Init(void) {

    //Initialise dependencies
    Encoders_Init();
    MotorDriver_Enable();

    //Turn PID: differential wheel arc (cm) -> turn speed
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

    state = TURN_IDLE;
}

//Helper function to reset PID values
static void resetPID(void) {

    PIDController_Init(&turn_pid);

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

//Helper function to update PID controller
//direction: +1 for a left (anticlockwise) turn, -1 for a right (clockwise) turn
static void updatePID(float target_angle, float direction) {

    //Check if controller is running
    if (state != TURN_RUNNING) {
        return;
    }

    uint32_t current_time = HAL_GetTick();

    //Safety timeout so a stalled robot cannot spin here forever
    if (current_time - turn_start_time >= CONTROL_MOVE_TIMEOUT_MS) {

        state = TURN_TIMEOUT;
        Motor_Brake();
        return;
    }

    if (current_time - pid_last_time >= (uint32_t)(pid_sample_time_s * 1000.0f)) {

        //Update the last time
        pid_last_time = current_time;

        // Update encoder readings
        Encoders_Update();

        //Calculate the target arc each wheel must sweep, signed by direction.
        //A left turn drives the right wheel forward and the left wheel back.
        float target_distance = PI * ROBOT_WHEEL_BASE_CM * (target_angle / 360.0f) * direction;

        //Calcuate the current differential arc
        float current_distance = (Encoder_getRightDistance() - Encoder_getLeftDistance()) / 2.0f;

        //Debugging
        turn_target_distance = target_distance;
        turn_current_distance = current_distance;

        //Check whether the target is reached, and stay there a few cycles so
        //we do not declare success while coasting through it
        if (fabsf(current_distance - target_distance) < TURN_TOLERANCE_CM) {

            settle_counter++;

            if (settle_counter >= CONTROL_SETTLE_CYCLES) {

                //Set state to completed
                state = TURN_COMPLETED;

                //Stop motors completely
                Motor_Brake();

                return;
            }
        }
        else {
            settle_counter = 0;
        }

        //Turn PID calculations. The output is already signed by the error, so
        //a left turn yields a positive command and a right turn a negative one.
        float basespeed = PIDController_Update(&turn_pid, target_distance, current_distance);

        //Overcome gearbox stiction near the target
        basespeed = applyMinSpeed(basespeed);

        turn_basespeed = basespeed;

        //Pivot in place: wheels counter-rotate. Positive basespeed spins the
        //robot anticlockwise (right wheel forward, left wheel backward).
        Motor_runSignedSpeed(-basespeed, basespeed);
    }
}

//Helper to reset the state
static void resetTurnState(void) {

    //Reset encoders and PID controllers
    Encoders_Reset();
    resetPID();

    turn_start_time = HAL_GetTick();

    //Set running state
    state = TURN_RUNNING;
}

//Shared blocking runner. Returns 1 on success, 0 on timeout.
static uint8_t runTurn(float angle_deg, float direction) {

    //Check whether the controller is in idle state
    if (state != TURN_IDLE) {

        return 0;
    }

    resetTurnState();

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

        updatePID(angle_deg, direction);
    }
}

uint8_t turnLeftAngle(float angle_deg) {

    return runTurn(angle_deg, 1.0f);
}

uint8_t turnRightAngle(float angle_deg) {

    return runTurn(angle_deg, -1.0f);
}
