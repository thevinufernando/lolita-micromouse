#include "turn_controller.h"

//Turning PID structure variable
static PIDController turn_pid;

//Initialise turn state variable
static TurnState_t state;

static uint32_t pid_last_time;
static float pid_sample_time_s;

//Initialise the controller
void TurnController_Init(PIDController ctrl) {

    //Initialise dependencies
    Encoders_Init();
    MotorDriver_Enable();

    turn_pid = ctrl;
    pid_sample_time_s = ctrl.T;
    
    PIDController_Init(&turn_pid);

    state = TURN_IDLE;

}

//Helper function to reset PID values
static void resetPID() {

    PIDController_Init(&turn_pid);

    pid_last_time = 0;
}

//Helper function to update PID controller
static void updatePID(float target_angle, float direction) {

    //Check if controller is running
    if (state != TURN_RUNNING) {
        return;
    }

    uint32_t current_time = HAL_GetTick();

    if (current_time - pid_last_time >= (uint32_t)(pid_sample_time_s * 1000.0f)) {

        //Update the last time
        pid_last_time = current_time;

        // Update encoder readings
        Encoders_Update();

        //Calculate the target distance
        float target_distance = PI * ROBOT_WHEEL_BASE_CM * (target_angle/360.0f) * direction;

        //Calcuate the current distance
        float current_distance = (Encoder_getRightDistance() + -1.0f * Encoder_getLeftDistance()) / 2;

        if (fabs(current_distance - target_distance) < TURN_TOLERANCE_CM) {

            //Set state to completed
            state = TURN_COMPLETED;

            //Stop motors completely
            MotorForward_runSpeed(0, 0);
            HAL_Delay(20);

            return;
        }

        //Turn PID calculations
        float basespeed = PIDController_Update(&turn_pid, target_distance, current_distance);

        // Clamp motor commands
        if (basespeed > 200) basespeed = 200;
        if (basespeed < -200) basespeed = -200;

        //Set Motor speeds
        if (direction == -1.0f) {
            MotorRightTurn_runSpeed((uint8_t)(basespeed * direction));
        }
        else {
            MotorLeftTurn_runSpeed((uint8_t)basespeed);
        }

    }
}

//Helper to reset the state
static void resetTurnState(void) {

    //Reset encoders and PID controllers
    Encoders_Reset();
    resetPID();

    //Set running state
    state = TURN_RUNNING;
}

void turnLeftAngle(float angle_deg) {

    //Check whether the controller is in idle state
    if (state != TURN_IDLE) {

        return;
    }
    
    resetTurnState();

    //Run untill the distance is reached
    while(1) {
        
        //Check whether the controller is finished
        if (state == TURN_COMPLETED) {

            state = TURN_IDLE;
            return;
        }

        updatePID(angle_deg, 1.0f);
    }
} 

void turnRightAngle(float angle_deg) {

    //Check whether the controller is in idle state
    if (state != TURN_IDLE) {

        return;
    }
    
    resetTurnState();

    //Run untill the distance is reached
    while(1) {
        
        //Check whether the controller is finished
        if (state == TURN_COMPLETED) {

            state = TURN_IDLE;
            return;
        }

        updatePID(angle_deg, -1.0f);
    }
} 