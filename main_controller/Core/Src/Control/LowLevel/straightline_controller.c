#include "straightline_controller.h"

static Controller_t controller;

static uint32_t pid_last_time = 0;
static float pid_sample_time_s;

//Debugging
int32_t left_count;
int32_t right_count;
int16_t left_delta_count;
int16_t right_delta_count;
float left_distance;
float right_distance;
float basespeed;
float steering;

//Initialse the controller
void StraightlineController_Init(Controller_t ctrl) {

    //Initialise dependencies
    Encoders_Init();
    MotorDriver_Enable();

    //Initialize PID values
    controller.straight_pid = ctrl.straight_pid;
    controller.distance_pid = ctrl.distance_pid;
    pid_sample_time_s = controller.straight_pid.T;

    //Initialise PID instances
    PIDController_Init(&controller.straight_pid);
    PIDController_Init(&controller.distance_pid);

    controller.state = STRAIGHTLINE_IDLE;
}

//Helper function to reset PID values
static void resetPID() {

    PIDController_Init(&controller.straight_pid);
    PIDController_Init(&controller.distance_pid);

    pid_last_time = 0;
}

//Helper function to update PID controllers
static void updatePID(float target_distance, float direction) {
    
    //Check if controller is running
    if (controller.state != STRAIGHTLINE_RUNNING) {
        return;
    }

    uint32_t current_time = HAL_GetTick();

    if (current_time - pid_last_time >= (uint32_t)(pid_sample_time_s * 1000.0f)) {

        //Update the last time
        pid_last_time = current_time;

        // Update encoder readings
        Encoders_Update();
        
        //Check whether target distance is reached
        if (fabs(Encoder_getAverageDistance() - target_distance * direction) < DISTANCE_TOLERANCE_CM) {

            //Set state to completed
            controller.state = STRAIGHTLINE_COMPLETED;
            
            //Stop motors completely
            MotorForward_runSpeed(0, 0);
            HAL_Delay(20);

            return;
        } 

        //Debugging
        left_count = Encoder_getLeftCount();
        right_count = Encoder_getRightCount();
        left_delta_count = Encoder_getLeftDeltaCount();
        right_delta_count = Encoder_getRightDeltaCount();
        left_distance = Encoder_getLeftDistance();
        right_distance = Encoder_getRightDistance();

        //Distance PID calculations(update)
        basespeed = PIDController_Update(&controller.distance_pid, target_distance * direction, Encoder_getAverageDistance());

        //Straightline PID calculations
        float straightline_measurement = (float)(Encoder_getLeftCount() - Encoder_getRightCount());

        //Straightline PID update
        steering = PIDController_Update(&controller.straight_pid, 0.0f, straightline_measurement);

        // Calculate motor speeds
        float left_speed = basespeed + steering;
        float right_speed = basespeed - steering;

        // Clamp motor commands
        if (left_speed > 200) left_speed = 200;
        if (left_speed < -200) left_speed = -200;
        if (right_speed > 200) right_speed = 200;
        if (right_speed < -200) right_speed = -200;

        // Set motor speeds
        if (direction == -1.0f) {

            MotorBackward_runSpeed((uint8_t)(left_speed * direction), (uint8_t)(right_speed * direction));
        }
        else {

            MotorForward_runSpeed((uint8_t)left_speed, (uint8_t)right_speed);
        }

    }
}

//Helper to reset the state
static void resetStraightlineState(void) {

    //Reset encoders and PID controllers
    Encoders_Reset();
    resetPID();

    //Set running state
    controller.state = STRAIGHTLINE_RUNNING;
}

void runForwardDistance(float distance_cm) {

    //Check whether the controller is in idle state
    if (controller.state != STRAIGHTLINE_IDLE) {

        return;
    }
    
    resetStraightlineState();

    //Run untill the distance is reached
    while(1) {
        
        //Check whether the controller is finished
        if (controller.state == STRAIGHTLINE_COMPLETED) {

            controller.state = STRAIGHTLINE_IDLE;
            return;
        }

        updatePID(distance_cm, 1.0f);
    }
} 

void runBackwardDistance(float distance_cm) {

    //Check whether the controller is in idle state
    if (controller.state != STRAIGHTLINE_IDLE) {

        return;
    }
    
    resetStraightlineState();

    //Run untill the distance is reached
    while(1) {
        
        //Check whether the controller is finished
        if (controller.state == STRAIGHTLINE_COMPLETED) {

            controller.state = STRAIGHTLINE_IDLE;
            return;
        }

        updatePID(distance_cm, -1.0f);
    }
} 