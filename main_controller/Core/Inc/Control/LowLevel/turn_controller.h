#ifndef TURN_CONTROLLER_H
#define TURN_CONTROLLER_H

#include "PID.h"
#include "encoders.h"
#include "DRV8833.h"
#include "control_config.h"
#include <math.h>
#include "main.h"

#ifndef PI
#define PI 3.14159265358979323846f
#endif

#define ROBOT_WHEEL_BASE_CM 11.20f //in cm
#define TURN_TOLERANCE_CM TURN_TOLERANCE_ARC_CM

typedef enum {

    //Turning modes
    TURN_IDLE = 0,
    TURN_RUNNING,
    TURN_COMPLETED,
    TURN_TIMEOUT

} TurnState_t;

//Debugging
extern float turn_target_distance;
extern float turn_current_distance;
extern float turn_basespeed;

//Function prototypes

//Initialise using the gains from control_config.h
void TurnController_Init(void);

//Blocking turns. Return 1 on success, 0 if the safety timeout fired.
uint8_t turnLeftAngle(float angle_deg);
uint8_t turnRightAngle(float angle_deg);

#endif /* TURN_CONTROLLER_H */
