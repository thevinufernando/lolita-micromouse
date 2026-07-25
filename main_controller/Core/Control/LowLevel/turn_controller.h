#ifndef TURN_CONTROLLER_H
#define TUEN_CONTROLLER_H

#include "PID.h"
#include "encoders.h"
#include "DRV8833.h"
#include <math.h>
#include "main.h"

#define PI 3.14159265358979323846f
#define ROBOT_WHEEL_BASE_CM 11.20f //in cm
#define TURN_TOLERANCE_CM 0.05f

typedef enum {

    //Turning modes
    TURN_IDLE = 0,
    TURN_RUNNING,
    TURN_COMPLETED

} TurnState_t;

//Function prototypes
void TurnController_Init(PIDController ctrl);
void turnLeftAngle(float angle_deg);
void turnRightAngle(float angle_deg);

#endif /* TURN_CONTROLLER_H */