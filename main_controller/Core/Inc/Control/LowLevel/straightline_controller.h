#ifndef STRAIGHTLINE_CONTROLLER_H
#define STRAIGHTLINE_CONTROLLER_H

#include "encoders.h"
#include "DRV8833.h"
#include "PID.h"
#include "control_config.h"
#include <math.h>
#include "main.h"

#define DISTANCE_TOLERANCE_CM STRAIGHT_TOLERANCE_CM

typedef enum {

    //Running modes
    STRAIGHTLINE_IDLE = 0,
    STRAIGHTLINE_RUNNING,
    STRAIGHTLINE_COMPLETED,
    STRAIGHTLINE_TIMEOUT

} StraightlineState_t;

typedef struct {

    //PID initialisations
    PIDController straight_pid;
    PIDController distance_pid;

    StraightlineState_t state;

} Controller_t;

//Debugging
extern int32_t left_count;
extern int32_t right_count;
extern int16_t left_delta_count;
extern int16_t right_delta_count;
extern float left_distance;
extern float right_distance;
extern float basespeed;
extern float steering;

//Function Prototypes

//Initialise using the gains from control_config.h
void StraightlineController_Init(void);

//Blocking moves. Return 1 on success, 0 if the safety timeout fired.
uint8_t runForwardDistance(float distance_cm);
uint8_t runBackwardDistance(float distance_cm);

#endif /* STRAIGHTLINE_CONTROLLER_H */
