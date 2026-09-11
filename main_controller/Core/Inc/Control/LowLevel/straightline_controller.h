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

/* ---- Fused move: gyro heading + single-wall ToF centring ----
 *
 * Distance still comes from the encoders. Heading comes from the FUSED yaw
 * estimate rather than the encoder tick difference, which is the only way to
 * tell a real rotation from a slipping wheel. Lateral position comes from
 * whichever side wall is in range, cascaded into the heading setpoint rather
 * than summed alongside it -- see wall_follow.h for why that matters.
 *
 * Holds TurnController_GetHeadingTargetDeg(), so a turn that finished a
 * couple of degrees short is inherited as an ordinary setpoint error and
 * corrected here. That is the whole reason yaw is no longer reset per move. */
uint8_t runForwardFused(float distance_cm);

/* Per-cycle trace of the last fused move, same idea as the turn trace.
 *
 * Guessing at the turn twice made things worse in two different directions;
 * one trace then found the real cause immediately. The straight move has had
 * no instrumentation at all, and it is now failing identically across two
 * different binaries, so the answer is data rather than another guess. */
#define SL_TRACE_CAPACITY 120U

typedef struct {
  float t_s;
  float ref_cm;   /* profile reference   */
  float act_cm;   /* encoder average     */
  float base;     /* forward command     */
  float steer;    /* differential        */
  float yaw_err;  /* heading error, deg  */
} StraightTrace_t;

_Static_assert(sizeof(StraightTrace_t) == 24,
               "StraightTrace_t stride changed: update the SWD telemetry reader");

extern volatile StraightTrace_t tm_sl_trace[SL_TRACE_CAPACITY];
extern volatile uint32_t        tm_sl_trace_count;

/* Live state of the fused move, for telemetry. */
extern volatile float sl_yaw_target_deg;   /* heading target + wall tilt  */
extern volatile float sl_yaw_error_deg;
extern volatile float sl_steering;
extern volatile float sl_basespeed;
extern volatile uint32_t sl_sat_cycles;    /* cycles where steering clipped base */
extern volatile float sl_ref_cm;           /* profile reference position  */

//Blocking moves. Return 1 on success, 0 if the safety timeout fired.
uint8_t runForwardDistance(float distance_cm);
uint8_t runBackwardDistance(float distance_cm);

#endif /* STRAIGHTLINE_CONTROLLER_H */
