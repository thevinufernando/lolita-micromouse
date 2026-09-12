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
/* 200 entries at CONTROL_SAMPLE_TIME_S covers 2 s, which is a whole cell at
 * STRAIGHT_PROFILE_MAX_CMS. At 120 the trace ran out four fifths of the way
 * through every move, and the end of the move is where it goes wrong. */
#define SL_TRACE_CAPACITY 200U

typedef struct {
  float t_s;
  float ref_cm;   /* profile reference   */
  float act_cm;   /* encoder average     */
  float base;     /* forward command     */
  float steer;    /* differential        */
  float yaw_err;  /* heading error, deg  */
  /* THE CONTINUOUS HEADING AND ITS THREE PARTS.
   *
   * yaw_err alone says the inner loop is happy; it cannot say whether the
   * heading it is happy about is the right one. These four together decompose
   * the demand completely:
   *
   *     yaw_deg  = where the robot believes it is pointing, unwrapped
   *     tilt_deg = what the lateral loop is asking for, position only
   *     drift_deg= what the integral has learned about the reference
   *     err_mm   = the lateral error driving both
   *
   * A robot that is visibly yawed while yaw_err reads zero is the case this
   * exists to catch: the estimate is wrong, not the loop, and drift_deg is
   * the term that has to move. */
  float yaw_deg;
  float tilt_deg;
  float drift_deg;
  float err_mm;
} StraightTrace_t;

_Static_assert(sizeof(StraightTrace_t) == 40,
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

/* Front-wall alignment result for the last move. sl_align_applied says whether
 * the retarget fired at all; sl_align_delta_cm is how far it moved the
 * endpoint. A delta that is consistently one sign means WALL_FRONT_ALIGN_MM
 * does not match where the robot actually stops -- re-measure it rather than
 * letting the alignment fight the profile every move. */
extern volatile float   sl_align_delta_cm;
extern volatile uint8_t sl_align_applied;

/* Breakaway pulses fired during the last move. Should normally be 0. A move
 * that needed one still succeeded; a move that needed STRAIGHT_BREAKAWAY_MAX
 * and then timed out was jammed, not merely stuck, and the difference is worth
 * knowing before reaching for the tuning. */
extern volatile uint32_t sl_breakaway_count;

//Blocking moves. Return 1 on success, 0 if the safety timeout fired.
uint8_t runForwardDistance(float distance_cm);
uint8_t runBackwardDistance(float distance_cm);

#endif /* STRAIGHTLINE_CONTROLLER_H */
