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


/* ---- One segment of a move, described rather than assumed ----
 *
 * runForwardFused() is this with every option at its default: from rest, to
 * rest, alignment on, no in-flight wall reading. Chained cell motion needs the
 * others, and passing six loose floats around was not going to age well.
 *
 * ENTRY AND EXIT SPEEDS ARE REFERENCE SPEEDS, not measurements. They describe
 * the profile the caller wants built, and the point of them is CONTINUITY: a
 * segment that begins where the last one's reference ended commands no step,
 * so the feedforward carries straight through a cell boundary instead of
 * dropping to zero and asking the robot to brake and restart. */
typedef struct {

  float distance_cm;      /* signed; maze moves are always forward       */

  /* Front-sensor reading the segment should END at, or 0 for odometry
   * alone. A chained segment stops short of the cell centre, so this is
   * the wall distance AT THAT POINT, not at the centre. */
  float front_target_mm;

  float entry_speed_cms;  /* 0 = starting from rest                      */
  float exit_speed_cms;   /* 0 = brake to rest and settle at the end     */

  /* 1 = a continuation of a move already in progress. The wall follower
   * keeps its side and its lean; only its travel baseline is restarted.
   * Resetting it mid-corridor throws away a good reference and steps the
   * tilt to zero at exactly the wrong moment. */
  uint8_t keep_wall_follow;

  /* 1 = do NOT zero the encoders; measure from wherever they are.
   *
   * A CALLER THAT TRACKS ITS OWN POSITION MUST SET THIS, and the default is
   * the other way round because everything that came before this struct read
   * Encoder_getAverageDistance() straight after the move and expected it to
   * be the distance travelled.
   *
   * Chained cell motion does track its own: it works out each segment's
   * length from an absolute odometer, which is how a segment absorbs the lag
   * the last one left and the open-loop travel during the solver's
   * think-time. Zeroing here would move the frame those numbers are measured
   * in out from under it, silently, between the caller computing a distance
   * and the segment starting. */
  uint8_t keep_odometry;

  /* IN-FLIGHT WALL READING. Sample the walls of the cell being ENTERED,
   * from `wall_window_cm` of travel onward, so the solver never has to
   * stop and vote. Negative disables it.
   *
   * `wall_centre_cm` is the travel at which the robot reaches that cell's
   * centre -- which a chained segment never does, and that is the point:
   * the front reading is compensated by the distance still to run, so a
   * wall is declared on what the sensor WILL read there. */
  float wall_window_cm;
  float wall_centre_cm;

} StraightMove_t;

uint8_t runForwardMove(const StraightMove_t *mv);


/* ---- What the last move saw while it was moving ----
 *
 * Votes counted the same way the stationary read counts them: strict majority
 * of the samples taken inside the window, with an invalid reading voting "no
 * wall" rather than abstaining. sl_flight_samples is the count they are out
 * of, and is the only thing that says whether the answer is worth having --
 * a window that produced two rotations has an opinion, not a measurement. */
extern volatile uint8_t  sl_flight_samples;
extern volatile uint8_t  sl_flight_front;
extern volatile uint8_t  sl_flight_left;
extern volatile uint8_t  sl_flight_right;
extern volatile uint8_t  sl_flight_front_votes;
extern volatile uint8_t  sl_flight_left_votes;
extern volatile uint8_t  sl_flight_right_votes;

/* Means over the valid samples. The front one is the PREDICTED reading at the
 * cell centre, so it compares directly against a stationary read. */
extern volatile uint16_t sl_flight_front_mm;
extern volatile uint16_t sl_flight_left_mm;
extern volatile uint16_t sl_flight_right_mm;


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
/* The lateral error the last move ENDED with, and whether it had a reference
 * to measure it against. Pair with sl_entry_err_mm: see the note in the .c. */
extern volatile float   sl_exit_err_mm;
extern volatile uint8_t sl_exit_valid;

extern volatile float   sl_align_delta_cm;
extern volatile uint8_t sl_align_applied;

/* WHY THE ALIGNMENT DID NOT FIRE, which until now had to be inferred from
 * where the robot stopped.
 *
 * It declines for six different reasons and they want six different responses:
 * a run with no wall ever in range is an arena that offers no chances, whereas
 * one that kept reading NO_ROOM is a window that has closed and a constant to
 * move. Guessing between them cost a run. Holds the LAST reason seen on the
 * move, so a move that fired reads FIRED and nothing else matters. */
#define SL_ALIGN_FIRED      0U  /* it fired                                  */
#define SL_ALIGN_NO_WALL    1U  /* nothing valid inside _RANGE_MM all move   */
#define SL_ALIGN_TOO_FAR    2U  /* in range but worse than the margin allows */
#define SL_ALIGN_NO_ROOM    3U  /* not enough left to decelerate into        */
#define SL_ALIGN_BIG_DELTA  4U  /* correction beyond _MAX_CM; not believed   */
#define SL_ALIGN_STALLED    5U  /* the move was wedged; not worth retargeting*/
#define SL_ALIGN_INFEASIBLE 6U  /* the rebuilt profile would not fit         */

extern volatile uint8_t sl_align_reason;

/* Breakaway pulses fired during the last move. Should normally be 0. A move
 * that needed one still succeeded; a move that needed STRAIGHT_BREAKAWAY_MAX
 * and then timed out was jammed, not merely stuck, and the difference is worth
 * knowing before reaching for the tuning. */
extern volatile uint32_t sl_breakaway_count;

/* The lateral error the last move STARTED with, and whether it had a reference
 * at all to measure one from.
 *
 * Answers a question no log could: does a PIVOT throw the robot sideways? One
 * run came out of a dead end 46 mm further from the same wall than it went in,
 * across one 180 and one cell of travel, with no way to tell which of the two
 * did it. Recorded per cell by the navigator, because the per-cycle trace only
 * survives the last move. */
extern volatile float   sl_entry_err_mm;
extern volatile uint8_t sl_entry_valid;

/* Set when a move was abandoned because the robot stopped moving while the
 * command was still above the stiction floor -- a wedge, not a steering fault.
 * Cleared at the start of every move. */
extern volatile uint8_t sl_stall_abort;

//Blocking moves. Return 1 on success, 0 if the safety timeout fired.
uint8_t runForwardDistance(float distance_cm);
uint8_t runBackwardDistance(float distance_cm);

#endif /* STRAIGHTLINE_CONTROLLER_H */
