#include "navigator.h"

#include "maze_map.h"
#include "wall_sense.h"
#include "wall_follow.h"
#include "tof_sensors.h"
#include "straightline_controller.h"
#include "turn_controller.h"
#include "encoders.h"
#include "DRV8833.h"
#include "main.h"

volatile MazeTrace_t tm_maze_trace[MAZE_TRACE_CAPACITY];
volatile uint32_t    tm_maze_trace_count;
volatile uint8_t     tm_maze_complete;
volatile uint32_t    tm_maze_moves;
volatile uint8_t     tm_maze_abort_reason;


static void recordCell(float move_error_cm, uint8_t move_ok,
                       const WallReading_t *w, uint8_t action)
{
    if (tm_maze_trace_count >= MAZE_TRACE_CAPACITY) return;

    volatile MazeTrace_t *r = &tm_maze_trace[tm_maze_trace_count];

    r->timestamp_ms   = HAL_GetTick();
    r->x              = mouse_x;
    r->y              = mouse_y;
    r->dir            = (uint8_t)mouse_dir;
    r->front          = w->front;
    r->left           = w->left;
    r->right          = w->right;
    r->front_mm       = wall_front_mm;
    r->left_mm        = wall_left_mm;
    r->right_mm       = wall_right_mm;
    r->yaw_deg        = TurnController_GetYawDeg();
    r->heading_target = TurnController_GetHeadingTargetDeg();
    r->move_error_cm  = move_error_cm;
    r->move_ok        = move_ok;
    r->wall_side      = wf_side;
    r->action         = action;

    r->votes = (uint16_t)((wall_front_votes & 7U)
                        | ((wall_left_votes  & 7U) << 3)
                        | ((wall_right_votes & 7U) << 6));

    tm_maze_trace_count++;
}


/* Come to rest, read the walls, and write them into the map.
 *
 * Deliberately does NOT log. The record has to carry the action chosen from
 * this reading, and that is not known until after the decision -- so observing
 * and recording are two steps with the decision in between, and the log reads
 * as a decision trace rather than a list of places the robot happened to be. */
static void observe(WallReading_t *w, uint8_t write_map)
{
    Motor_Brake();
    HAL_Delay(NAV_SETTLE_MS);

    (void)WallSense_ReadCell(w);

    /* After a FAILED move the robot is somewhere between two cells and the
     * pose was deliberately not advanced, so these readings belong to no cell
     * the map can name. Log them, because they are the evidence of what went
     * wrong, but do not write them: a wall recorded into the wrong cell is
     * exactly the error the map can never undo. */
    if (write_map) MazeMap_UpdateWalls(w->front, w->left, w->right);
}


/* THE WHOLE NAVIGATOR. Three booleans in, one action out, no state.
 * See navigator.h for the rule and for why every action ends in a forward. */
static uint8_t decide(const WallReading_t *w)
{
#if NAV_HAND_RIGHT
    if (!w->right) return NAV_ACT_RIGHT;
    if (!w->front) return NAV_ACT_FORWARD;
    if (!w->left)  return NAV_ACT_LEFT;
#else
    if (!w->left)  return NAV_ACT_LEFT;
    if (!w->front) return NAV_ACT_FORWARD;
    if (!w->right) return NAV_ACT_RIGHT;
#endif
    return NAV_ACT_AROUND;   /* walled on three sides: dead end */
}


/* A pivot invalidates every ToF history: the sensor that was looking at a wall
 * 60 mm away is now looking somewhere else entirely, and the median and EMA
 * stages carry several samples of the old value across. The jump detector only
 * rescues LARGE steps, so a turn from one wall to another wall at a similar
 * distance would slip through as a slow ramp.
 *
 * The wall follower is reset for the same reason: its chosen side and its
 * accumulated drift both refer to a heading the robot no longer holds. */
static void afterTurn(void)
{
    ToF_ResetFilterAll();
    WallFollow_Reset();
}


/* Turn to face the chosen opening. Returns 0 if a turn timed out. */
static uint8_t faceOpening(uint8_t action)
{
    switch (action) {

        case NAV_ACT_RIGHT:
            if (!turnRightAngle(90.0f)) return 0U;
            MazeMap_TurnRight();
            afterTurn();
            return 1U;

        case NAV_ACT_LEFT:
            if (!turnLeftAngle(90.0f)) return 0U;
            MazeMap_TurnLeft();
            afterTurn();
            return 1U;

        case NAV_ACT_AROUND:
            /* Two 90 degree turns rather than one 180. Every gain, the profile
             * and the settle window were tuned and measured at 90, and the
             * continuous heading target means the second turn absorbs whatever
             * the first one left behind. A failure also says which half. */
            if (!turnRightAngle(90.0f)) return 0U;
            MazeMap_TurnRight();
            afterTurn();

            Motor_Brake();
            HAL_Delay(NAV_SETTLE_MS);

            if (!turnRightAngle(90.0f)) return 0U;
            MazeMap_TurnRight();
            afterTurn();
            return 1U;

        default:
            return 1U;          /* NAV_ACT_FORWARD: already facing right */
    }
}


void Navigator_Run(void)
{
    WallReading_t w = {0U, 0U, 0U};

    if (tm_maze_complete) return;

    MazeMap_Init();
    WallFollow_Reset();
    ToF_ResetFilterAll();
    TurnController_ResetYaw();   /* start of run: heading origin is here */

    tm_maze_moves = 0U;
    tm_maze_abort_reason = NAV_END_RUNNING;

    const int16_t start_x = mouse_x;
    const int16_t start_y = mouse_y;

    /* ABORT ON THE FIRST FAILED MOVE.
     *
     * The pose is only advanced when a move reports success, so that a failure
     * cannot write walls into the wrong cell. But that means after a failure
     * the map and the robot disagree, and every later survey records real
     * readings against a fictional pose. An early run did exactly that: one
     * timed-out move, and the remaining three cells were logged at (0,1)
     * facing north while the robot was physically somewhere else entirely.
     *
     * Stopping immediately makes the log say where it went wrong instead of
     * burying it under three cells of plausible-looking nonsense. That matters
     * more now than it did with a fixed route: the readings do not just fill
     * in the map any more, they choose the next turn. */

    /* Outcome of the move that arrived at the cell about to be surveyed. The
     * start cell was not reached by any move, hence the zero and the 1. */
    float   arrival_err_cm = 0.0f;
    uint8_t arrival_ok     = 1U;

    while (1) {

        observe(&w, 1U);

        uint8_t action = NAV_ACT_NONE;

        if (tm_maze_moves >= NAV_MAX_MOVES) {
            tm_maze_abort_reason = NAV_END_BUDGET;
        } else if (tm_maze_trace_count + 1U >= MAZE_TRACE_CAPACITY) {
            tm_maze_abort_reason = NAV_END_TRACE_FULL;
        } else if (NAV_STOP_AT_START
                   && tm_maze_moves >= NAV_STOP_AT_START
                   && mouse_x == start_x && mouse_y == start_y) {
            /* Back in the cell it began in. In a closed arena a wall follower
             * does this and would then repeat the identical lap forever.
             * Heading is NOT part of the test: the robot usually comes home
             * facing the opposite way, having turned around in a dead end, and
             * waiting for the exact start pose can mean waiting for a second
             * lap that adds nothing. */
            tm_maze_abort_reason = NAV_END_LOOPED;
        } else {
            action = decide(&w);
        }

        if (action == NAV_ACT_NONE) {
            recordCell(arrival_err_cm, arrival_ok, &w, NAV_ACT_STOP);
            break;
        }

        recordCell(arrival_err_cm, arrival_ok, &w, action);

        /* Act: turn to face the chosen opening, then drive into it. Pose
         * bookkeeping happens only after each move actually reports success,
         * so a timeout leaves the map consistent with reality. */
        uint8_t ok = faceOpening(action);

        if (ok) {
            Motor_Brake();
            HAL_Delay(NAV_SETTLE_MS);

            ok = runForwardFused(NAV_CELL_CM);
            arrival_err_cm = NAV_CELL_CM - Encoder_getAverageDistance();

            if (ok) MazeMap_Advance();
        } else {
            arrival_err_cm = 0.0f;   /* turn failed; no forward was attempted */
        }

        arrival_ok = ok;
        tm_maze_moves++;

        if (!ok) {
            observe(&w, 0U);
            recordCell(arrival_err_cm, 0U, &w, NAV_ACT_STOP);
            tm_maze_abort_reason = NAV_END_MOVE_FAILED;
            break;
        }
    }

    Motor_Brake();
    tm_maze_complete = 1U;
}
