#include "navigator.h"

#include "cell_motion.h"

#include "maze_map.h"
#include "wall_sense.h"
#include "straightline_controller.h"
#include "DRV8833.h"
#include "main.h"









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




/* Turn to face the chosen opening. Returns 0 if a turn timed out. */
static uint8_t faceOpening(uint8_t action)
{
    switch (action) {

        case NAV_ACT_RIGHT:
            if (!CellMotion_TurnRight()) return 0U;
            MazeMap_TurnRight();
            return 1U;

        case NAV_ACT_LEFT:
            if (!CellMotion_TurnLeft()) return 0U;
            MazeMap_TurnLeft();
            return 1U;

        case NAV_ACT_AROUND:
            /* Two 90s rather than one 180. Every gain, the profile and the
             * settle window were tuned and measured at 90, the continuous
             * heading target means the second absorbs whatever the first left
             * behind, and a failure says which half failed.
             *
             * BACKING OUT INSTEAD WAS TRIED AND REMOVED. The idea was to buy a
             * cell of lateral correction before pivoting, but the side sensors
             * are at the very front of the chassis and cannot steer the robot
             * sideways in reverse -- see the REVERSE note in control_config.h.
             * Without that correction the reverse bought nothing: the pivot
             * happened at exactly the lateral offset it would have anyway, one
             * cell further back. Turning first is strictly better, because the
             * forward move that follows DOES correct laterally. */
            if (!CellMotion_TurnRight()) return 0U;
            MazeMap_TurnRight();

            if (!CellMotion_TurnRight()) return 0U;
            MazeMap_TurnRight();
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
    MazeMap_SetPose(NAV_START_X, NAV_START_Y, NAV_START_DIR);
    CellMotion_BeginRun();

    tm_maze_moves = 0U;
    tm_maze_abort_reason = NAV_END_RUNNING;
    tm_maze_residual_cm = 0.0f;

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

        CellMotion_Observe(&w, 1U);

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
            CellMotion_Record(arrival_err_cm, arrival_ok, &w, NAV_ACT_STOP);
            break;
        }

        CellMotion_Record(arrival_err_cm, arrival_ok, &w, action);

        /* Act. Pose bookkeeping happens only after each move actually reports
         * success, so a timeout leaves the map consistent with reality. */
        uint8_t ok = faceOpening(action);

        if (ok) {
            /* The settle after a pivot lives in CellMotion_TurnLeft/Right
             * now, so a straight-through cell no longer pays for one it does
             * not need -- it has been standing still through the wall vote
             * already, and the second pause cost 800 ms of every cell. */
            ok = CellMotion_Forward();


            /* Logged as the CUMULATIVE offset from the ideal grid, not this
             * move's own tracking error. It answers the question the map cares
             * about -- how far from the cell centre the robot actually is --
             * and with the carry in place the two are the same number anyway
             * at the end of every move. */
            arrival_err_cm = tm_maze_residual_cm;

            /* The robot has physically moved. If the map cannot follow it,
             * the run is over RIGHT HERE -- carrying on would log real wall
             * readings against a pose that is no longer where the robot is,
             * and the map never clears a wall once written. */
            if (ok && !MazeMap_Advance()) {
                CellMotion_Record(tm_maze_residual_cm, 1U, &w, NAV_ACT_STOP);
                tm_maze_abort_reason = NAV_END_OFF_MAP;
                break;
            }
        } else {
            arrival_err_cm = 0.0f;   /* turn failed; no forward was attempted */
        }

        arrival_ok = ok;
        tm_maze_moves++;

        if (!ok) {
            CellMotion_Observe(&w, 0U);
            CellMotion_Record(arrival_err_cm, 0U, &w, NAV_ACT_STOP);

            /* A wedge and a timeout are different failures and want different
             * answers -- one is mechanical, the other is usually a gain. The
             * log said MOVE FAILED for both, which sent a run's worth of
             * investigation at the steering when the robot had simply been
             * stuck against a wall. */
            tm_maze_abort_reason = sl_stall_abort ? NAV_END_STALLED
                                                  : NAV_END_MOVE_FAILED;
            break;
        }
    }

    /* ONE EXIT, and everything that has to be undone is undone here.
     *
     * Every break above lands on this line, and the only other way out of this
     * function returns before the sensors are started. That matters more than
     * usual because stopping is the awkward path on this part: the API call
     * only REQUESTS a stop, the sensor finishes whatever measurement is in
     * flight first, and reconfiguring during that window leaves the device in
     * an undefined state. A stop skipped on an abort path would leave the
     * sensors free-running into whatever ran next. */
    CellMotion_EndRun();

    tm_maze_complete = 1U;
}
