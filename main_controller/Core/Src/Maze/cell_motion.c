#include "cell_motion.h"

#include "maze_map.h"
#include "navigator.h"
#include "wall_follow.h"
#include "tof_sensors.h"
#include "straightline_controller.h"
#include "turn_controller.h"
#include "encoders.h"
#include "DRV8833.h"
#include "control_config.h"
#include "main.h"

volatile float   tm_maze_residual_cm;
volatile uint8_t tm_maze_tof_start_fail;
volatile uint8_t tm_maze_tof_stop_fail;

/* THE TRACE LIVES HERE, not with either driver, because it is a record of what
 * the ROBOT did rather than of what any particular rule decided. Every
 * diagnosis this firmware has survived came out of it, and having two drivers
 * each keep their own would mean the flood fill starting blind. Declared in
 * navigator.h, which is also where MazeTrace_t and the NAV_ACT_* vocabulary
 * live. */
volatile MazeTrace_t tm_maze_trace[MAZE_TRACE_CAPACITY];
volatile uint32_t    tm_maze_trace_count;
volatile uint8_t     tm_maze_complete;
volatile uint32_t    tm_maze_moves;
volatile uint8_t     tm_maze_abort_reason;


void CellMotion_Record(float move_error_cm, uint8_t move_ok,
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
    r->drift_deg      = WallFollow_GetDriftDeg();
    r->entry_err_mm   = sl_entry_err_mm;
    r->entry_valid    = sl_entry_valid;
    r->align_delta_cm = sl_align_applied ? sl_align_delta_cm : 0.0f;

    r->votes = (uint16_t)((wall_front_votes & 7U)
                        | ((wall_left_votes  & 7U) << 3)
                        | ((wall_right_votes & 7U) << 6));

    tm_maze_trace_count++;
}


/* Hand the wall follower what the map knows about the two cells this move
 * touches, so it can tell a reference that has ended from a robot that has
 * drifted.
 *
 * Called immediately before each forward move, from the pose the move starts
 * at -- which is why it reads mouse_x/y/dir directly rather than taking them:
 * any disagreement between the pose used here and the pose the move actually
 * begins from would silently veto the wrong cell's walls.
 *
 * An unknown next cell leaves next_known at 0, which contributes no opinion
 * and leaves the follower exactly as it behaves without any of this. That is
 * the common case on a first pass and it is meant to be. */
static void setCellContext(void)
{
    WallFollowCells_t c;
    uint8_t f;

    c.this_known = MazeMap_IsKnown(mouse_x, mouse_y);
    MazeMap_CellWalls(mouse_x, mouse_y, mouse_dir, &f, &c.left_this, &c.right_this);

    int16_t nx = 0, ny = 0;

    c.left_next  = 0U;
    c.right_next = 0U;
    c.next_known = 0U;

    if (MazeMap_NextCell(mouse_x, mouse_y, mouse_dir, &nx, &ny)
        && MazeMap_IsKnown(nx, ny)) {

        /* Same heading: the robot does not turn during a forward move, so the
         * next cell's left and right are the same sides its own are. */
        MazeMap_CellWalls(nx, ny, mouse_dir, &f, &c.left_next, &c.right_next);
        c.next_known = 1U;
    }

    /* The sensors lead the axle, so they cross into the next cell well before
     * the robot does. Derived rather than configured: it is a consequence of
     * the cell pitch and where the sensors are bolted, and two constants that
     * can disagree about the same fact is one too many. */
    c.cross_cm = NAV_CELL_CM * 0.5f - TOF_SIDE_AHEAD_CM;

    WallFollow_SetCells(&c);
}


/* Come to rest, read the walls, and write them into the map.
 *
 * Deliberately does NOT log. The record has to carry the action chosen from
 * this reading, and that is not known until after the decision -- so observing
 * and recording are two steps with the decision in between, and the log reads
 * as a decision trace rather than a list of places the robot happened to be. */
void CellMotion_Observe(WallReading_t *w, uint8_t write_map)
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

    /* The carried residual is an error ALONG the direction of travel. A pivot
     * makes that axis the new lateral axis, where this number means nothing --
     * carrying it would aim the next move at a target derived from a distance
     * measured sideways. Zero is the honest value: after a turn the robot does
     * not know its longitudinal offset, and the wall follower and (later) the
     * front-wall anchor are what recover it. */
    tm_maze_residual_cm = 0.0f;
}


void CellMotion_BeginRun(void)
{
    WallFollow_ResetBias();   /* start of a RUN: forget the learned bias too */

    /* FREE-RUN THE SENSORS FOR THE WHOLE RUN. See MAZE_TOF_CONTINUOUS.
     *
     * Started before the filter reset, not after, so the reset sees the
     * sensors already in continuous mode and arms its discard accordingly.
     * The two are describing the same state and should not disagree. */
#if MAZE_TOF_CONTINUOUS
    tm_maze_tof_start_fail = (ToF_StartContinuousAll() != TOF_OK) ? 1U : 0U;
#endif

    ToF_ResetFilterAll();
    TurnController_ResetYaw();   /* start of run: heading origin is here */

    tm_maze_residual_cm = 0.0f;
}


void CellMotion_EndRun(void)
{
    Motor_Brake();

#if MAZE_TOF_CONTINUOUS
    tm_maze_tof_stop_fail = (ToF_StopContinuousAll() != TOF_OK) ? 1U : 0U;
#endif
}


uint8_t CellMotion_TurnLeft(void)
{
    if (!turnLeftAngle(90.0f)) return 0U;

    Motor_Brake();
    HAL_Delay(NAV_SETTLE_MS);
    afterTurn();

    return 1U;
}


uint8_t CellMotion_TurnRight(void)
{
    if (!turnRightAngle(90.0f)) return 0U;

    Motor_Brake();
    HAL_Delay(NAV_SETTLE_MS);
    afterTurn();

    return 1U;
}


uint8_t CellMotion_Forward(void)
{
    /* Aim at one cell pitch PLUS whatever the last move left short, so the
     * shortfall is corrected instead of accumulating. */
    const float target_cm = NAV_CELL_CM + tm_maze_residual_cm;

    setCellContext();

    const uint8_t ok = runForwardFused(target_cm);

    /* !! THE TWO CORRECTIONS MUST NOT BOTH FIRE !!
     *
     * The carry exists because the encoders quietly lose ground every move.
     * The front-wall alignment exists because the encoders cannot know where
     * the cell boundaries are. When the alignment fires it has ALREADY put the
     * robot where it belongs, measured against a wall -- so the gap the
     * encoders report is not an error left over, it IS the correction, and
     * carrying it into the next move applies the same correction twice.
     *
     * Observed exactly that: two cells finished 88 and 89 mm from a wall
     * against an 87 mm target, so within a millimetre of perfect, while
     * reporting 2.18 and 3.00 cm of shortfall. Both were carried forward, the
     * next move overran by that much, and after the following turn it came
     * back as lateral error. */
    float left_over = sl_align_applied
                    ? 0.0f
                    : (target_cm - Encoder_getAverageDistance());

    if (left_over >  NAV_RESIDUAL_LIMIT_CM) left_over =  NAV_RESIDUAL_LIMIT_CM;
    if (left_over < -NAV_RESIDUAL_LIMIT_CM) left_over = -NAV_RESIDUAL_LIMIT_CM;

    tm_maze_residual_cm = left_over;

    return ok;
}
