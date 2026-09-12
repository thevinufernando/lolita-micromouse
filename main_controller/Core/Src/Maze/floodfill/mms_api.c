/*
 * ============================================================================
 *        API.h, IMPLEMENTED AGAINST THE ROBOT INSTEAD OF THE SIMULATOR
 * ============================================================================
 *
 * The flood fill talks to the world through exactly these functions. On the
 * desktop they were a pipe to mms; here they drive motors and read ToF
 * sensors. Nothing above this file knows the difference, which is the whole
 * reason the algorithm came across unchanged.
 *
 * ---------------------------------------------------------------------------
 * THE ONE ADAPTATION THAT MATTERS: WALL READINGS ARE CACHED PER CELL
 * ---------------------------------------------------------------------------
 * API_wallFront/Left/Right are called several times per cell -- once inside
 * updateWalls() and again inside getBestDirection(). On the simulator that is
 * a free array lookup. Here each one would be a five-vote sensor sweep costing
 * around 200 ms, so a cell would spend the better part of a second re-reading
 * walls that cannot have moved.
 *
 * So the cell is read ONCE, when the robot arrives, and every query is served
 * from that snapshot until it moves again. This changes nothing about the
 * algorithm's behaviour: it never moves between those calls, so a fresh read
 * would return the same thing at several hundred milliseconds a time. The
 * host test asserts the two produce identical decisions.
 *
 * ---------------------------------------------------------------------------
 * A FAILED TURN IS REPORTED ONE CALL LATE
 * ---------------------------------------------------------------------------
 * API_turnLeft/Right return void, so a pivot that times out has nowhere to
 * report it. Rather than change the algorithm's interface, the failure is
 * latched here and returned from the next API_moveForward() -- which Main.c
 * already treats as a crash and ends the run on. The trace will show the
 * failure attached to the move after the turn rather than to the turn itself.
 *
 * ---------------------------------------------------------------------------
 * THE POSE IS NOT OURS
 * ---------------------------------------------------------------------------
 * Main.c updates mouse_x, mouse_y and mouse_dir itself after every completed
 * move, exactly as it did on the simulator. This file must never advance or
 * rotate them. Two writers would disagree the first time a move failed, and
 * every wall recorded afterwards would land in a cell the robot never entered.
 * ============================================================================
 */
#include "API.h"
#include "maze.h"
#include "mms_api.h"

#include "cell_motion.h"
#include "maze_map.h"
#include "navigator.h"
#include "wall_sense.h"
#include "straightline_controller.h"

/* This cell's walls, read once on arrival. */
static WallReading_t s_cell;
static uint8_t       s_cell_valid;

/* A pivot that timed out, waiting for somewhere to be reported. */
static uint8_t s_turn_failed;

/* What the last decision turned out to be, so the trace records a decision and
 * not just a list of places the robot has been. Turns arrive before the move
 * they precede, which is the order Main.c issues them in. */
static uint8_t s_pending_action = NAV_ACT_FORWARD;


/* Read this cell if it has not been read since the last move. */
static void ensureCell(void)
{
    if (s_cell_valid) return;

    CellMotion_Observe(&s_cell, 0U);

    /* The algorithm writes the walls itself, through its own updateWalls(),
     * into the same arrays. But it knows nothing about the visited bitmap the
     * map view keeps, so without this MazeMap_IsKnown() would answer "no" for
     * every cell of a flood-fill run and the wall follower's cell veto would
     * never fire. */
    MazeMap_MarkKnown((int16_t)mouse_x, (int16_t)mouse_y);

    s_cell_valid = 1U;
}


/* Called by the runner between phases and at the start, so the first cell of a
 * run is read fresh rather than inherited from whatever came before. */
void MMS_ApiReset(void)
{
    s_cell_valid     = 0U;
    s_turn_failed    = 0U;
    s_pending_action = NAV_ACT_FORWARD;
}


int API_mazeWidth(void)  { return MAZE_SIZE; }
int API_mazeHeight(void) { return MAZE_SIZE; }

int API_wallFront(void) { ensureCell(); return s_cell.front ? 1 : 0; }
int API_wallLeft(void)  { ensureCell(); return s_cell.left  ? 1 : 0; }
int API_wallRight(void) { ensureCell(); return s_cell.right ? 1 : 0; }


int API_moveForward(void)
{
    /* A turn failed earlier and this is the first chance to say so. Main.c
     * reads a zero here as a crash and ends the run, which is the right
     * outcome: after a failed pivot the robot's heading is unknown and every
     * wall it records from here would be recorded against a lie. */
    if (s_turn_failed) {
        CellMotion_Record(0.0f, 0U, &s_cell, NAV_ACT_STOP);
        tm_maze_abort_reason = NAV_END_MOVE_FAILED;
        return 0;
    }

    ensureCell();

    /* Logged BEFORE the move, with the action chosen from this cell's reading,
     * so the trace reads as a decision followed by its outcome. The distance
     * error belongs to the move that ARRIVED here, which is the residual the
     * previous forward left behind. */
    CellMotion_Record(tm_maze_residual_cm, 1U, &s_cell, s_pending_action);

    const uint8_t ok = CellMotion_Forward();

    s_cell_valid     = 0U;    /* the robot has moved; the snapshot is stale */
    s_pending_action = NAV_ACT_FORWARD;

    tm_maze_moves++;

    if (!ok) {
        /* Somewhere between two cells, so the readings taken now belong to no
         * cell the map can name. Worth logging as the evidence, not worth
         * storing. */
        WallReading_t w = {0U, 0U, 0U};

        CellMotion_Observe(&w, 0U);
        CellMotion_Record(0.0f, 0U, &w, NAV_ACT_STOP);

        tm_maze_abort_reason = sl_stall_abort ? NAV_END_STALLED
                                              : NAV_END_MOVE_FAILED;
        return 0;
    }

    return 1;
}


void API_turnLeft(void)
{
    if (s_turn_failed) return;

    if (!CellMotion_TurnLeft()) {
        s_turn_failed = 1U;
        return;
    }

    /* Two lefts in a row is the algorithm's way of spelling a 180, and that is
     * worth seeing in the trace as one about-turn rather than two pivots. */
    s_pending_action = (s_pending_action == NAV_ACT_LEFT) ? NAV_ACT_AROUND
                                                          : NAV_ACT_LEFT;
    s_cell_valid = 0U;
}


void API_turnRight(void)
{
    if (s_turn_failed) return;

    if (!CellMotion_TurnRight()) {
        s_turn_failed = 1U;
        return;
    }

    s_pending_action = (s_pending_action == NAV_ACT_RIGHT) ? NAV_ACT_AROUND
                                                           : NAV_ACT_RIGHT;
    s_cell_valid = 0U;
}


/* ---- The simulator's drawing surface, which the robot does not have ----
 *
 * Kept as no-ops rather than stripped from the algorithm, because every call
 * to them is a line of the original that would otherwise have to be edited
 * out. An empty function costs nothing and keeps the port honest. */
void API_setWall(int x, int y, char direction)   { (void)x; (void)y; (void)direction; }
void API_clearWall(int x, int y, char direction) { (void)x; (void)y; (void)direction; }
void API_setColor(int x, int y, char color)      { (void)x; (void)y; (void)color; }
void API_clearColor(int x, int y)                { (void)x; (void)y; }
void API_clearAllColor(void)                     { }
void API_setText(int x, int y, char *str)        { (void)x; (void)y; (void)str; }
void API_clearText(int x, int y)                 { (void)x; (void)y; }
void API_clearAllText(void)                      { }

/* The simulator could restart a run under the algorithm's feet. Nothing can do
 * that here: a reset is a power cycle, and the firmware starts from the top. */
int  API_wasReset(void) { return 0; }
void API_ackReset(void) { }

/* There is no stderr on the target. The messages are still worth their place
 * in the algorithm -- they say what it believes is happening -- and the trace
 * records the same transitions in a form that survives a power cycle. */
void debug_log(char *text) { (void)text; }
