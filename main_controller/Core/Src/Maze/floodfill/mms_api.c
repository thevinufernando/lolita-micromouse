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
 * WHERE THAT ONE READ COMES FROM IS THE MOTION LAYER'S BUSINESS. With chained
 * cells it was taken on the way in, at speed, and costs nothing at all; only
 * when too few samples landed does the robot stop and vote. The distinction is
 * invisible from here and from the algorithm, which is the point.
 *
 * ---------------------------------------------------------------------------
 * A TURN DOES NOT NEED A NEW READING
 * ---------------------------------------------------------------------------
 * The snapshot is robot-relative, so a pivot invalidates it -- but not the
 * underlying fact. The algorithm has already written this cell's walls into
 * v_walls/h_walls through its own updateWalls(), which runs before the turn,
 * so the rotated view can be read straight back out of the map.
 *
 * That is worth having: re-observing after every pivot cost a settle and a
 * five-vote sweep, better than a second per turn, to rediscover something
 * already written down. The fourth side -- the one behind -- is the cell the
 * robot drove in from, which is open, and the map says so.
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
#include "test_harness.h"   /* LED_Blink -- the robot's only output device */

/* This cell's walls, read once on arrival. */
static WallReading_t s_cell;
static uint8_t       s_cell_valid;

/* A pivot that timed out, waiting for somewhere to be reported. */
static uint8_t s_turn_failed;

/* Has the robot actually moved yet? Distinguishes the algorithm's startup
 * API_setColor(0,0,'G') -- emitted before the first move -- from the 'G' that
 * means "returned to the start", which is a genuine milestone. */
static uint8_t s_run_started;

/* What the last decision turned out to be, so the trace records a decision and
 * not just a list of places the robot has been. Turns arrive before the move
 * they precede, which is the order Main.c issues them in. */
static uint8_t s_pending_action = NAV_ACT_FORWARD;


/* Read this cell if it has not been read since the last move. */
static void ensureCell(void)
{
    if (s_cell_valid) return;

    /* Free if the way in gathered enough; a stop and a five-vote sweep if it
     * did not. Never a guess. */
    if (!CellMotion_FlightWalls(&s_cell)) {
        CellMotion_Observe(&s_cell, 0U);
    }

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

    /* NOT cleared here. MMS_ApiReset() runs between PHASES as well as at the
     * start of a run, and the robot has certainly moved by the time the second
     * phase begins -- clearing it would make the return-to-start 'G' silent
     * every time. It is a run-lifetime flag, and a run begins at power-on. */
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

    s_run_started    = 1U;    /* any later 'G' means "came back", not "start" */
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


/* Re-express the cached reading for the heading the robot has just turned to.
 *
 * The pose belongs to the algorithm and is not updated until after this call
 * returns, so the new heading is computed here rather than read. This still
 * touches nothing it does not own: it reads mouse_x/y/dir and writes only the
 * local snapshot and the published distances. */
static void rotateCell(uint8_t turned_left)
{
    if (!s_cell_valid) return;

    const Direction dir = (Direction)((mouse_dir + (turned_left ? 3 : 1)) % 4);

    uint8_t f = 0U, l = 0U, r = 0U;

    MazeMap_CellWalls((int16_t)mouse_x, (int16_t)mouse_y, dir, &f, &l, &r);

    s_cell.front = f;
    s_cell.left  = l;
    s_cell.right = r;

    /* THE DISTANCES HAVE TO TURN WITH THE FLAGS, or the per-cell trace records
     * a front wall beside a left-hand distance and the next person to read a
     * log spends an hour on it.
     *
     * The side the robot has turned AWAY from becomes the side behind it,
     * which no sensor has measured from here -- so it is reported as what it
     * is, unmeasured, rather than filled in with a number that means something
     * else. The flag for that side still comes from the map above, which does
     * know. */
    const uint16_t front = wall_front_mm;
    const uint16_t left  = wall_left_mm;
    const uint16_t right = wall_right_mm;

    /* The vote tallies turn with them. Leaving those behind put a 5-of-5
     * "wall ahead" in the trace beside a front flag of 0, which is the sort of
     * contradiction that costs an hour before anyone suspects the log. */
    const uint8_t vf = wall_front_votes;
    const uint8_t vl = wall_left_votes;
    const uint8_t vr = wall_right_votes;

    if (turned_left) {
        wall_front_mm = left;    wall_front_votes = vl;
        wall_right_mm = front;   wall_right_votes = vf;
        wall_left_mm  = TOF_DISTANCE_INVALID;
        wall_left_votes = 0U;
    }
    else {
        wall_front_mm = right;   wall_front_votes = vr;
        wall_left_mm  = front;   wall_left_votes  = vf;
        wall_right_mm = TOF_DISTANCE_INVALID;
        wall_right_votes = 0U;
    }
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

    rotateCell(1U);
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

    rotateCell(0U);
}


/* ---- The simulator's drawing surface, which the robot does not have ----
 *
 * Kept as no-ops rather than stripped from the algorithm, because every call
 * to them is a line of the original that would otherwise have to be edited
 * out. An empty function costs nothing and keeps the port honest. */
void API_setWall(int x, int y, char direction)   { (void)x; (void)y; (void)direction; }
void API_clearWall(int x, int y, char direction) { (void)x; (void)y; (void)direction; }

/* ---- Except this one, which is now the MILESTONE INDICATOR ----
 *
 * The robot has no screen, but it has an LED, and the algorithm already marks
 * its milestones by colouring a cell. Those calls are in the ORIGINAL, so
 * hooking them here costs the port nothing.
 *
 * !! THIS IS WHY THE BLINK LIVES HERE AND NOT IN floodfill_run.c !!
 * `tests/floodfill_diff.sh` builds the ported algorithm and the upstream
 * MicroMouseAlgorithm copy against the same simulated maze and compares their
 * transcripts action for action. A call added inside floodfill_run.c would
 * make the two differ and destroy that guarantee. This file is the porting
 * boundary and is EXPECTED to differ -- it is where "what the algorithm means"
 * becomes "what this robot does". (The diff test links floodfill_sim_api.c,
 * not this file, so it never sees any of it.)
 *
 * ---------------------------------------------------------------------------
 * WHICH COLOURS ARE MILESTONES, AND WHICH ARE NOT
 * ---------------------------------------------------------------------------
 * The algorithm calls this from SIX places, and most of them are not events:
 *
 *   'R' at goal      -- all four centre cells confirmed visited   BLINK
 *   'R' speed done   -- speed run reached the goal                BLINK
 *   'G' at start     -- ONCE before the run begins, not a result  ignored
 *   'G' back home    -- returned to start, about to speed run     blink
 *   'B' / 'Y'        -- EVERY ORDINARY CELL of every phase        ignored
 *
 * That last line is the one that matters: 'B'/'Y' fires on every cell the
 * robot enters, so blinking on anything other than an explicit milestone
 * colour would stop the robot for a second in each of ~250 cells. Only 'R'
 * and 'G' are handled, and everything else falls through silently.
 *
 * The startup 'G' is suppressed by s_run_started, which the first forward
 * move sets -- at that point 'G' can only mean "came back". Without it the
 * robot blinks the return pattern before it has moved at all.
 *
 * !! IT MUST STOP THE ROBOT FIRST, AND THE FIRST VERSION DID NOT !!
 *
 * The original note here claimed the robot was "standing at a cell centre,
 * motors already braked". That is true only WITHOUT cell chaining. With
 * MAZE_CONTINUOUS_CELLS a forward move ends early and returns with the robot
 * STILL ROLLING AT CRUISE, precisely so the next move can continue without
 * stopping -- and the milestone fires on that path too.
 *
 * Measured: tm_chain_gap_ms_max read 4226 ms against a 4200 ms goal pattern.
 * The blink WAS the gap. The motors held their last command open-loop for the
 * whole 4.2 s, which at CELL_CHAIN_SPEED_CMS is about 59 cm of travel with no
 * control loop running -- three cells, blind, from a robot that thought it was
 * celebrating.
 *
 * So the robot is brought to rest first. CellMotion_StopAtCell() does nothing
 * when it is already stopped, so the unchained path is unaffected; on the
 * chained path it drives the remaining decision offset and brakes, which is
 * exactly what a turn would have done. After that the blocking blink is
 * genuinely safe, and sits in the same gap NAV_SETTLE_MS already occupies. */
void API_setColor(int x, int y, char color)
{
    (void)x;
    (void)y;

    /* Nothing below may run while the robot is moving. */
    if (color == 'R' || (color == 'G' && s_run_started)) {
        (void)CellMotion_StopAtCell();
    }

    if (color == 'R') {
        /* GOAL. Long-short-short, repeated: a RHYTHM, which is the one thing
         * no other indicator on this robot uses. Boot is 3 slow or 6 fast, a
         * halted test is a steady 1 Hz, a running ToF test is a fast toggle --
         * all of them are RATES, and all look alike from across a maze. */
        for (uint8_t i = 0; i < MAZE_GOAL_BLINK_REPEATS; i++) {
            LED_Blink(1U, MAZE_GOAL_BLINK_LONG_MS, MAZE_GOAL_BLINK_GAP_MS);
            LED_Blink(2U, MAZE_GOAL_BLINK_SHORT_MS, MAZE_GOAL_BLINK_GAP_MS);
            HAL_Delay(MAZE_GOAL_BLINK_PAUSE_MS);
        }
    }
    else if (color == 'G' && s_run_started) {
        /* Back at the start, about to begin the speed run. Even blinks --
         * clearly not the goal rhythm, because two milestones that look alike
         * are two milestones you cannot tell apart. */
        LED_Blink(MAZE_START_BLINK_COUNT, MAZE_START_BLINK_MS,
                  MAZE_START_BLINK_MS);
    }

    /* 'B', 'Y', and the pre-run 'G' deliberately do nothing. */
}

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
