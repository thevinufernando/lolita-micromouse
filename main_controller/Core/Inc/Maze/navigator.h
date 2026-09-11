#ifndef NAVIGATOR_H
#define NAVIGATOR_H

#include <stdint.h>

/*
 * ============================================================================
 *                    REACTIVE NAVIGATOR - WALL FOLLOWING
 * ============================================================================
 *
 * Drives the robot through an arena with no route coded anywhere. It stops at
 * each cell centre, reads the three ToF sensors, and picks its next action
 * from what it saw, so the same binary runs any arena.
 *
 * WHY THIS IS NOT IN test_harness.c. It used to be, and it did not belong
 * there: everything else in that file is bring-up scaffolding that exercises
 * one subsystem and is then never touched again. This is the robot's actual
 * behaviour. It is the piece the flood fill eventually replaces, and when that
 * happens only Navigator_Decide() changes while the map, the wall sensing and
 * the motion primitives underneath it stay exactly as they are.
 *
 * ---------------------------------------------------------------------------
 * THE RULE IS WALL FOLLOWING, NOT FLOOD FILL
 * ---------------------------------------------------------------------------
 * It keeps one hand on a wall and takes the first opening in a fixed priority
 * order. That is enough to traverse a simply-connected maze, but it knows
 * nothing about where the goal is and will loop forever in a maze with
 * islands. The map it records is a by-product; nothing reads it back to make
 * a decision.
 *
 * EVERY ACTION ENDS IN ONE CELL OF FORWARD MOTION. The turn only chooses which
 * way to leave; it is never the whole move. Getting this wrong is the classic
 * way to write a wall follower that does not work, and the host test caught
 * exactly that: with "turn right" as a standalone action the robot reached the
 * opening, turned into it, then saw an open right again and turned once more,
 * so it pivoted back down the corridor it came from and never entered the new
 * cell at all.
 *
 * Priority for the right-hand rule, checked in this order:
 *     right open -> turn right, advance    keeps the hand on the right wall
 *     front open -> advance
 *     left  open -> turn left,  advance
 *     else       -> dead end: turn around, advance
 *
 * Switch hands by setting NAV_HAND_RIGHT to 0. Neither hand is better in
 * general; they differ in which way they go round an obstacle.
 *
 * ---------------------------------------------------------------------------
 * DECISIONS COME FROM THE SENSORS, NEVER FROM THE MAP
 * ---------------------------------------------------------------------------
 * Those are not the same thing. The map has the outer boundary pre-set and
 * never clears a wall once set, so deciding from it would let one bad
 * reflection close a corridor for the rest of the run. The map records where
 * the robot has been; it is not an input to where it goes next.
 * ============================================================================
 */

#define NAV_CELL_CM 19.2f    /* centre-to-centre cell pitch */

/* Where the robot is placed in the map at the start of a run.
 *
 * PUT THE ARENA INSIDE THE MAP. (0,0) facing north is the competition
 * convention and is correct for a real 16x16 maze, but it pins the robot into
 * a corner with the west and south boundaries hard against it. A bench arena
 * that opens west of the start column does not fit there: a run found exactly
 * that opening, drove through it, and left the map behind.
 *
 * Starting mid-map costs nothing -- the coordinates are arbitrary labels, and
 * the recorded walls are the same shape wherever they land -- and it also
 * exercises the recorder honestly, because none of the boundary walls are
 * pre-set on top of readings the robot actually took. */
#define NAV_START_X   8
#define NAV_START_Y   0
#define NAV_START_DIR NORTH
#define NAV_HAND_RIGHT 1     /* 1 = right-hand rule, 0 = left-hand */

/* How much of a previous move's shortfall may be carried into the next one,
 * in cm. See tm_maze_residual_cm below for why anything is carried at all.
 *
 * A normal landing is inside STRAIGHT_TOLERANCE_CM, so a residual past twice
 * that did not come from an ordinary approach -- a slipped wheel, a nudged
 * robot, a move that barely finished. Carrying it would aim the next move at
 * something well past a cell boundary and turn one bad move into two. */
#define NAV_RESIDUAL_LIMIT_CM 3.0f

/* Pause before each wall reading, in ms. Lets the chassis settle so the
 * sensors are not measuring during a rock, and so the encoder reading at the
 * end of a move is unambiguous. */
#define NAV_SETTLE_MS 800U

/* Hard bound on the run, counted in CELLS ENTERED. A wall follower in an open
 * area circles forever, and a bench arena has no outer boundary to stop it, so
 * this is the only thing that guarantees the robot ends up somewhere you can
 * reach it. At roughly 3 s per cell, 24 is about 90 s of driving. */
#define NAV_MAX_MOVES 24U

/* Also stop on returning to the starting cell, after at least this many cells.
 * A wall follower in a closed arena comes home and then repeats the identical
 * lap forever, so this ends a bench run at the natural place instead of
 * waiting out the budget. Set to 0 to disable and use only the budget -- which
 * is what a real maze wants, since passing back through the start cell
 * mid-exploration is perfectly normal there. */
#define NAV_STOP_AT_START 3U

/* Why the run ended, in tm_maze_abort_reason. */
#define NAV_END_RUNNING     0U
#define NAV_END_BUDGET      1U   /* used up NAV_MAX_MOVES                */
#define NAV_END_MOVE_FAILED 2U   /* a forward or turn timed out          */
#define NAV_END_LOOPED      3U   /* back at the start cell, arena closed */
#define NAV_END_TRACE_FULL  4U   /* ran out of room to record            */
#define NAV_END_OFF_MAP     5U   /* the next cell is outside the maze     */

/* Actions the decision rule can produce. Recorded per cell so the log shows
 * WHY the robot did what it did, not just where it ended up. */
#define NAV_ACT_NONE     0U      /* no decision made (run ending)       */
#define NAV_ACT_FORWARD  1U      /* advance one cell                    */
#define NAV_ACT_LEFT     2U      /* turn left,  then advance one cell   */
#define NAV_ACT_RIGHT    3U      /* turn right, then advance one cell   */
#define NAV_ACT_AROUND   4U      /* turn 180,   then advance one cell   */
#define NAV_ACT_STOP     5U      /* logged on the final record          */

/* Per-cell record of what the run actually did. One entry per completed action
 * plus one for the start cell, so this must outlast the move budget. */
#define MAZE_TRACE_CAPACITY 32U

typedef struct {
  uint32_t timestamp_ms;
  int16_t x;             /* pose when the walls were read */
  int16_t y;
  uint8_t dir;           /* Direction enum                */
  uint8_t front;         /* walls seen, robot-relative    */
  uint8_t left;
  uint8_t right;
  uint16_t front_mm;     /* distances behind those calls  */
  uint16_t left_mm;
  uint16_t right_mm;
  /* Vote tallies out of WALL_SENSE_SAMPLES, 3 bits each:
   *     front | left << 3 | right << 6
   * Packed because it fits exactly in the padding the compiler was already
   * inserting here, so the record stays 36 bytes and the SWD reader's stride
   * does not move. A unanimous 5-0 is a confident call; a 3-2 on the FRONT
   * sensor is the one worth looking at, because that is the reading that
   * decides whether the robot drives into a wall. */
  uint16_t votes;
  float yaw_deg;         /* fused heading at the cell     */
  float heading_target;  /* what it should have been      */
  float move_error_cm;   /* distance error of the move in */
  uint8_t move_ok;       /* 1 = the move that got here completed */
  uint8_t wall_side;     /* WallFollowSide_t in use       */
  uint8_t action;        /* NAV_ACT_* chosen FROM this reading */
} MazeTrace_t;

_Static_assert(sizeof(MazeTrace_t) == 36,
               "MazeTrace_t stride changed: update the SWD telemetry reader");

extern volatile MazeTrace_t tm_maze_trace[MAZE_TRACE_CAPACITY];
extern volatile uint32_t    tm_maze_trace_count;
extern volatile uint8_t     tm_maze_complete;

/* Completed actions so far, and why the run stopped (NAV_END_*). */
extern volatile uint32_t tm_maze_moves;
extern volatile uint8_t  tm_maze_abort_reason;

/* How far the robot is behind the ideal cell grid along the CURRENT direction
 * of travel, in cm. Positive means short of where the map thinks it is.
 *
 * WHY THIS EXISTS. A move finishes as soon as it is inside
 * STRAIGHT_TOLERANCE_CM, and it approaches the target from below, so it always
 * stops a little early and never a little late. Measured over one corridor
 * run: seven moves, every one short, mean 0.39 cm. That is a bias, not noise,
 * and at 100 cells it is 39 cm -- more than two cells of disagreement between
 * the robot and its own map, which is when a solver starts recording walls in
 * the wrong place.
 *
 * So each move aims at one cell pitch PLUS what the last one left over, and
 * the leftover is tracked rather than discarded. This is the same trick the
 * heading already uses, and it is why heading error stayed inside +/-1.5 deg
 * across that run while distance quietly walked away.
 *
 * IT IS NOT A CURE FOR MISCALIBRATION. It closes the loop on the ENCODERS, so
 * it removes error the controller introduced. If the wheel diameter or gear
 * ratio is wrong, the encoders report a confident 19.2 cm while the robot
 * travels something else, and this will faithfully hold that wrong number.
 * Only an outside reference fixes that -- a front wall at a known distance. */
extern volatile float tm_maze_residual_cm;

/* BLOCKING. Runs the whole exploration and returns when it ends; check
 * tm_maze_abort_reason for why. Calling it again after a run has completed
 * does nothing, because a second lap would drive the robot back through an
 * arena it has already left. */
void Navigator_Run(void);

#endif /* NAVIGATOR_H */
