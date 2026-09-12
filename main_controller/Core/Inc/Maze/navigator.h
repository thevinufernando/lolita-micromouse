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
 *     else       -> dead end: REVERSE one cell, then turn around
 *
 * That last one is the exception to the forward rule above, and it is the same
 * two moves in the opposite order: it ends in the same cell facing the same
 * way, for the same cost. Turning first means pivoting the instant the robot
 * arrives, carrying whatever error it collected driving into the wall.
 * Reversing first means pivoting after a full cell of lateral correction. The
 * chassis is large for the cell -- every successful pivot in one measured run
 * was within 8.5 mm of centre and the one that jammed was 24 mm out -- so that
 * margin decides whether the turn happens at all.
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
 * (0,0) FACING NORTH: the competition convention, and correct when the robot
 * really does start in the south-west corner of a 16x16 maze. The map's
 * perimeter is pre-set by MazeMap_Init(), so the west and south boundaries are
 * already walls before the robot looks at them.
 *
 * !! THE POSE MUST MATCH WHERE THE ROBOT PHYSICALLY IS. !! These coordinates
 * are labels, not measurements -- nothing checks them against the arena, and a
 * mismatch does not announce itself. It shows up later as the run ending with
 * NAV_END_OFF_MAP, because the robot drove through an opening the map says is
 * a boundary wall.
 *
 * That is not hypothetical here. This was 8,0 for exactly that reason: a bench
 * arena that opens west of the start column, and a run that found the opening,
 * drove through it and left the map behind. The last recorded run travelled
 * SEVEN CELLS WEST of its start column, from x=8 down to x=1. If the robot is
 * not genuinely against the west boundary when it is placed, that run from
 * (0,0) aborts on its second westward cell.
 *
 * Starting mid-map costs nothing when the arena is a bench setup: the
 * coordinates are arbitrary, the recorded walls are the same shape wherever
 * they land, and none of the pre-set boundary walls sit on top of readings the
 * robot actually took. Move it back if the map runs out of room to the west. */
#define NAV_START_X   0
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
 * end of a move is unambiguous.
 *
 * This is the EXPENSIVE one and it used to be paid at every cell. With chained
 * motion the walls are read on the way in and the stationary read is the
 * fallback, so a generous figure now costs almost nothing -- and the times it
 * is reached are exactly the times the robot could not get a confident answer
 * any other way, which is when it should be careful. */
#define NAV_SETTLE_MS 800U

/* Pause after a pivot, in ms. SEPARATE FROM THE ABOVE, because it is on the
 * critical path of every turn and it is not guarding the same thing.
 *
 * The wall-reading pause exists so five sensor votes are not taken during a
 * rock. This one exists so the next move does not begin while the chassis is
 * still swinging -- and the turn controller has already waited
 * TURN_PROFILE_SETTLE_MS with the yaw rate under TURN_SETTLE_RATE_DPS before
 * it returned, so most of that has happened already.
 *
 * It was 800 by sharing the constant above, which cost two thirds of a second
 * per pivot to re-confirm something the turn controller had just confirmed. */
#define NAV_PIVOT_SETTLE_MS 150U

/* Hard bound on the run, counted in CELLS ENTERED. A wall follower in an open
 * area circles forever, and a bench arena has no outer boundary to stop it, so
 * this is the only thing that guarantees the robot ends up somewhere you can
 * reach it.
 *
 * Raised 24 -> 48 after a run used the whole 24 without a single failed move.
 * Measured pace on that run was 5.2 s per cell including the stop-and-vote at
 * each centre, so 48 is about four minutes of driving -- long enough that
 * battery state becomes part of the experiment, since the turn feedforward
 * moves with it. */
#define NAV_MAX_MOVES 48U

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
#define NAV_END_STALLED     6U   /* wedged: commanded hard, went nowhere   */

/* Actions the decision rule can produce. Recorded per cell so the log shows
 * WHY the robot did what it did, not just where it ended up. */
#define NAV_ACT_NONE     0U      /* no decision made (run ending)       */
#define NAV_ACT_FORWARD  1U      /* advance one cell                    */
#define NAV_ACT_LEFT     2U      /* turn left,  then advance one cell   */
#define NAV_ACT_RIGHT    3U      /* turn right, then advance one cell   */
#define NAV_ACT_AROUND   4U      /* reverse one cell, THEN turn 180     */
#define NAV_ACT_STOP     5U      /* logged on the final record          */

/* Per-cell record of what the run actually did. One entry per completed action
 * plus one for the start cell, so this must outlast the move budget -- if it
 * does not, the run stops on a full buffer instead of on the budget and the
 * reason is reported as NAV_END_TRACE_FULL. Keep it comfortably above
 * NAV_MAX_MOVES + 2, which covers the budget, the final record, and the extra
 * one a failed move writes. At 48 bytes each this costs 3072 bytes of RAM. */
#define MAZE_TRACE_CAPACITY 64U

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
  /* Vote tallies out of WALL_SENSE_SAMPLES, 3 bits each, plus the alignment
   * reason in the three bits above them:
   *     front | left << 3 | right << 6 | SL_ALIGN_* << 9
   *
   * The reason rides here because the struct has no padding left and its
   * stride is load-bearing -- see the assert below. align_delta_cm already
   * says how far the front-wall alignment moved a move's endpoint; the reason
   * says why it did not, which is the half that was missing. A run where the
   * alignment fires once in nineteen cells is either an arena that offered no
   * chances or a window that has closed, and those want opposite responses.
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
  /* What the lateral loop has learned about the heading reference so far.
   *
   * Per cell rather than per move because it is supposed to CONVERGE over a
   * run: rising early and then flat is the term working, still climbing at
   * the last cell means it never got there, and pinned at
   * WALL_FOLLOW_KI_LIMIT_DEG means the asymmetry is mechanical. None of that
   * is visible in a single move's trace. */
  /* Whether entry_err_mm below means anything: a move with no wall in view
   * the whole way has no entry error to report, and zero is a plausible
   * value rather than an obviously absent one. Sits in padding the compiler
   * was already inserting after `action`. */
  uint8_t entry_valid;
  float drift_deg;
  /* THE LATERAL ERROR THE ARRIVING MOVE STARTED WITH.
   *
   * Paired with the move_error_cm above, this says whether a cell's lateral
   * offset was inherited or created. It exists to answer one question the
   * logs could not: does a PIVOT throw the robot sideways? A run came out of
   * a dead end 46 mm further from the same wall than it went in, across one
   * 180 and one cell of travel, and nothing recorded which of the two did it.
   * The per-cycle trace only survives the last move; this survives all of
   * them. */
  float entry_err_mm;
  /* How far the front-wall alignment moved this move's endpoint, cm, or zero
   * if it never fired. Which moves align and by how much was previously only
   * inferable from where the robot happened to stop. */
  float align_delta_cm;
} MazeTrace_t;

_Static_assert(sizeof(MazeTrace_t) == 48,
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
 * Only an outside reference fixes that -- a front wall at a known distance.
 *
 * WHICH IS WHY IT STANDS DOWN WHEN THE FRONT WALL SPEAKS. A move that ended on
 * the front-wall alignment reports zero here, whatever the encoders think. The
 * outside reference has already placed the robot; the encoder disagreement is
 * the measure of how wrong the encoders were, not of how far the robot still
 * has to go. Carrying it would apply the correction twice. */
extern volatile float tm_maze_residual_cm;

/* Continuous-ranging mode switching, per run. Both should read 0.
 *
 * A failed START means the run went ahead on blocking single-shot reads, so it
 * is slow but correct -- read the timing before concluding anything about the
 * tuning. A failed STOP is the more serious one: the stop is only a request
 * and the part is in an undefined state if it is reconfigured during the
 * window, so a 1 here means the next thing to touch the sensors may find them
 * in a state it does not expect. */
extern volatile uint8_t tm_maze_tof_start_fail;
extern volatile uint8_t tm_maze_tof_stop_fail;

/* BLOCKING. Runs the whole exploration and returns when it ends; check
 * tm_maze_abort_reason for why. Calling it again after a run has completed
 * does nothing, because a second lap would drive the robot back through an
 * arena it has already left. */
void Navigator_Run(void);

#endif /* NAVIGATOR_H */
