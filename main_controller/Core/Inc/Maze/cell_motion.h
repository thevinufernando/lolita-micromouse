#ifndef CELL_MOTION_H
#define CELL_MOTION_H

#include <stdint.h>
#include "wall_sense.h"

/*
 * ============================================================================
 *            CELL-LEVEL MOTION - THE LAYER EVERY DRIVER SITS ON
 * ============================================================================
 *
 * Three primitives, and everything a maze solver needs from a robot:
 *
 *   observe   come to rest, read this cell's walls
 *   turn      pivot 90 degrees
 *   forward   drive one cell
 *
 * ---------------------------------------------------------------------------
 * A FORWARD DOES NOT STOP AT THE END, AND THAT IS THE POINT
 * ---------------------------------------------------------------------------
 * With MAZE_CONTINUOUS_CELLS set, CellMotion_Forward() ends
 * CELL_DECISION_OFFSET_CM short of the cell centre with the robot still
 * travelling at cruise, and returns. The motors hold their last command while
 * the solver decides what happens next. Another forward simply continues; a
 * turn is preceded by CellMotion_StopAtCell(), which drives the remaining
 * offset and comes to rest at the centre exactly as the robot always did.
 *
 * NOTHING ABOVE THIS LAYER HAD TO CHANGE FOR THAT. Every entry point that
 * needs the robot standing still calls the stop itself -- both pivots,
 * Observe(), and EndRun() -- so a caller cannot forget, and a caller that
 * observes at every cell (the reactive navigator) never chains at all and
 * behaves exactly as it used to.
 *
 * It was all inside navigator.c, private to the right-hand wall follower,
 * until a second driver needed it. What is here is not the interesting part of
 * that file -- the decision rule stayed behind -- but it is the part that took
 * the longest to get right, and re-implementing it for the flood fill would
 * have meant re-learning every one of these the hard way:
 *
 *   THE RESIDUAL CARRY AND THE FRONT-WALL ALIGNMENT MUST NOT BOTH FIRE. The
 *   carry exists because the encoders lose ground; the alignment exists
 *   because the encoders cannot see cell boundaries. When the alignment has
 *   fired it has ALREADY placed the robot, so the gap the encoders report is
 *   not an error left over, it IS the correction -- and carrying it applies
 *   the same correction twice.
 *
 *   A PIVOT INVALIDATES EVERY ToF HISTORY, so the filters are reset and the
 *   sample that straddles the turn is discarded. The carried residual is
 *   discarded too: it is an error along the direction of travel, and a pivot
 *   makes that the lateral axis, where the number means nothing.
 *
 *   THE WALL FOLLOWER NEEDS TO KNOW WHICH CELL IT IS LOOKING AT, because the
 *   side sensors lead the axle and cross the boundary a third of the way into
 *   a move. That context is set from the map before every forward.
 *
 * ---------------------------------------------------------------------------
 * WHAT THIS LAYER DOES NOT DO
 * ---------------------------------------------------------------------------
 * IT NEVER MOVES THE POSE. mouse_x, mouse_y and mouse_dir belong to whoever is
 * driving, and the two drivers keep them differently: the reactive navigator
 * calls the map helpers, the flood fill updates them itself as it always did
 * on the simulator. If this layer also moved them they would disagree the
 * first time a move failed.
 *
 * It also makes no decisions. It reports what it sees and does what it is
 * told.
 * ============================================================================
 */

/* Prepare for a run: free-run the ToF sensors, clear the trace and the learned
 * lateral bias, and zero the carried residual. Pair with CellMotion_EndRun(). */
void CellMotion_BeginRun(void);

/* Put the sensors back to single-shot and brake. Stopping continuous ranging
 * is the awkward path on this part -- the call only REQUESTS the stop -- so it
 * belongs on one exit rather than at each place a run can end. */
void CellMotion_EndRun(void);

/* Drive the last stretch to the current cell's centre and stop.
 *
 * Does nothing when the robot is already at rest, so it is safe to call
 * anywhere. Returns 0 if that last stretch failed.
 *
 * Callers do not normally need this: the pivots, Observe() and EndRun() all
 * call it themselves. It is exposed for a driver that wants the robot stopped
 * for a reason of its own. */
uint8_t CellMotion_StopAtCell(void);

/* The walls of the cell just entered, read WHILE ENTERING IT.
 *
 * Returns 0 when the last forward did not gather enough samples to be worth
 * believing, or when anything has happened since that invalidates them -- a
 * pivot, a failed move, the start of a run. A driver that gets 0 here should
 * fall back to CellMotion_Observe(), which is slow and always right.
 *
 * Fills wall_front_mm and friends on success, so the per-cell trace records
 * the same columns whichever path produced the answer. */
uint8_t CellMotion_FlightWalls(WallReading_t *w);

/* Come to rest and read this cell's walls, several times, and vote.
 *
 * `write_map` records them against the current pose. Pass 0 after a FAILED
 * move: the robot is then somewhere between two cells and the readings belong
 * to no cell the map can name, so they are worth logging and not worth
 * storing. A wall written into the wrong cell is never undone. */
void CellMotion_Observe(WallReading_t *w, uint8_t write_map);

/* Pivot 90 degrees. Returns 0 if the turn timed out.
 *
 * Does the settle and the post-pivot housekeeping, and does NOT touch the
 * pose. */
uint8_t CellMotion_TurnLeft(void);
uint8_t CellMotion_TurnRight(void);

/* Drive one cell, carrying forward whatever the last move left short.
 * Returns 0 if the move failed or the robot wedged.
 *
 * Reads the pose it is about to leave, so it can tell the wall follower which
 * cells its side sensors will see, and leaves the robot ROLLING at the next
 * decision point -- see the note at the top of this file. */
uint8_t CellMotion_Forward(void);

/* Write one cell into the trace: where the robot is, what it saw, how the move
 * that got here went, and what was decided from it. Both drivers call this;
 * the action codes are the NAV_ACT_* set in navigator.h. */
void CellMotion_Record(float move_error_cm, uint8_t move_ok,
                       const WallReading_t *w, uint8_t action);

/* Distance error carried into the next move, cm. Exposed because the trace
 * records it per cell and it is the clearest single number for whether the
 * robot is keeping its place along a corridor. */
extern volatile float tm_maze_residual_cm;

/* Continuous-ranging mode switching, per run. Both should read 0. A failed
 * START means the run went ahead on blocking single-shot reads, so it is slow
 * but correct. A failed STOP means the sensors may be in a state the next
 * caller does not expect. */
extern volatile uint8_t tm_maze_tof_start_fail;
extern volatile uint8_t tm_maze_tof_stop_fail;

/* ---- How chained motion is actually going ----
 *
 * tm_chain_gap_ms_max is the one to watch. It is the longest the motors ever
 * held their last command with nothing watching, waiting for the solver to
 * decide, and CELL_DECISION_MARGIN_CM is what pays for it: 1.5 cm of slack is
 * about 107 ms at cruise. If this ever approaches that, the margin is being
 * spent on think-time instead of on stopping.
 *
 * The two read counters say how often the robot got its walls for free against
 * how often it had to stop and vote for them. A run where the second is more
 * than the odd cell is a run where chaining is not paying for itself, and the
 * in-flight window or WALL_FLIGHT_MIN_SAMPLES is why. */
extern volatile uint32_t tm_chain_gap_ms_max;
extern volatile uint32_t tm_chain_segments;
extern volatile uint32_t tm_chain_flight_reads;
extern volatile uint32_t tm_chain_stop_reads;

#endif /* CELL_MOTION_H */
