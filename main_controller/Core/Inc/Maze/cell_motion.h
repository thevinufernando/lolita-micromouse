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
 * Takes the pose it is about to leave so it can tell the wall follower which
 * cells its side sensors will see. */
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

#endif /* CELL_MOTION_H */
