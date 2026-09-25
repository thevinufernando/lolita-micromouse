#ifndef MAZE_MAP_H
#define MAZE_MAP_H

#include <stdint.h>
#include <stdbool.h>

/* MAZE_SIZE, Direction, v_walls, h_walls and the pose now come from the
 * ALGORITHM, which owns them. This module is a view over that state, not a
 * second copy of it -- see the note below. */
#include "maze.h"

/*
 * ============================================================================
 *                      MAZE MAP - WALLS AND POSE
 * ============================================================================
 *
 * The robot's record of which walls it has seen and where it thinks it is.
 *
 * SCOPE: a VIEW over state the algorithm owns, plus the one thing the
 * algorithm does not track.
 *
 * THE ARRAYS AND THE POSE ARE NOT DEFINED HERE ANY MORE. v_walls, h_walls,
 * mouse_x, mouse_y and mouse_dir live in the ported flood fill
 * (Core/Src/Maze/floodfill/floodfill_run.c), because that is what updates them
 * after every move. This file used to define its own copies under the same
 * names, which was harmless only while the two never met. They meet now.
 *
 * What remains here is the shaped access the firmware wants and the algorithm
 * has no need for: walls by compass side, walls of an arbitrary cell in
 * robot-relative terms, the next cell along a heading, and whether a cell has
 * actually been looked at.
 *
 * ---------------------------------------------------------------------------
 * WALL REPRESENTATION - MUST MATCH THE ALGORITHM SIDE
 * ---------------------------------------------------------------------------
 * Two separate arrays, exactly as in MicroMouseAlgorithm/maze.c, so a map
 * captured off the robot can be dropped straight into the simulator without
 * any conversion:
 *
 *   v_walls[y][x]   VERTICAL wall on the WEST side of cell (x, y)
 *                   so the EAST side of (x, y) is v_walls[y][x + 1]
 *
 *   h_walls[y][x]   HORIZONTAL wall on the SOUTH side of cell (x, y)
 *                   so the NORTH side of (x, y) is h_walls[y + 1][x]
 *
 * Hence the off-by-one array sizes: there are MAZE_SIZE + 1 wall lines in the
 * direction each array spans.
 *
 * COORDINATES: x is the column and y is the row. (0, 0) is the start cell.
 * NORTH is +y and EAST is +x, matching the Direction enum below.
 *
 * A wall is only ever SET, never cleared. Not seeing a wall is not evidence
 * that there is no wall -- it may simply not have been looked at yet. The
 * algorithm treats unset as open, so clearing on a bad reading would invent a
 * passage that does not exist, which is the one error a maze solver cannot
 * recover from.
 * ============================================================================
 */

/* MAZE_SIZE and Direction come from maze.h above. The algorithm declares
 * Direction as { NORTH, EAST, SOUTH, WEST } with the default numbering, which
 * is the 0..3 clockwise order the turn helpers below rely on. */

/* DEFINED BY THE ALGORITHM, in floodfill_run.c. Declared here so firmware code
 * that only wants the view does not have to reach into the solver's headers.
 * The types are the algorithm's -- bool rather than uint8_t, plain int rather
 * than int16_t -- and must not be "tidied" to match the old firmware ones, or
 * the declaration stops matching the definition. */
extern bool v_walls[MAZE_SIZE][MAZE_SIZE + 1];
extern bool h_walls[MAZE_SIZE + 1][MAZE_SIZE];

/* Where the robot believes it is, at cell granularity.
 *
 * WHOEVER IS DRIVING UPDATES THIS, and only one of them ever is. The reactive
 * navigator moves it with MazeMap_Advance() and the turn helpers below; the
 * flood fill moves it itself after each completed move, exactly as it did on
 * the simulator. Two writers would disagree the first time a move failed, and
 * every wall recorded afterwards would land in a cell the robot never stood
 * in -- which is the one error a maze map cannot recover from. */
extern int mouse_x;
extern int mouse_y;
extern Direction mouse_dir;

/* Clear the map, place the mouse at (0,0) facing NORTH, and set the maze's
 * four outer boundary walls (which are always present and are what stop the
 * pose helpers from walking off the array). */
void MazeMap_Init(void);

/* Record walls seen from the current cell, in ROBOT-RELATIVE terms.
 * Direct port of updateWalls() from MicroMouseAlgorithm/maze.c. */
void MazeMap_UpdateWalls(uint8_t front, uint8_t left, uint8_t right);

/* Place the mouse explicitly. MazeMap_Init() uses the competition convention
 * of (0,0) facing NORTH, which is correct for a real maze but puts the west
 * and south boundary walls immediately against the robot. A small bench arena
 * that turns left early needs room on that side, so set the pose to somewhere
 * the arena actually fits -- (0,0) facing EAST works for an L-shaped rig. */
void MazeMap_SetPose(int16_t x, int16_t y, Direction dir);

/* Pose bookkeeping. Call these when a move actually completes, not when it is
 * commanded, so a failed move does not corrupt the position estimate. */
/* One cell in the current heading. Returns 1 if the pose moved, 0 if that
 * would have left the maze -- in which case NOTHING is updated.
 *
 * !! CHECK THE RETURN VALUE !! This used to clamp silently, and that cost a
 * whole run: the robot found an opening on the west side of column 0, drove
 * through it, and the pose stayed put while the machine kept going. Four
 * cells of real wall readings were then written into one cell it was not in,
 * which is unrecoverable -- the map never clears a wall. A caller that
 * ignores this is not tracking the robot, it is tracking a fiction. */
uint8_t MazeMap_Advance(void);

/* The cell one step in `dir` from (x, y), without moving the mouse.
 *
 * Returns 0 when that step would leave the maze, in which case the outputs are
 * untouched -- the same contract as MazeMap_Advance(), which is now written in
 * terms of this so the two can never disagree about where the edge is. */
uint8_t MazeMap_NextCell(int16_t x, int16_t y, Direction dir,
                         int16_t *nx, int16_t *ny);

void MazeMap_TurnLeft(void);
void MazeMap_TurnRight(void);

/* Wall queries in absolute terms, bounds-checked. */
uint8_t MazeMap_WallNorth(int16_t x, int16_t y);
uint8_t MazeMap_WallEast(int16_t x, int16_t y);
uint8_t MazeMap_WallSouth(int16_t x, int16_t y);
uint8_t MazeMap_WallWest(int16_t x, int16_t y);

/* Has this cell's walls actually been READ?
 *
 * MazeMap_UpdateWalls() only ever sets a wall to 1, so a zero from the
 * accessors above means "no wall seen here", which reads identically to "this
 * side is open". That is fine for a cell the robot has stood in and wrong for
 * any cell ahead of it: an unexplored cell otherwise reports as wide open on
 * all four sides. Anything reasoning about a cell it has not visited must ask
 * this first. */
uint8_t MazeMap_IsKnown(int16_t x, int16_t y);

/* Mark a cell as surveyed without writing any walls.
 *
 * For a driver that records walls itself. The flood fill calls the algorithm's
 * own updateWalls(), which writes the same arrays but knows nothing about the
 * visited bitmap this module keeps -- so without this, MazeMap_IsKnown() would
 * answer "no" for every cell of a flood-fill run and the wall follower's cell
 * veto would never fire. */
void MazeMap_MarkKnown(int16_t x, int16_t y);

/* The three robot-relative walls of any cell, for a robot facing `dir`.
 *
 * The exact mirror of MazeMap_UpdateWalls(), and kept beside it so the
 * direction-to-compass arithmetic exists once. Getting a left/right swap wrong
 * here would produce a map that is plausible, self-consistent and mirrored,
 * which is the kind of bug that survives a long time.
 *
 * Any pointer may be NULL. Outside the maze every side reads as walled. */
void MazeMap_CellWalls(int16_t x, int16_t y, Direction dir,
                       uint8_t *front, uint8_t *left, uint8_t *right);

#endif /* MAZE_MAP_H */
