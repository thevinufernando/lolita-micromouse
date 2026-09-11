#ifndef MAZE_MAP_H
#define MAZE_MAP_H

#include <stdint.h>

/*
 * ============================================================================
 *                      MAZE MAP - WALLS AND POSE
 * ============================================================================
 *
 * The robot's record of which walls it has seen and where it thinks it is.
 *
 * SCOPE: storage and bookkeeping only. There is no flood fill, no path
 * planning and no decision about where to go next -- that lives in the
 * MicroMouseAlgorithm repo and is deliberately not duplicated here. This
 * module answers "what have I seen" and "where am I", nothing more.
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

#define MAZE_SIZE 16

/* Same ordering as the algorithm side. Do not renumber: the turn helpers
 * below rely on NORTH..WEST being 0..3 clockwise. */
typedef enum { NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3 } Direction;

extern uint8_t v_walls[MAZE_SIZE][MAZE_SIZE + 1];
extern uint8_t h_walls[MAZE_SIZE + 1][MAZE_SIZE];

/* Where the robot believes it is. Updated by the advance/turn helpers, so it
 * is dead reckoning at cell granularity -- it is only as good as the moves
 * that were actually completed. */
extern int16_t mouse_x;
extern int16_t mouse_y;
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
void MazeMap_Advance(void);    /* one cell in the current heading */
void MazeMap_TurnLeft(void);
void MazeMap_TurnRight(void);

/* Wall queries in absolute terms, bounds-checked. */
uint8_t MazeMap_WallNorth(int16_t x, int16_t y);
uint8_t MazeMap_WallEast(int16_t x, int16_t y);
uint8_t MazeMap_WallSouth(int16_t x, int16_t y);
uint8_t MazeMap_WallWest(int16_t x, int16_t y);

#endif /* MAZE_MAP_H */
