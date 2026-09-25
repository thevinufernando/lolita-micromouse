#ifndef FLOODFILL_RUN_H
#define FLOODFILL_RUN_H

#include "maze.h"

/*
 * ============================================================================
 *            FLOOD FILL, PORTED FROM THE SIMULATOR ONTO THE ROBOT
 * ============================================================================
 *
 * The solver itself is MicroMouseAlgorithm/maze.c and Main.c, brought across
 * essentially unchanged. It knows nothing about this robot: it talks to the
 * world only through API.h, which on the desktop was a pipe to the mms
 * simulator and here is mms_api.c driving real motors and real sensors.
 *
 * ---------------------------------------------------------------------------
 * WHAT OWNS WHAT
 * ---------------------------------------------------------------------------
 * THE ALGORITHM OWNS THE POSE AND THE MAP. mouse_x, mouse_y, mouse_dir,
 * v_walls and h_walls are defined in floodfill_run.c and updated there after
 * every move, exactly as they were on the simulator. The shim must never
 * advance or rotate them -- if both did, they would disagree the first time a
 * move failed, and the map would fill with walls recorded against a pose the
 * robot never held.
 *
 * THE FIRMWARE OWNS THE MOTION. Reading a cell's walls, pivoting 90 degrees
 * and driving one cell are cell_motion.c, which also carries the residual
 * carry, the front-wall alignment interaction, the wall-follower's cell
 * context and the per-cell trace. None of that is visible from here, and none
 * of it had to be rewritten to get this running.
 *
 * ---------------------------------------------------------------------------
 * THE THREE PHASES
 * ---------------------------------------------------------------------------
 *   EXPLORE_TO_GOAL   flood fill toward the four centre cells, updating walls
 *                     from the sensors at every cell. Ends only once ALL FOUR
 *                     centre cells have been physically visited.
 *   EXPLORE_TO_START  flood fill back to (0,0), preferring cells it has not
 *                     seen -- the return trip is still exploration, which is
 *                     what makes the speed run worth having.
 *   SPEED_TO_GOAL     flood fill over visited, non-dead-end cells only, and
 *                     drive it without consulting the sensors for decisions.
 *
 * The goal is hardcoded at (7,7), (7,8), (8,7) and (8,8), so this needs a real
 * 16x16 maze. On a smaller bench arena the first phase never completes,
 * because two of those cells do not exist to be visited.
 * ============================================================================
 */

/* Run the whole thing, blocking until the speed run completes or a move fails.
 * The simulator's main(), under a name that does not collide with the
 * firmware's own. */
void FloodFill_Run(void);

#endif /* FLOODFILL_RUN_H */
