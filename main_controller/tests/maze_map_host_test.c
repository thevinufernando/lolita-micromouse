/* Host-side verification of the maze map's wall indexing.
 *
 * This exists because the v_walls/h_walls convention is the single point where
 * the firmware and MicroMouseAlgorithm must agree exactly, and an off-by-one
 * there produces a map that is wrong but entirely plausible -- the robot fills
 * in walls, nothing errors, and the solver quietly plans through a wall.
 *
 * The strongest check here is test 4: a wall between two cells must read as
 * present from BOTH sides. That invariant is what an index slip breaks, and it
 * holds regardless of which cell or heading the wall was seen from.
 *
 * Runs on the host, no hardware. Exit code 0 = pass. */
#include "maze_map.h"
#include <stdio.h>

static int failures = 0;

static void check(const char *name, int got, int expect)
{
    int ok = (got == expect);
    if (!ok) failures++;
    printf("  [%s] %-52s got=%d expect=%d\n", ok ? "PASS" : "FAIL", name, got, expect);
}

int main(void)
{
    printf("\n===== maze map host tests =====\n");

    printf("\nTEST 1: init clears the interior and walls the boundary\n");
    MazeMap_Init();
    check("start pose x", mouse_x, 0);
    check("start pose y", mouse_y, 0);
    check("start heading is NORTH", mouse_dir, NORTH);
    check("south boundary walled", h_walls[0][0], 1);
    check("west boundary walled", v_walls[0][0], 1);
    check("north boundary walled", h_walls[MAZE_SIZE][5], 1);
    check("east boundary walled", v_walls[5][MAZE_SIZE], 1);
    check("interior wall starts clear", h_walls[3][3], 0);

    printf("\nTEST 2: facing NORTH, each relative wall lands on the right line\n");
    MazeMap_Init();
    mouse_x = 5; mouse_y = 5; mouse_dir = NORTH;
    MazeMap_UpdateWalls(1, 1, 1);
    check("front -> north side of cell", h_walls[6][5], 1);
    check("left  -> west side of cell",  v_walls[5][5], 1);
    check("right -> east side of cell",  v_walls[5][6], 1);
    check("south side untouched",        h_walls[5][5], 0);

    printf("\nTEST 3: facing EAST, the same walls rotate with the robot\n");
    MazeMap_Init();
    mouse_x = 5; mouse_y = 5; mouse_dir = EAST;
    MazeMap_UpdateWalls(1, 1, 1);
    check("front -> east side",  v_walls[5][6], 1);
    check("left  -> north side", h_walls[6][5], 1);
    check("right -> south side", h_walls[5][5], 1);
    check("west side untouched", v_walls[5][5], 0);

    printf("\nTEST 4: a shared wall reads the same from both cells\n");
    MazeMap_Init();
    mouse_x = 5; mouse_y = 5; mouse_dir = NORTH;
    MazeMap_UpdateWalls(1, 0, 0);          /* wall between (5,5) and (5,6) */
    check("north of (5,5)", MazeMap_WallNorth(5, 5), 1);
    check("south of (5,6) is the SAME wall", MazeMap_WallSouth(5, 6), 1);
    MazeMap_Init();
    mouse_x = 5; mouse_y = 5; mouse_dir = NORTH;
    MazeMap_UpdateWalls(0, 0, 1);          /* wall between (5,5) and (6,5) */
    check("east of (5,5)", MazeMap_WallEast(5, 5), 1);
    check("west of (6,5) is the SAME wall", MazeMap_WallWest(6, 5), 1);

    printf("\nTEST 5: turning and advancing\n");
    MazeMap_Init();
    MazeMap_TurnLeft();  check("NORTH turn left  -> WEST", mouse_dir, WEST);
    MazeMap_TurnLeft();  check("WEST  turn left  -> SOUTH", mouse_dir, SOUTH);
    MazeMap_TurnRight(); check("SOUTH turn right -> WEST", mouse_dir, WEST);
    MazeMap_Init();
    MazeMap_Advance();   check("north advance raises y", mouse_y, 1);
    MazeMap_TurnRight(); MazeMap_Advance();
    check("east advance raises x", mouse_x, 1);
    check("east advance leaves y", mouse_y, 1);

    printf("\nTEST 6: the arena -- fwd, fwd, (front wall), turn left, fwd\n");
    /* Facing EAST from (0,0), not NORTH. Facing NORTH the left turn at the
     * third cell would head WEST straight into the outer boundary, which is
     * not a code problem but does mean the bench arena has to be placed where
     * it fits. Facing EAST the whole L-shape sits inside the maze. */
    MazeMap_Init();
    MazeMap_SetPose(0, 0, EAST);
    MazeMap_UpdateWalls(0, 1, 1);   /* (0,0): corridor east, both sides walled */
    MazeMap_Advance();
    MazeMap_UpdateWalls(0, 1, 1);   /* (1,0) */
    MazeMap_Advance();
    MazeMap_UpdateWalls(1, 0, 1);   /* (2,0): wall ahead, open to the left */
    check("mouse reached (2,0) x", mouse_x, 2);
    check("mouse reached (2,0) y", mouse_y, 0);
    check("wall ahead recorded east of (2,0)", MazeMap_WallEast(2, 0), 1);
    check("left (north) is open at (2,0)", MazeMap_WallNorth(2, 0), 0);
    check("right (south) walled at (2,0)", MazeMap_WallSouth(2, 0), 1);
    check("side walls of the corridor recorded", MazeMap_WallNorth(1, 0), 1);
    MazeMap_TurnLeft();
    check("now facing NORTH", mouse_dir, NORTH);
    MazeMap_Advance();
    check("advanced to (2,1) x", mouse_x, 2);
    check("advanced to (2,1) y", mouse_y, 1);

    printf("\nTEST 7: walls are never cleared by a later negative reading\n");
    MazeMap_Init();
    mouse_x = 5; mouse_y = 5; mouse_dir = NORTH;
    MazeMap_UpdateWalls(1, 0, 0);
    MazeMap_UpdateWalls(0, 0, 0);   /* a later sweep sees nothing */
    check("wall survives a negative sweep", MazeMap_WallNorth(5, 5), 1);

    printf("\n===== %s (%d failures) =====\n\n",
           failures ? "FAILURES" : "ALL CHECKS PASSED", failures);
    return failures ? 1 : 0;
}
