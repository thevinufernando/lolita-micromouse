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

    printf("\nTEST 6: the real arena\n");
    /* (0,0)N -> (0,1)N -> (0,2)N -> turn right -> (0,2)E -> (1,2)E.
     * North to east is a RIGHT turn. The west wall present at every cell
     * along x=0 is the maze's own outer boundary, which MazeMap_Init()
     * already sets, so the robot re-seeing it is consistent, not a conflict. */
    MazeMap_Init();
    MazeMap_UpdateWalls(0, 1, 1);   /* (0,0) facing N: corridor, walls both sides */
    MazeMap_Advance();
    check("advanced to (0,1)", mouse_y, 1);
    MazeMap_UpdateWalls(0, 1, 1);   /* (0,1) */
    MazeMap_Advance();
    check("advanced to (0,2)", mouse_y, 2);
    MazeMap_UpdateWalls(1, 1, 0);   /* (0,2): wall ahead and left, open right */
    check("wall ahead -> north of (0,2)", MazeMap_WallNorth(0, 2), 1);
    check("left wall  -> west of (0,2)",  MazeMap_WallWest(0, 2), 1);
    check("right open -> east of (0,2)",  MazeMap_WallEast(0, 2), 0);
    check("corridor wall east of (0,1)",  MazeMap_WallEast(0, 1), 1);
    MazeMap_TurnRight();
    check("now facing EAST", mouse_dir, EAST);
    MazeMap_Advance();
    check("advanced to (1,2) x", mouse_x, 1);
    check("advanced to (1,2) y", mouse_y, 2);
    check("the wall behind is still recorded", MazeMap_WallWest(1, 2), 0);

    printf("\nTEST 7: walls are never cleared by a later negative reading\n");
    MazeMap_Init();
    mouse_x = 5; mouse_y = 5; mouse_dir = NORTH;
    MazeMap_UpdateWalls(1, 0, 0);
    MazeMap_UpdateWalls(0, 0, 0);   /* a later sweep sees nothing */
    check("wall survives a negative sweep", MazeMap_WallNorth(5, 5), 1);

    /* The edge case that cost a run: advancing out of the maze must REPORT it,
       not clamp silently. */
    MazeMap_Init();
    MazeMap_SetPose(0, 0, WEST);
    check("advance off the west edge is refused", MazeMap_Advance(), 0);
    check("  pose x unchanged", mouse_x, 0);
    check("  pose y unchanged", mouse_y, 0);
    MazeMap_SetPose(0, 0, NORTH);
    check("advance inside the maze succeeds", MazeMap_Advance(), 1);

    /* ---- KNOWN vs OPEN ----
     *
     * The accessors only ever report 1 for a wall that has been SEEN, so a
     * zero from an unvisited cell means nothing at all. Anything reasoning
     * about a cell ahead has to be able to tell those apart. */
    MazeMap_Init();
    check("a fresh map knows nothing", MazeMap_IsKnown(5, 5), 0);
    check("  not even the cell the mouse starts in", MazeMap_IsKnown(0, 0), 0);
    check("  and out of bounds is never known", MazeMap_IsKnown(-1, 0), 0);

    mouse_x = 5; mouse_y = 5; mouse_dir = NORTH;
    MazeMap_UpdateWalls(0, 0, 0);
    check("reading a cell's walls makes it known", MazeMap_IsKnown(5, 5), 1);
    check("  a sweep that saw NO walls still counts", MazeMap_WallNorth(5, 5), 0);
    check("  and the neighbour stays unknown", MazeMap_IsKnown(5, 6), 0);

    /* Moving through a cell is not the same as looking at it. A pose can be
       set or advanced without any walls being read -- after a failed move, for
       instance -- and counting that as known would hand out wall data nobody
       measured. */
    MazeMap_Init();
    MazeMap_SetPose(3, 3, NORTH);
    check("setting a pose does not make a cell known", MazeMap_IsKnown(3, 3), 0);
    (void)MazeMap_Advance();
    check("nor does advancing into one", MazeMap_IsKnown(3, 4), 0);

    /* ---- CellWalls MIRRORS UpdateWalls ----
     *
     * Written from every heading and read back from the same heading: the
     * three answers must come back exactly as they went in. A left/right swap
     * here would produce a mirrored map that is entirely self-consistent. */
    {
        const Direction dirs[4] = { NORTH, EAST, SOUTH, WEST };
        const char *names[4] = { "north", "east", "south", "west" };

        for (int i = 0; i < 4; i++) {
            MazeMap_Init();
            mouse_x = 7; mouse_y = 7; mouse_dir = dirs[i];
            MazeMap_UpdateWalls(1, 0, 1);        /* front and right only */

            uint8_t f = 9, l = 9, r = 9;
            MazeMap_CellWalls(7, 7, dirs[i], &f, &l, &r);

            char msg[64];
            snprintf(msg, sizeof msg, "facing %s: front reads back", names[i]);
            check(msg, f, 1);
            snprintf(msg, sizeof msg, "facing %s: left reads back", names[i]);
            check(msg, l, 0);
            snprintf(msg, sizeof msg, "facing %s: right reads back", names[i]);
            check(msg, r, 1);
        }

        /* The other half of the mirror: a wall written facing one way must be
           found from the heading that looks at the same physical side. Facing
           NORTH the right wall is the EAST side, which a robot facing EAST
           sees as its front. */
        MazeMap_Init();
        mouse_x = 2; mouse_y = 2; mouse_dir = NORTH;
        MazeMap_UpdateWalls(0, 0, 1);            /* east side of (2,2) */

        uint8_t f2 = 9;
        MazeMap_CellWalls(2, 2, EAST, &f2, 0, 0);
        check("a right wall seen facing north is a front wall facing east", f2, 1);

        /* NULL pointers are allowed, because most callers want one side. */
        MazeMap_CellWalls(2, 2, NORTH, 0, 0, 0);
        check("NULL outputs are survivable", 1, 1);

        /* Outside the maze everything is walled, matching the accessors. */
        uint8_t f3 = 9, l3 = 9, r3 = 9;
        MazeMap_CellWalls(-1, 0, NORTH, &f3, &l3, &r3);
        check("out of bounds reads as walled (front)", f3, 1);
        check("out of bounds reads as walled (left)",  l3, 1);
        check("out of bounds reads as walled (right)", r3, 1);
    }

    printf("\n===== %s (%d failures) =====\n\n",
           failures ? "FAILURES" : "ALL CHECKS PASSED", failures);

    return failures ? 1 : 0;
}