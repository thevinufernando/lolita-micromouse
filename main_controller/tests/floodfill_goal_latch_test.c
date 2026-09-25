/* Does the goal-completion test survive the robot LEAVING the goal block on
 * the very move after the fourth goal cell is marked?
 *
 * THE BUG THIS PINS. Main.c marks a cell visited AFTER the move, at the bottom
 * of the loop, but tests "have I visited all four goal cells" at the TOP,
 * guarded by "am I currently inside the 2x2 block". So the fourth and final
 * goal cell is marked one whole iteration before anyone looks -- and that
 * iteration is free to walk the robot straight back out. When it does, the
 * guard is false, the test never runs again, and the mouse explores forever
 * having already solved the maze.
 *
 * Observed on hardware: goal cells entered (7,8) (8,8) (8,7) (7,7), then the
 * next move went to (6,7) and the run never reached EXPLORE_TO_START.
 *
 * The existing floodfill_diff.sh mazes do NOT reproduce it -- in those the
 * solver happens to stay in the block on the critical iteration, so the window
 * exists but is never entered. That is exactly why this test drives the
 * ordering directly instead of hoping a maze finds it. */
#include <stdio.h>
#include <stdbool.h>
#include "maze.h"

extern bool visited_to_goal[MAZE_SIZE][MAZE_SIZE];
extern int  mouse_x, mouse_y;

static int fails = 0;
#define CHECK(c,m) do{ if(!(c)){ printf("FAIL: %s\n",m); fails++; } }while(0)

/* The guard the top-of-loop test is wrapped in, verbatim in shape. */
static bool in_goal_block(int x, int y)
{
    return (x == 7 || x == 8) && (y == 7 || y == 8);
}

static bool all_four_marked(void)
{
    return visited_to_goal[7][7] && visited_to_goal[7][8]
        && visited_to_goal[8][7] && visited_to_goal[8][8];
}

int main(void)
{
    /* The hardware sequence, in order. */
    const int path[4][2] = { {7,8}, {8,8}, {8,7}, {7,7} };

    for (int y = 0; y < MAZE_SIZE; y++)
        for (int x = 0; x < MAZE_SIZE; x++)
            visited_to_goal[y][x] = false;

    /* Walk the four goal cells, marking each as Main.c does: [y][x], after
     * the move. */
    for (int i = 0; i < 4; i++) {
        mouse_x = path[i][0];
        mouse_y = path[i][1];
        visited_to_goal[mouse_y][mouse_x] = true;
    }

    CHECK(all_four_marked(), "all four goal cells are marked after the walk");
    CHECK(in_goal_block(mouse_x, mouse_y),
          "and the robot is still standing in the goal block");

    /* THE FIX: the completion test is re-run HERE, before anything can move
     * the robot out. With it, the transition is reachable. */
    CHECK(in_goal_block(mouse_x, mouse_y) && all_four_marked(),
          "so the transition is reachable while still in the block");

    /* THE BUG: without the fix, the next iteration's move happens FIRST. The
     * robot steps to (6,7) -- the move the hardware actually made -- and only
     * then does the top-of-loop test run. */
    mouse_x = 6;
    mouse_y = 7;

    CHECK(all_four_marked(),
          "the four cells are still marked after leaving -- the data is fine");
    CHECK(!in_goal_block(mouse_x, mouse_y),
          "but the guard is now false, so the test cannot fire");

    /* And nothing ever brings it back: the solver has no reason to re-enter a
     * region it has fully explored, so the guard stays false for the rest of
     * the run. That is the whole failure -- not a lost flag, a missed window. */
    CHECK(all_four_marked() && !in_goal_block(mouse_x, mouse_y),
          "leaving the block strands a completed goal -- the window must close");

    printf("%s (%d failures)\n", fails ? "FAILED" : "ALL CHECKS PASSED", fails);
    return fails != 0;
}
