/* PORTED FROM MicroMouseAlgorithm/Main.c.
 *
 * The phase machine, the globals and the arrays are the original. Two changes,
 * both at the boundary rather than in the logic:
 *
 *   - main() is FloodFill_Run(), because the firmware already has a main().
 *   - <stdio.h> is gone; nothing here used it once debug_log moved out.
 *
 * THE POSE LIVES HERE. mouse_x, mouse_y and mouse_dir are updated by this file
 * after every move, exactly as they were on the simulator, and the robot-side
 * shim must never touch them -- see mms_api.c. */
#include <stdbool.h>
#include <stdint.h>
#include "API.h"
#include "maze.h"
#include "floodfill_run.h"

// Mouse state
int mouse_x = 0, mouse_y = 0;
Direction mouse_dir = NORTH;
Phase current_phase = EXPLORE_TO_GOAL;

// Wall and distance arrays
bool v_walls[MAZE_SIZE][MAZE_SIZE + 1];
bool h_walls[MAZE_SIZE + 1][MAZE_SIZE];
uint8_t dist[MAZE_SIZE][MAZE_SIZE];
bool visited_to_goal[MAZE_SIZE][MAZE_SIZE];
bool visited_to_start[MAZE_SIZE][MAZE_SIZE];
bool visited_speed[MAZE_SIZE][MAZE_SIZE];
bool backtrack_cells[MAZE_SIZE][MAZE_SIZE];
bool revisited_cells[MAZE_SIZE][MAZE_SIZE];
bool dead_end_cells[MAZE_SIZE][MAZE_SIZE];

// Queue for floodfill
int queue[QUEUE_SIZE][2];
int queue_head = 0, queue_tail = 0;

void FloodFill_Run(void) {
    debug_log("Starting Floodfill...");
    initWalls();
    for (int y = 0; y < MAZE_SIZE; y++) {
        for (int x = 0; x < MAZE_SIZE; x++) {
            visited_to_goal[y][x] = false;
            visited_to_start[y][x] = false;
        }
    }
    visited_to_goal[0][0] = true;
    floodfill_phase(EXPLORE_TO_GOAL);

    API_setColor(0, 0, 'G');
    API_setText(0, 0, "Start");

    while (1) {
        if (API_wasReset()) {
            debug_log("Reset detected!");
            API_ackReset();
            mouse_x = 0;
            mouse_y = 0;
            mouse_dir = NORTH;
            current_phase = EXPLORE_TO_GOAL;
            initWalls();
            floodfill_phase(EXPLORE_TO_GOAL);
            for (int y = 0; y < MAZE_SIZE; y++)
                for (int x = 0; x < MAZE_SIZE; x++)
                    visited_speed[y][x] = false;
            for (int y = 0; y < MAZE_SIZE; y++)
                for (int x = 0; x < MAZE_SIZE; x++)
                    backtrack_cells[y][x] = false;
            for (int y = 0; y < MAZE_SIZE; y++)
                for (int x = 0; x < MAZE_SIZE; x++)
                    revisited_cells[y][x] = false;
            for (int y = 0; y < MAZE_SIZE; y++) {
                for (int x = 0; x < MAZE_SIZE; x++) {
                    visited_to_goal[y][x] = false;
                    visited_to_start[y][x] = false;
                }
            }
            visited_to_goal[0][0] = true;
            API_clearAllColor();
            API_clearAllText();
            API_setColor(0, 0, 'G');
            API_setText(0, 0, "Start");
            continue;
        }

        if (API_mazeWidth() != MAZE_SIZE || API_mazeHeight() != MAZE_SIZE) {
            debug_log("Error: Maze size mismatch!");
            break;
        }

    if (current_phase == EXPLORE_TO_GOAL && (mouse_x == 7 || mouse_x == 8) && (mouse_y == 7 || mouse_y == 8)) {
            // Only transition after ALL 4 goal-center cells have been physically visited
            bool all_goal_cells_visited =
                visited_to_goal[7][7] && visited_to_goal[7][8] &&
                visited_to_goal[8][7] && visited_to_goal[8][8];
            if (all_goal_cells_visited) {
                API_setColor(mouse_x, mouse_y, 'R');
                debug_log("All 4 goal cells explored! Initiating exploratory return trip...");
                for (int y = 0; y < MAZE_SIZE; y++) {
                    for (int x = 0; x < MAZE_SIZE; x++) {
                        if (!visited_to_goal[y][x]) {
                            dist[y][x] = MAX_DIST;
                        }
                    }
                }
                current_phase = EXPLORE_TO_START;
                for (int y = 0; y < MAZE_SIZE; y++)
                    for (int x = 0; x < MAZE_SIZE; x++)
                        visited_to_start[y][x] = false;
                visited_to_start[mouse_y][mouse_x] = true;
                floodfill_phase(EXPLORE_TO_START);
                continue;
            } else {
                // Still in the goal area but not all cells visited; keep exploring other goal cells
                debug_log("At goal area, continuing to visit remaining goal-center cells...");
            }
        }

    if (current_phase == EXPLORE_TO_START && mouse_x == 0 && mouse_y == 0) {
            API_setColor(mouse_x, mouse_y, 'G');
            debug_log("Returned to start! Initiating speed run to goal...");
            for (int y = 0; y < MAZE_SIZE; y++) {
                for (int x = 0; x < MAZE_SIZE; x++) {
                    if (!visited_to_start[y][x]) {
                        dist[y][x] = MAX_DIST; // unvisited set to unknown
                    }
                }
            }
            current_phase = SPEED_TO_GOAL;
            for (int y = 0; y < MAZE_SIZE; y++)
                for (int x = 0; x < MAZE_SIZE; x++)
                    visited_speed[y][x] = false;
            visited_speed[mouse_y][mouse_x] = true;
            // Prepare and run specialized speed run floodfill (which inherently ignores invalid cells)
            floodfill_speed_run();
            show_dist();
            continue;
        }

    if (current_phase == SPEED_TO_GOAL && dist[mouse_y][mouse_x] == 0) {
            API_setColor(mouse_x, mouse_y, 'R');
            debug_log("Speed run to goal complete!");
            break;
        }

        if (current_phase == EXPLORE_TO_GOAL) {
            updateWalls();
            floodfill_phase(EXPLORE_TO_GOAL);
        } else if (current_phase == EXPLORE_TO_START) {
            updateWalls();
            floodfill_phase(EXPLORE_TO_START);
        } else {
            show_dist();
        }

        int dir = getBestDirection();
        if ((current_phase == EXPLORE_TO_GOAL || current_phase == EXPLORE_TO_START) && dir == 2) {
            backtrack_cells[mouse_y][mouse_x] = true;
        }
        if (dir == -1) {
            API_turnLeft();
            mouse_dir = (mouse_dir + 3) % 4;
        } else if (dir == 1) {
            API_turnRight();
            mouse_dir = (mouse_dir + 1) % 4;
        } else if (dir == 2) {
            API_turnLeft();
            API_turnLeft();
            mouse_dir = (mouse_dir + 2) % 4;
        }

        if (API_moveForward() == 0) {
            debug_log("Crash detected!");
            break;
        }

        if (mouse_dir == NORTH) mouse_y++;
        else if (mouse_dir == EAST) mouse_x++;
        else if (mouse_dir == SOUTH) mouse_y--;
        else if (mouse_dir == WEST) mouse_x--;

        if (current_phase == EXPLORE_TO_GOAL) {
            visited_to_goal[mouse_y][mouse_x] = true;
            static bool first_visit_goal[MAZE_SIZE][MAZE_SIZE] = {false};
            if (first_visit_goal[mouse_y][mouse_x]) {
                if (!is_junction(mouse_y, mouse_x)) revisited_cells[mouse_y][mouse_x] = true;
            } else {
                first_visit_goal[mouse_y][mouse_x] = true;
            }
        } else if (current_phase == EXPLORE_TO_START) {
            visited_to_start[mouse_y][mouse_x] = true;
            static bool first_visit_start[MAZE_SIZE][MAZE_SIZE] = {false};
            if (first_visit_start[mouse_y][mouse_x]) {
                if (!is_junction(mouse_y, mouse_x)) revisited_cells[mouse_y][mouse_x] = true;
            } else {
                first_visit_start[mouse_y][mouse_x] = true;
            }
        }

        char color = (current_phase == SPEED_TO_GOAL) ? 'Y' : 'B';
        API_setColor(mouse_x, mouse_y, color);

        if (current_phase == SPEED_TO_GOAL) {
            visited_speed[mouse_y][mouse_x] = true;
        }
    }

    debug_log("Run complete.");
}