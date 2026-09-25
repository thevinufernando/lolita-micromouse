/* PORTED FROM MicroMouseAlgorithm/maze.c, and deliberately almost unchanged.
 *
 * Every line of floodfill_phase(), updateWalls(), getBestDirection(),
 * checkDeadEnd() and floodfill_speed_run() is the original. ONE thing had to
 * go, and only because it has no meaning on a microcontroller: the body of
 * debug_log(), which was fprintf(stderr). It now lives in mms_api.c with the
 * rest of the simulator boundary.
 *
 * <stdio.h> stays, because show_dist() genuinely uses snprintf and newlib
 * provides it. Only the stderr stream was the problem.
 *
 * If this file ever needs a real change, change it in MicroMouseAlgorithm
 * first and re-port. tests/floodfill_host_test.c compiles both copies against
 * the same mazes and fails if their decisions ever differ, which is what keeps
 * that honest. */
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include "API.h"
#include "maze.h"

// Global variables (extern declarations should be in maze.h if needed by other files)
extern int mouse_x, mouse_y;
extern Direction mouse_dir;
extern Phase current_phase;
extern bool v_walls[MAZE_SIZE][MAZE_SIZE + 1];
extern bool h_walls[MAZE_SIZE + 1][MAZE_SIZE];
extern uint8_t dist[MAZE_SIZE][MAZE_SIZE];
extern bool visited_to_goal[MAZE_SIZE][MAZE_SIZE];
extern bool visited_to_start[MAZE_SIZE][MAZE_SIZE];
extern bool visited_speed[MAZE_SIZE][MAZE_SIZE];
extern bool backtrack_cells[MAZE_SIZE][MAZE_SIZE];
extern bool revisited_cells[MAZE_SIZE][MAZE_SIZE];
extern int queue[QUEUE_SIZE][2];
extern int queue_head, queue_tail;
extern Phase current_phase; // need phase to restrict dead-end filling to exploration
extern bool dead_end_cells[MAZE_SIZE][MAZE_SIZE];

// Helper: returns true if cell is a junction (3+ open directions)
bool is_junction(int y, int x) {
    int open = 0;
    if (y < MAZE_SIZE - 1 && !h_walls[y + 1][x]) open++;
    if (y > 0 && !h_walls[y][x]) open++;
    if (x < MAZE_SIZE - 1 && !v_walls[y][x + 1]) open++;
    if (x > 0 && !v_walls[y][x]) open++;
    return open >= 3;
}

// Floodfill for current phase target (goal center or start)
void floodfill_phase(Phase phase) {
    queue_head = queue_tail = 0;
    for (int y = 0; y < MAZE_SIZE; y++)
        for (int x = 0; x < MAZE_SIZE; x++)
            dist[y][x] = MAX_DIST;

    if (phase == EXPLORE_TO_GOAL || phase == SPEED_TO_GOAL) {
        enqueue(7, 7); dist[7][7] = 0;
        enqueue(7, 8); dist[7][8] = 0;
        enqueue(8, 7); dist[8][7] = 0;
        enqueue(8, 8); dist[8][8] = 0;
    } else { // to start
        enqueue(0, 0); dist[0][0] = 0;
    }

    int cy, cx;
    while (dequeue(&cy, &cx)) {
        uint8_t d = dist[cy][cx] + 1;
        // North
        if (cy < MAZE_SIZE - 1 && !h_walls[cy + 1][cx] && dist[cy + 1][cx] > d) { dist[cy + 1][cx] = d; enqueue(cy + 1, cx);}    
        // East
        if (cx < MAZE_SIZE - 1 && !v_walls[cy][cx + 1] && dist[cy][cx + 1] > d) { dist[cy][cx + 1] = d; enqueue(cy, cx + 1);}    
        // South
        if (cy > 0 && !h_walls[cy][cx] && dist[cy - 1][cx] > d) { dist[cy - 1][cx] = d; enqueue(cy - 1, cx);}    
        // West
        if (cx > 0 && !v_walls[cy][cx] && dist[cy][cx - 1] > d) { dist[cy][cx - 1] = d; enqueue(cy, cx - 1);}    
    }
}

// Queue helpers
void enqueue(int y, int x) {
    if (queue_tail >= QUEUE_SIZE) return;
    queue[queue_tail][0] = y;
    queue[queue_tail][1] = x;
    queue_tail++;
}

bool dequeue(int *y, int *x) {
    if (queue_head >= queue_tail) return false;
    *y = queue[queue_head][0];
    *x = queue[queue_head][1];
    queue_head++;
    return true;
}

// Initialize walls
void initWalls() {
    for (int y = 0; y < MAZE_SIZE; y++) {
        for (int x = 0; x < MAZE_SIZE + 1; x++) {
            v_walls[y][x] = false;
        }
    }
    for (int y = 0; y < MAZE_SIZE + 1; y++) {
        for (int x = 0; x < MAZE_SIZE; x++) {
            h_walls[y][x] = false;
        }
    }
    for (int i = 0; i < MAZE_SIZE; i++) {
        v_walls[i][0] = true;
        v_walls[i][MAZE_SIZE] = true;
        h_walls[0][i] = true;
        h_walls[MAZE_SIZE][i] = true;
    }
}

// Update walls based on sensors
void updateWalls() {
    if (API_wallFront()) {
        if (mouse_dir == NORTH) {
            h_walls[mouse_y + 1][mouse_x] = true;
            API_setWall(mouse_x, mouse_y, 'n');
        } else if (mouse_dir == EAST) {
            v_walls[mouse_y][mouse_x + 1] = true;
            API_setWall(mouse_x, mouse_y, 'e');
        } else if (mouse_dir == SOUTH) {
            h_walls[mouse_y][mouse_x] = true;
            API_setWall(mouse_x, mouse_y, 's');
        } else if (mouse_dir == WEST) {
            v_walls[mouse_y][mouse_x] = true;
            API_setWall(mouse_x, mouse_y, 'w');
        }
    }
    if (API_wallLeft()) {
        if (mouse_dir == NORTH) {
            v_walls[mouse_y][mouse_x] = true;
            API_setWall(mouse_x, mouse_y, 'w');
        } else if (mouse_dir == EAST) {
            h_walls[mouse_y + 1][mouse_x] = true;
            API_setWall(mouse_x, mouse_y, 'n');
        } else if (mouse_dir == SOUTH) {
            v_walls[mouse_y][mouse_x + 1] = true;
            API_setWall(mouse_x, mouse_y, 'e');
        } else if (mouse_dir == WEST) {
            h_walls[mouse_y][mouse_x] = true;
            API_setWall(mouse_x, mouse_y, 's');
        }
    }
    if (API_wallRight()) {
        if (mouse_dir == NORTH) {
            v_walls[mouse_y][mouse_x + 1] = true;
            API_setWall(mouse_x, mouse_y, 'e');
        } else if (mouse_dir == EAST) {
            h_walls[mouse_y][mouse_x] = true;
            API_setWall(mouse_x, mouse_y, 's');
        } else if (mouse_dir == SOUTH) {
            v_walls[mouse_y][mouse_x] = true;
            API_setWall(mouse_x, mouse_y, 'w');
        } else if (mouse_dir == WEST) {
            h_walls[mouse_y + 1][mouse_x] = true;
            API_setWall(mouse_x, mouse_y, 'n');
        }
    }

    // Mark dead-end cells only during exploration phases (do not modify walls here)
    if (current_phase == EXPLORE_TO_GOAL || current_phase == EXPLORE_TO_START) {
        checkDeadEnd();
    }
}

// Backwards compatibility wrappers (optional - can be removed if not referenced elsewhere)
void floodfill_to_goal() { floodfill_phase(EXPLORE_TO_GOAL); }
void floodfill_to_start() { floodfill_phase(EXPLORE_TO_START); }

// Choose best direction
int getBestDirection() {
    uint8_t min_dist = MAX_DIST;
    int best_dir = 0;
    bool found = false;

    // Metadata for tie-breaking during EXPLORE_TO_START
    bool best_unexplored_global = false; // not visited in either phase
    bool best_unvisited_start = false;   // not yet visited in EXPLORE_TO_START phase

    bool use_sensors = (current_phase != SPEED_TO_GOAL);
    bool wall_front = false, wall_left = false, wall_right = false, wall_back = false;

    uint8_t (*use_dist)[MAZE_SIZE] = dist;

    if (use_sensors) {
        wall_front = API_wallFront();
        wall_left = API_wallLeft();
        wall_right = API_wallRight();
    } else {
        switch (mouse_dir) {
            case NORTH:
                wall_front = h_walls[mouse_y + 1][mouse_x];
                wall_left = v_walls[mouse_y][mouse_x];
                wall_right = v_walls[mouse_y][mouse_x + 1];
                wall_back = h_walls[mouse_y][mouse_x];
                break;
            case EAST:
                wall_front = v_walls[mouse_y][mouse_x + 1];
                wall_left = h_walls[mouse_y + 1][mouse_x];
                wall_right = h_walls[mouse_y][mouse_x];
                wall_back = v_walls[mouse_y][mouse_x];
                break;
            case SOUTH:
                wall_front = h_walls[mouse_y][mouse_x];
                wall_left = v_walls[mouse_y][mouse_x + 1];
                wall_right = v_walls[mouse_y][mouse_x];
                wall_back = h_walls[mouse_y + 1][mouse_x];
                break;
            case WEST:
                wall_front = v_walls[mouse_y][mouse_x];
                wall_left = h_walls[mouse_y][mouse_x];
                wall_right = h_walls[mouse_y + 1][mouse_x];
                wall_back = v_walls[mouse_y][mouse_x + 1];
                break;
        }
    }

    int fx = mouse_x, fy = mouse_y;
    if (mouse_dir == NORTH) fy++;
    else if (mouse_dir == EAST) fx++;
    else if (mouse_dir == SOUTH) fy--;
    else if (mouse_dir == WEST) fx--;
    if (fx >= 0 && fx < MAZE_SIZE && fy >= 0 && fy < MAZE_SIZE && !wall_front) {
        uint8_t cand = use_dist[fy][fx];
        bool cand_unvisited_start = !visited_to_start[fy][fx];
        bool cand_unexplored_global = !(visited_to_goal[fy][fx] || visited_to_start[fy][fx]);
        if (cand < min_dist && (current_phase != SPEED_TO_GOAL || (!visited_speed[fy][fx] && !backtrack_cells[fy][fx] && (!revisited_cells[fy][fx] || is_junction(fy, fx))))) {
            min_dist = cand; best_dir = 0; found = true;
            best_unvisited_start = cand_unvisited_start; best_unexplored_global = cand_unexplored_global;
        } else if (cand == min_dist && found && current_phase == EXPLORE_TO_START) {
            if (cand_unexplored_global > best_unexplored_global ||
               (cand_unexplored_global == best_unexplored_global && cand_unvisited_start > best_unvisited_start)) {
                best_dir = 0; best_unvisited_start = cand_unvisited_start; best_unexplored_global = cand_unexplored_global;
            }
        }
    }

    int lx = mouse_x, ly = mouse_y;
    if (mouse_dir == NORTH) lx--;
    else if (mouse_dir == EAST) ly++;
    else if (mouse_dir == SOUTH) lx++;
    else if (mouse_dir == WEST) ly--;
    if (lx >= 0 && lx < MAZE_SIZE && ly >= 0 && ly < MAZE_SIZE && !wall_left) {
        uint8_t cand = use_dist[ly][lx];
        bool cand_unvisited_start = !visited_to_start[ly][lx];
        bool cand_unexplored_global = !(visited_to_goal[ly][lx] || visited_to_start[ly][lx]);
        if (cand < min_dist && (current_phase != SPEED_TO_GOAL || (!visited_speed[ly][lx] && !backtrack_cells[ly][lx] && (!revisited_cells[ly][lx] || is_junction(ly, lx))))) {
            min_dist = cand; best_dir = -1; found = true;
            best_unvisited_start = cand_unvisited_start; best_unexplored_global = cand_unexplored_global;
        } else if (cand == min_dist && found && current_phase == EXPLORE_TO_START) {
            if (cand_unexplored_global > best_unexplored_global ||
               (cand_unexplored_global == best_unexplored_global && cand_unvisited_start > best_unvisited_start)) {
                best_dir = -1; best_unvisited_start = cand_unvisited_start; best_unexplored_global = cand_unexplored_global;
            }
        }
    }

    int rx = mouse_x, ry = mouse_y;
    if (mouse_dir == NORTH) rx++;
    else if (mouse_dir == EAST) ry--;
    else if (mouse_dir == SOUTH) rx--;
    else if (mouse_dir == WEST) ry++;
    if (rx >= 0 && rx < MAZE_SIZE && ry >= 0 && ry < MAZE_SIZE && !wall_right) {
        uint8_t cand = use_dist[ry][rx];
        bool cand_unvisited_start = !visited_to_start[ry][rx];
        bool cand_unexplored_global = !(visited_to_goal[ry][rx] || visited_to_start[ry][rx]);
        if (cand < min_dist && (current_phase != SPEED_TO_GOAL || (!visited_speed[ry][rx] && !backtrack_cells[ry][rx] && (!revisited_cells[ry][rx] || is_junction(ry, rx))))) {
            min_dist = cand; best_dir = 1; found = true;
            best_unvisited_start = cand_unvisited_start; best_unexplored_global = cand_unexplored_global;
        } else if (cand == min_dist && found && current_phase == EXPLORE_TO_START) {
            if (cand_unexplored_global > best_unexplored_global ||
               (cand_unexplored_global == best_unexplored_global && cand_unvisited_start > best_unvisited_start)) {
                best_dir = 1; best_unvisited_start = cand_unvisited_start; best_unexplored_global = cand_unexplored_global;
            }
        }
    }

    int bx = mouse_x, by = mouse_y;
    if (mouse_dir == NORTH) by--;
    else if (mouse_dir == EAST) bx--;
    else if (mouse_dir == SOUTH) by++;
    else if (mouse_dir == WEST) bx++;
    if (bx >= 0 && bx < MAZE_SIZE && by >= 0 && by < MAZE_SIZE && !wall_back) {
        uint8_t cand = use_dist[by][bx];
        bool cand_unvisited_start = !visited_to_start[by][bx];
        bool cand_unexplored_global = !(visited_to_goal[by][bx] || visited_to_start[by][bx]);
        if (cand < min_dist && (current_phase != SPEED_TO_GOAL || (!visited_speed[by][bx] && !backtrack_cells[by][bx] && (!revisited_cells[by][bx] || is_junction(by, bx))))) {
            min_dist = cand; best_dir = 2; found = true;
            best_unvisited_start = cand_unvisited_start; best_unexplored_global = cand_unexplored_global;
        } else if (cand == min_dist && found && current_phase == EXPLORE_TO_START) {
            if (cand_unexplored_global > best_unexplored_global ||
               (cand_unexplored_global == best_unexplored_global && cand_unvisited_start > best_unvisited_start)) {
                best_dir = 2; best_unvisited_start = cand_unvisited_start; best_unexplored_global = cand_unexplored_global;
            }
        }
    }

    if (current_phase == SPEED_TO_GOAL && !found) {
        min_dist = MAX_DIST;
        fx = mouse_x; fy = mouse_y;
        if (mouse_dir == NORTH) fy++; else if (mouse_dir == EAST) fx++; else if (mouse_dir == SOUTH) fy--; else if (mouse_dir == WEST) fx--;
    if (fx >= 0 && fx < MAZE_SIZE && fy >= 0 && fy < MAZE_SIZE && !wall_front && use_dist[fy][fx] < min_dist) {
            min_dist = use_dist[fy][fx]; best_dir = 0; found = true;
        }
        lx = mouse_x; ly = mouse_y;
        if (mouse_dir == NORTH) lx--; else if (mouse_dir == EAST) ly++; else if (mouse_dir == SOUTH) lx++; else if (mouse_dir == WEST) ly--;
    if (lx >= 0 && lx < MAZE_SIZE && ly >= 0 && ly < MAZE_SIZE && !wall_left && use_dist[ly][lx] < min_dist) {
            min_dist = use_dist[ly][lx]; best_dir = -1; found = true;
        }
        rx = mouse_x; ry = mouse_y;
        if (mouse_dir == NORTH) rx++; else if (mouse_dir == EAST) ry--; else if (mouse_dir == SOUTH) rx--; else if (mouse_dir == WEST) ry++;
    if (rx >= 0 && rx < MAZE_SIZE && ry >= 0 && ry < MAZE_SIZE && !wall_right && use_dist[ry][rx] < min_dist) {
            min_dist = use_dist[ry][rx]; best_dir = 1; found = true;
        }
        bx = mouse_x; by = mouse_y;
        if (mouse_dir == NORTH) by--; else if (mouse_dir == EAST) bx--; else if (mouse_dir == SOUTH) by++; else if (mouse_dir == WEST) bx++;
    if (bx >= 0 && bx < MAZE_SIZE && by >= 0 && by < MAZE_SIZE && !wall_back && use_dist[by][bx] < min_dist) {
            min_dist = use_dist[by][bx]; best_dir = 2; found = true;
        }
    }

    return best_dir;
}

// Show combined-distance overlay during speed run
void show_dist() {
    for (int y = 0; y < MAZE_SIZE; y++) {
        for (int x = 0; x < MAZE_SIZE; x++) {
            if (dist[y][x] != MAX_DIST) {
                char text[4];
                snprintf(text, sizeof(text), "%d", dist[y][x]);
                API_setText(x, y, text);
            } else {
                API_clearText(x, y);
            }
        }
    }
}

// Dead-end detection algorithm: iteratively find cells (excluding start (0,0) and 4 goal center cells)
// that currently have exactly one open neighbor (i.e., 3 surrounding walls). Instead of sealing,
// we mark them in dead_end_cells. We simulate sealing by treating newly marked dead-ends as closed
// for the purpose of propagating further dead-end detection in the same pass.
void checkDeadEnd() {
    // First clear previous marks (could optimize with dirty tracking)
    for (int y = 0; y < MAZE_SIZE; y++)
        for (int x = 0; x < MAZE_SIZE; x++)
            dead_end_cells[y][x] = false;

    bool changed = true;
    bool considered[MAZE_SIZE][MAZE_SIZE] = {false};
    while (changed) {
        changed = false;
        for (int y = 0; y < MAZE_SIZE; y++) {
            for (int x = 0; x < MAZE_SIZE; x++) {
                if ((x == 0 && y == 0) || ((x == 7 || x == 8) && (y == 7 || y == 8))) continue;
                if (considered[y][x]) continue; // already finalized as not a dead-end in previous iteration
                // Skip cells not yet visited in any exploration phase to avoid speculative pruning
                if (!(visited_to_goal[y][x] || visited_to_start[y][x])) continue;

                int open_dirs = 0;
                // Count open neighbors treating already-marked dead ends as closed
                // North
                if (y < MAZE_SIZE - 1 && !h_walls[y + 1][x] && !dead_end_cells[y + 1][x]) open_dirs++;
                // East
                if (x < MAZE_SIZE - 1 && !v_walls[y][x + 1] && !dead_end_cells[y][x + 1]) open_dirs++;
                // South
                if (y > 0 && !h_walls[y][x] && !dead_end_cells[y - 1][x]) open_dirs++;
                // West
                if (x > 0 && !v_walls[y][x] && !dead_end_cells[y][x - 1]) open_dirs++;

                if (open_dirs <= 1) { // treat 0 or 1 as dead-end (0 could happen after neighbors marked)
                    if (!dead_end_cells[y][x]) {
                        dead_end_cells[y][x] = true;
                        changed = true;
                    }
                } else {
                    considered[y][x] = true; // stable non-dead-end
                }
            }
        }
    }
}

// Floodfill variant for speed run: treat any cell that is unvisited (in both phases) or marked as dead-end as blocked.
void floodfill_speed_run() {
    queue_head = queue_tail = 0;
    for (int y = 0; y < MAZE_SIZE; y++)
        for (int x = 0; x < MAZE_SIZE; x++)
            dist[y][x] = MAX_DIST;

    // Seed goal center
    enqueue(7, 7); dist[7][7] = 0;
    enqueue(7, 8); dist[7][8] = 0;
    enqueue(8, 7); dist[8][7] = 0;
    enqueue(8, 8); dist[8][8] = 0;

    int cy, cx;
    while (dequeue(&cy, &cx)) {
        uint8_t d = dist[cy][cx] + 1;
        // Skip propagation into invalid cells
        // North
        if (cy < MAZE_SIZE - 1 && !h_walls[cy + 1][cx] && dist[cy + 1][cx] > d) {
            int ny = cy + 1, nx = cx;
            if ((visited_to_goal[ny][nx] || visited_to_start[ny][nx]) && !dead_end_cells[ny][nx]) { dist[ny][nx] = d; enqueue(ny, nx);}    
        }
        // East
        if (cx < MAZE_SIZE - 1 && !v_walls[cy][cx + 1] && dist[cy][cx + 1] > d) {
            int ny = cy, nx = cx + 1;
            if ((visited_to_goal[ny][nx] || visited_to_start[ny][nx]) && !dead_end_cells[ny][nx]) { dist[ny][nx] = d; enqueue(ny, nx);}    
        }
        // South
        if (cy > 0 && !h_walls[cy][cx] && dist[cy - 1][cx] > d) {
            int ny = cy - 1, nx = cx;
            if ((visited_to_goal[ny][nx] || visited_to_start[ny][nx]) && !dead_end_cells[ny][nx]) { dist[ny][nx] = d; enqueue(ny, nx);}    
        }
        // West
        if (cx > 0 && !v_walls[cy][cx] && dist[cy][cx - 1] > d) {
            int ny = cy, nx = cx - 1;
            if ((visited_to_goal[ny][nx] || visited_to_start[ny][nx]) && !dead_end_cells[ny][nx]) { dist[ny][nx] = d; enqueue(ny, nx);}    
        }
    }
}