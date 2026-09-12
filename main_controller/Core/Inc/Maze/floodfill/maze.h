#ifndef MAZE_H
#define MAZE_H

#include <stdbool.h>
#include <stdint.h>

// Maze constants
#define MAZE_SIZE 16
#define MAX_DIST 255
#define QUEUE_SIZE (MAZE_SIZE * MAZE_SIZE + 10)

// Single distance map (overwritten per phase: to goal during EXPLORE_TO_GOAL & SPEED_TO_GOAL, to start during EXPLORE_TO_START)
extern uint8_t dist[MAZE_SIZE][MAZE_SIZE];
// Dead-end tracking (computed during exploration; used to mask during speed run)
extern bool dead_end_cells[MAZE_SIZE][MAZE_SIZE];

// Directions
typedef enum { NORTH, EAST, SOUTH, WEST } Direction;

// Phases
typedef enum { EXPLORE_TO_GOAL, EXPLORE_TO_START, SPEED_TO_GOAL } Phase;

// Function declarations
void debug_log(char* text);
bool is_junction(int y, int x);
void floodfill_phase(Phase phase);
void enqueue(int y, int x);
bool dequeue(int *y, int *x);
void initWalls();
void updateWalls();
void floodfill_to_goal();
int getBestDirection();
void show_dist();
// Dead-end filling (iteratively close corridors with only one exit, excluding start & goal cells)
void checkDeadEnd(); // now only marks dead_end_cells instead of modifying walls
// Speed run floodfill (ignores unvisited and dead-end cells)
void floodfill_speed_run();

#endif // MAZE_H