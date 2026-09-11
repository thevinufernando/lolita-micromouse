#include "maze_map.h"

uint8_t v_walls[MAZE_SIZE][MAZE_SIZE + 1];
uint8_t h_walls[MAZE_SIZE + 1][MAZE_SIZE];

int16_t   mouse_x;
int16_t   mouse_y;
Direction mouse_dir;


void MazeMap_Init(void)
{
    for (int16_t y = 0; y < MAZE_SIZE; y++) {
        for (int16_t x = 0; x < MAZE_SIZE + 1; x++) {
            v_walls[y][x] = 0U;
        }
    }

    for (int16_t y = 0; y < MAZE_SIZE + 1; y++) {
        for (int16_t x = 0; x < MAZE_SIZE; x++) {
            h_walls[y][x] = 0U;
        }
    }

    /* The outer boundary is always walled. Setting it up front means the
     * pose helpers cannot walk the mouse outside the arrays even if a move
     * is mis-reported, and it matches what the simulator assumes. */
    for (int16_t y = 0; y < MAZE_SIZE; y++) {
        v_walls[y][0] = 1U;           /* west edge  */
        v_walls[y][MAZE_SIZE] = 1U;   /* east edge  */
    }
    for (int16_t x = 0; x < MAZE_SIZE; x++) {
        h_walls[0][x] = 1U;           /* south edge */
        h_walls[MAZE_SIZE][x] = 1U;   /* north edge */
    }

    mouse_x = 0;
    mouse_y = 0;
    mouse_dir = NORTH;
}


/* Direct port of updateWalls() from MicroMouseAlgorithm/maze.c. The nested
 * conditionals are kept in the same shape on purpose: this is the one place
 * where firmware and simulator must agree exactly, and a "tidier" rewrite is
 * a silent-divergence risk for no benefit. */
void MazeMap_UpdateWalls(uint8_t front, uint8_t left, uint8_t right)
{
    if (front) {
        if (mouse_dir == NORTH)      h_walls[mouse_y + 1][mouse_x] = 1U;
        else if (mouse_dir == EAST)  v_walls[mouse_y][mouse_x + 1] = 1U;
        else if (mouse_dir == SOUTH) h_walls[mouse_y][mouse_x]     = 1U;
        else                         v_walls[mouse_y][mouse_x]     = 1U;
    }

    if (left) {
        if (mouse_dir == NORTH)      v_walls[mouse_y][mouse_x]     = 1U;
        else if (mouse_dir == EAST)  h_walls[mouse_y + 1][mouse_x] = 1U;
        else if (mouse_dir == SOUTH) v_walls[mouse_y][mouse_x + 1] = 1U;
        else                         h_walls[mouse_y][mouse_x]     = 1U;
    }

    if (right) {
        if (mouse_dir == NORTH)      v_walls[mouse_y][mouse_x + 1] = 1U;
        else if (mouse_dir == EAST)  h_walls[mouse_y][mouse_x]     = 1U;
        else if (mouse_dir == SOUTH) v_walls[mouse_y][mouse_x]     = 1U;
        else                         h_walls[mouse_y + 1][mouse_x] = 1U;
    }
}


void MazeMap_SetPose(int16_t x, int16_t y, Direction dir)
{
    if (x >= 0 && x < MAZE_SIZE) mouse_x = x;
    if (y >= 0 && y < MAZE_SIZE) mouse_y = y;

    mouse_dir = dir;
}


uint8_t MazeMap_Retreat(void)
{
    /* Face the other way, step, face back. Reusing Advance() rather than
     * repeating the bounds logic means the two can never disagree about where
     * the edge of the maze is. */
    MazeMap_TurnRight();
    MazeMap_TurnRight();

    uint8_t moved = MazeMap_Advance();

    MazeMap_TurnRight();
    MazeMap_TurnRight();

    return moved;
}


uint8_t MazeMap_Advance(void)
{
    switch (mouse_dir) {
        case NORTH: if (mouse_y >= MAZE_SIZE - 1) return 0U; mouse_y++; break;
        case EAST:  if (mouse_x >= MAZE_SIZE - 1) return 0U; mouse_x++; break;
        case SOUTH: if (mouse_y <= 0)             return 0U; mouse_y--; break;
        case WEST:  if (mouse_x <= 0)             return 0U; mouse_x--; break;
        default: return 0U;
    }
    return 1U;
}


void MazeMap_TurnLeft(void)
{
    mouse_dir = (Direction)((mouse_dir + 3) % 4);
}


void MazeMap_TurnRight(void)
{
    mouse_dir = (Direction)((mouse_dir + 1) % 4);
}


static uint8_t inBounds(int16_t x, int16_t y)
{
    return (x >= 0 && x < MAZE_SIZE && y >= 0 && y < MAZE_SIZE) ? 1U : 0U;
}

uint8_t MazeMap_WallNorth(int16_t x, int16_t y)
{ return inBounds(x, y) ? h_walls[y + 1][x] : 1U; }

uint8_t MazeMap_WallEast(int16_t x, int16_t y)
{ return inBounds(x, y) ? v_walls[y][x + 1] : 1U; }

uint8_t MazeMap_WallSouth(int16_t x, int16_t y)
{ return inBounds(x, y) ? h_walls[y][x] : 1U; }

uint8_t MazeMap_WallWest(int16_t x, int16_t y)
{ return inBounds(x, y) ? v_walls[y][x] : 1U; }
