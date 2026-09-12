#include "maze_map.h"

/* v_walls, h_walls and the pose are DEFINED BY THE ALGORITHM, in
 * floodfill_run.c, because that is what updates them after every move. They
 * used to be defined here under the same names, which was harmless only while
 * the two never met in one binary. */

/* Cells whose walls have actually been read, one bit each.
 *
 * WITHOUT THIS THE MAP CANNOT SAY "I DO NOT KNOW". MazeMap_UpdateWalls() only
 * ever SETS a wall to 1, so a zero means "no wall has been seen here" -- which
 * is indistinguishable from "this side is open". That is harmless while the
 * map is only ever asked about cells the robot has stood in, and wrong the
 * moment anything reasons about a cell ahead: every unexplored cell would read
 * as wide open on all four sides.
 *
 * 16x16 bits is 32 bytes, which is cheaper than the confusion. */
static uint8_t s_known[(MAZE_SIZE * MAZE_SIZE + 7) / 8];

/* Defined further down, with the wall accessors it was written for. */
static uint8_t inBounds(int16_t x, int16_t y);



void MazeMap_Init(void)
{
    for (int16_t y = 0; y < MAZE_SIZE; y++) {
        for (int16_t x = 0; x < MAZE_SIZE + 1; x++) {
            v_walls[y][x] = false;
        }
    }

    for (int16_t y = 0; y < MAZE_SIZE + 1; y++) {
        for (int16_t x = 0; x < MAZE_SIZE; x++) {
            h_walls[y][x] = false;
        }
    }

    /* The outer boundary is always walled. Setting it up front means the
     * pose helpers cannot walk the mouse outside the arrays even if a move
     * is mis-reported, and it matches what the simulator assumes. */
    for (int16_t y = 0; y < MAZE_SIZE; y++) {
        v_walls[y][0] = true;           /* west edge  */
        v_walls[y][MAZE_SIZE] = true;   /* east edge  */
    }
    for (int16_t x = 0; x < MAZE_SIZE; x++) {
        h_walls[0][x] = true;           /* south edge */
        h_walls[MAZE_SIZE][x] = true;   /* north edge */
    }

    for (uint16_t i = 0; i < sizeof s_known; i++) {
        s_known[i] = 0U;
    }

    mouse_x = 0;
    mouse_y = 0;
    mouse_dir = NORTH;
}


void MazeMap_MarkKnown(int16_t x, int16_t y)
{
    if (!inBounds(x, y)) return;

    const uint16_t bit = (uint16_t)(y * MAZE_SIZE + x);

    s_known[bit >> 3] |= (uint8_t)(1U << (bit & 7U));
}


uint8_t MazeMap_IsKnown(int16_t x, int16_t y)
{
    if (!inBounds(x, y)) return 0U;

    const uint16_t bit = (uint16_t)(y * MAZE_SIZE + x);

    return (s_known[bit >> 3] >> (bit & 7U)) & 1U;
}


/* Direct port of updateWalls() from MicroMouseAlgorithm/maze.c. The nested
 * conditionals are kept in the same shape on purpose: this is the one place
 * where firmware and simulator must agree exactly, and a "tidier" rewrite is
 * a silent-divergence risk for no benefit. */
void MazeMap_UpdateWalls(uint8_t front, uint8_t left, uint8_t right)
{
    /* Marked here rather than on arrival because THIS is the call that means
     * the cell has been looked at. A pose can be set or advanced without any
     * walls being read -- after a failed move, for instance -- and a cell
     * counted as known on that basis would hand out wall data nobody measured.
     */
    MazeMap_MarkKnown((int16_t)mouse_x, (int16_t)mouse_y);

    if (front) {
        if (mouse_dir == NORTH)      h_walls[mouse_y + 1][mouse_x] = true;
        else if (mouse_dir == EAST)  v_walls[mouse_y][mouse_x + 1] = true;
        else if (mouse_dir == SOUTH) h_walls[mouse_y][mouse_x]     = true;
        else                         v_walls[mouse_y][mouse_x]     = true;
    }

    if (left) {
        if (mouse_dir == NORTH)      v_walls[mouse_y][mouse_x]     = true;
        else if (mouse_dir == EAST)  h_walls[mouse_y + 1][mouse_x] = true;
        else if (mouse_dir == SOUTH) v_walls[mouse_y][mouse_x + 1] = true;
        else                         h_walls[mouse_y][mouse_x]     = true;
    }

    if (right) {
        if (mouse_dir == NORTH)      v_walls[mouse_y][mouse_x + 1] = true;
        else if (mouse_dir == EAST)  h_walls[mouse_y][mouse_x]     = true;
        else if (mouse_dir == SOUTH) v_walls[mouse_y][mouse_x]     = true;
        else                         h_walls[mouse_y + 1][mouse_x] = true;
    }
}


void MazeMap_SetPose(int16_t x, int16_t y, Direction dir)
{
    if (x >= 0 && x < MAZE_SIZE) mouse_x = x;
    if (y >= 0 && y < MAZE_SIZE) mouse_y = y;

    mouse_dir = dir;
}


uint8_t MazeMap_NextCell(int16_t x, int16_t y, Direction dir,
                         int16_t *nx, int16_t *ny)
{
    switch (dir) {
        case NORTH: if (y >= MAZE_SIZE - 1) return 0U; y++; break;
        case EAST:  if (x >= MAZE_SIZE - 1) return 0U; x++; break;
        case SOUTH: if (y <= 0)             return 0U; y--; break;
        case WEST:  if (x <= 0)             return 0U; x--; break;
        default: return 0U;
    }

    if (nx) *nx = x;
    if (ny) *ny = y;

    return 1U;
}


uint8_t MazeMap_Advance(void)
{
    /* Through locals, because the pose is the algorithm's `int` and this takes
     * int16_t. Narrowing is safe: every value is a maze index. */
    int16_t nx = 0, ny = 0;

    if (!MazeMap_NextCell((int16_t)mouse_x, (int16_t)mouse_y, mouse_dir,
                          &nx, &ny)) {
        return 0U;
    }

    mouse_x = nx;
    mouse_y = ny;

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


/* The read counterpart of MazeMap_UpdateWalls(), and deliberately its mirror
 * image: same cell, same heading, same three robot-relative answers.
 *
 * It lives here rather than in the navigator because the direction-to-compass
 * arithmetic is the one thing in this module that is easy to get subtly wrong
 * and impossible to notice -- a left/right swap produces a map that is
 * plausible, self-consistent and mirrored. One copy, next to the one that
 * writes it, is the whole point.
 *
 * Outside the maze every side reads as walled, matching the accessors above. */
void MazeMap_CellWalls(int16_t x, int16_t y, Direction dir,
                       uint8_t *front, uint8_t *left, uint8_t *right)
{
    uint8_t n = MazeMap_WallNorth(x, y);
    uint8_t e = MazeMap_WallEast(x, y);
    uint8_t s = MazeMap_WallSouth(x, y);
    uint8_t w = MazeMap_WallWest(x, y);

    uint8_t f_v, l_v, r_v;

    switch (dir) {
        case NORTH: f_v = n; l_v = w; r_v = e; break;
        case EAST:  f_v = e; l_v = n; r_v = s; break;
        case SOUTH: f_v = s; l_v = e; r_v = w; break;
        default:    f_v = w; l_v = s; r_v = n; break;   /* WEST */
    }

    if (front) *front = f_v;
    if (left)  *left  = l_v;
    if (right) *right = r_v;
}
