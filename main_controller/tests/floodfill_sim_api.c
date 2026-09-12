/* A maze and a mouse, standing in for the robot, so the ported flood fill can
 * be compared against the original one decision at a time.
 *
 * THIS FILE IS LINKED INTO TWO BINARIES. One is built from
 * MicroMouseAlgorithm/maze.c and Main.c; the other from the copies under
 * Core/{Inc,Src}/Maze/floodfill/. Everything else about the two builds is
 * identical, so any difference in the transcript they print is a difference
 * the port introduced. That is the entire argument: "the algorithm did not
 * change" is a claim that can be checked rather than asserted.
 *
 * The transcript goes to stdout and carries one line per action -- the move,
 * the pose it was taken from, and the phase. debug_log() goes to stderr in the
 * original and nowhere in the port, so it is deliberately not compared.
 *
 * ---------------------------------------------------------------------------
 * IT ALSO TESTS THE ONE ADAPTATION THE ROBOT NEEDED
 * ---------------------------------------------------------------------------
 * On the robot, API_wallFront/Left/Right are served from a snapshot taken once
 * per cell, because a real read costs about 200 ms and the algorithm asks
 * several times per cell. Run with "cached" and this file does the same. The
 * transcripts must match "fresh" exactly -- if they ever do not, the caching
 * is changing behaviour rather than just saving time. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#define SIM_SIZE 16

/* The true maze. Walls the algorithm has not discovered yet still exist. */
static bool tv[SIM_SIZE][SIM_SIZE + 1];   /* west side of (x,y) */
static bool th[SIM_SIZE + 1][SIM_SIZE];   /* south side of (x,y) */

/* Where the mouse REALLY is, which is the ground truth the algorithm's own
 * belief is checked against implicitly: if they ever diverged, the walls it
 * reported would stop matching the maze and the transcripts would split. */
static int sim_x, sim_y, sim_dir;         /* 0=N 1=E 2=S 3=W */

static int cache_mode;                    /* serve walls from a per-cell snapshot */
static int snap_valid, snap_f, snap_l, snap_r;

static unsigned long rng_state;

static unsigned rnd(unsigned n)
{
    rng_state = rng_state * 6364136223846793005ULL + 1442695040888963407ULL;
    return (unsigned)((rng_state >> 33) % n);
}

/* Recursive backtracker, which gives a perfect maze -- every cell reachable,
 * exactly one path between any two, and plenty of dead ends for checkDeadEnd()
 * to find. Seeded, so both binaries carve the identical maze. */
static void carve(void)
{
    bool seen[SIM_SIZE][SIM_SIZE];
    int stack[SIM_SIZE * SIM_SIZE][2];
    int top = 0;

    for (int y = 0; y < SIM_SIZE; y++)
        for (int x = 0; x < SIM_SIZE + 1; x++) tv[y][x] = true;
    for (int y = 0; y < SIM_SIZE + 1; y++)
        for (int x = 0; x < SIM_SIZE; x++) th[y][x] = true;
    for (int y = 0; y < SIM_SIZE; y++)
        for (int x = 0; x < SIM_SIZE; x++) seen[y][x] = false;

    stack[top][0] = 0; stack[top][1] = 0; top++;
    seen[0][0] = true;

    while (top > 0) {
        int cy = stack[top - 1][0], cx = stack[top - 1][1];
        int cand[4][3], n = 0;

        if (cy + 1 < SIM_SIZE && !seen[cy + 1][cx]) { cand[n][0]=cy+1; cand[n][1]=cx; cand[n][2]=0; n++; }
        if (cx + 1 < SIM_SIZE && !seen[cy][cx + 1]) { cand[n][0]=cy; cand[n][1]=cx+1; cand[n][2]=1; n++; }
        if (cy - 1 >= 0       && !seen[cy - 1][cx]) { cand[n][0]=cy-1; cand[n][1]=cx; cand[n][2]=2; n++; }
        if (cx - 1 >= 0       && !seen[cy][cx - 1]) { cand[n][0]=cy; cand[n][1]=cx-1; cand[n][2]=3; n++; }

        if (n == 0) { top--; continue; }

        int k = (int)rnd((unsigned)n);
        int ny = cand[k][0], nx = cand[k][1];

        switch (cand[k][2]) {
            case 0: th[cy + 1][cx] = false; break;
            case 1: tv[cy][cx + 1] = false; break;
            case 2: th[cy][cx]     = false; break;
            default: tv[cy][cx]    = false; break;
        }

        seen[ny][nx] = true;
        stack[top][0] = ny; stack[top][1] = nx; top++;
    }

    /* Open the four centre cells into one chamber, as a real maze does. */
    tv[7][8] = false;  tv[8][8] = false;
    th[8][7] = false;  th[8][8] = false;
}

/* Carved BEFORE main() runs, from the environment, because the original's
 * Main.c owns main() and ignores its arguments. A constructor is the only hook
 * that fires early enough without editing it, and using the same one for both
 * builds means the two binaries are driven identically -- which is the whole
 * point of the exercise. */
__attribute__((constructor))
static void simSetup(void)
{
    const char *s = getenv("FF_SEED");
    const char *c = getenv("FF_CACHED");

    rng_state = (s ? strtoul(s, 0, 10) : 1UL) * 2654435761UL + 12345UL;
    carve();
    sim_x = 0; sim_y = 0; sim_dir = 0;
    cache_mode = c ? atoi(c) : 0;
    snap_valid = 0;
}

/* Is there a wall on the given side of the mouse's true cell? */
static int wallAt(int rel)   /* 0 = front, 3 = left, 1 = right */
{
    int d = (sim_dir + rel) & 3;

    switch (d) {
        case 0:  return th[sim_y + 1][sim_x] ? 1 : 0;
        case 1:  return tv[sim_y][sim_x + 1] ? 1 : 0;
        case 2:  return th[sim_y][sim_x]     ? 1 : 0;
        default: return tv[sim_y][sim_x]     ? 1 : 0;
    }
}

static void snapshot(void)
{
    if (!cache_mode || snap_valid) return;

    snap_f = wallAt(0);
    snap_r = wallAt(1);
    snap_l = wallAt(3);
    snap_valid = 1;
}

int API_mazeWidth(void)  { return SIM_SIZE; }
int API_mazeHeight(void) { return SIM_SIZE; }

int API_wallFront(void) { snapshot(); return cache_mode ? snap_f : wallAt(0); }
int API_wallRight(void) { snapshot(); return cache_mode ? snap_r : wallAt(1); }
int API_wallLeft(void)  { snapshot(); return cache_mode ? snap_l : wallAt(3); }

int API_moveForward(void)
{
    printf("F %d %d %d\n", sim_x, sim_y, sim_dir);

    if (wallAt(0)) { printf("CRASH\n"); return 0; }

    if      (sim_dir == 0) sim_y++;
    else if (sim_dir == 1) sim_x++;
    else if (sim_dir == 2) sim_y--;
    else                   sim_x--;

    snap_valid = 0;
    return 1;
}

void API_turnLeft(void)
{
    printf("L %d %d %d\n", sim_x, sim_y, sim_dir);
    sim_dir = (sim_dir + 3) & 3;
    snap_valid = 0;
}

void API_turnRight(void)
{
    printf("R %d %d %d\n", sim_x, sim_y, sim_dir);
    sim_dir = (sim_dir + 1) & 3;
    snap_valid = 0;
}

/* Drawing surface: no output, or the transcript would be dominated by it. */
void API_setWall(int x, int y, char d)   { (void)x; (void)y; (void)d; }
void API_clearWall(int x, int y, char d) { (void)x; (void)y; (void)d; }
void API_setColor(int x, int y, char c)  { (void)x; (void)y; (void)c; }
void API_clearColor(int x, int y)        { (void)x; (void)y; }
void API_clearAllColor(void)             { }
void API_setText(int x, int y, char *s)  { (void)x; (void)y; (void)s; }
void API_clearText(int x, int y)         { (void)x; (void)y; }
void API_clearAllText(void)              { }

int  API_wasReset(void) { return 0; }
void API_ackReset(void) { }

#ifdef SIM_PROVIDE_DEBUG_LOG
/* The port moved debug_log's body into mms_api.c, which is robot-only, so the
 * ported build needs one here. The original build does not: its maze.c still
 * carries the fprintf version, and providing a second would collide. */
void debug_log(char *text) { (void)text; }
#endif
