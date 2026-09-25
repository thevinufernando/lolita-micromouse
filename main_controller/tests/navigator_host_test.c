/* Host check of the reactive navigator.
 *
 * Two things are verified: the decision rule's truth table, and the rule
 * driving the REAL maze_map.c around a simulated arena, so the pose helpers
 * and the wall writes are exercised together rather than in isolation.
 *
 * THIS TEST ALREADY EARNED ITS PLACE. The first version of the rule treated
 * "turn right" as a complete action, and this test showed the robot reaching
 * the opening, turning into it, immediately seeing another open right, and
 * pivoting back down the corridor it came from -- never entering the new cell.
 * Every action must end in one cell of forward motion. See navigator.h.
 *
 *   gcc -O1 -Wall -Wextra -o nav_test tests/navigator_host_test.c \
 *       Core/Src/Maze/maze_map.c -I Core/Inc/Maze && ./nav_test
 *
 * Exit code 0 = pass.
 *
 * The rule is duplicated below rather than linked, because decide() is static
 * in navigator.c and that file cannot build off-target (it needs the HAL, the
 * motor driver and the ToF stack). Keep the two in step: the constants come
 * from the real navigator.h, so only the four comparisons are copied.  */
#include <stdio.h>
#include <string.h>
#include "maze_map.h"

#include "navigator.h"

typedef struct { unsigned char front, left, right; } WallReading_t;

/* Verbatim copy of Maze_Decide() from test_harness.c. */
static unsigned char Maze_Decide(const WallReading_t *w)
{
#if NAV_HAND_RIGHT
  if (!w->right) return NAV_ACT_RIGHT;
  if (!w->front) return NAV_ACT_FORWARD;
  if (!w->left)  return NAV_ACT_LEFT;
#else
  if (!w->left)  return NAV_ACT_LEFT;
  if (!w->front) return NAV_ACT_FORWARD;
  if (!w->right) return NAV_ACT_RIGHT;
#endif
  return NAV_ACT_AROUND;
}

static int fails = 0;
#define CHECK(c,m) do{ if(!(c)){ printf("FAIL: %s\n", m); fails++; } }while(0)

/* --- the user's arena, as truth for the simulated sensors ---------------
 * Corridor north from (0,0) to (0,2), walled east and west; wall across the
 * top of (0,2); passage east to (1,2) walled north and south; (1,2) is a
 * dead end (wall on its east side).                                      */
static int truth_v[4][5];   /* [y][x] west side of (x,y) */
static int truth_h[5][4];   /* [y][x] south side of (x,y) */

static void buildArena(void)
{
  memset(truth_v,0,sizeof truth_v); memset(truth_h,0,sizeof truth_h);
  for (int y=0;y<3;y++){ truth_v[y][0]=1; truth_v[y][1]=1; }  /* corridor sides */
  truth_v[2][1]=0;                                            /* opening east from (0,2) */
  truth_h[0][0]=1;                                            /* south of start */
  truth_h[3][0]=1;                                            /* north of (0,2) */
  truth_h[2][1]=1; truth_h[3][1]=1;                           /* (1,2) south+north */
  truth_v[2][2]=1;                                            /* east of (1,2): dead end */
}

static int wallAbs(int x,int y,int dir)
{
  switch(dir){
    case NORTH: return truth_h[y+1][x];
    case EAST:  return truth_v[y][x+1];
    case SOUTH: return truth_h[y][x];
    default:    return truth_v[y][x];
  }
}
static void sense(WallReading_t *w)
{
  w->front = wallAbs(mouse_x,mouse_y,mouse_dir);
  w->left  = wallAbs(mouse_x,mouse_y,(mouse_dir+3)%4);
  w->right = wallAbs(mouse_x,mouse_y,(mouse_dir+1)%4);
}

int main(void)
{
  /* --- rule table --------------------------------------------------- */
  WallReading_t a={0,0,0}; CHECK(Maze_Decide(&a)==NAV_ACT_RIGHT, "open cell -> right");
  WallReading_t b={0,0,1}; CHECK(Maze_Decide(&b)==NAV_ACT_FORWARD,"right walled -> forward");
  WallReading_t c={1,0,1}; CHECK(Maze_Decide(&c)==NAV_ACT_LEFT,   "front+right walled -> left");
  WallReading_t d={1,1,1}; CHECK(Maze_Decide(&d)==NAV_ACT_AROUND, "dead end -> around");
  WallReading_t e={1,1,0}; CHECK(Maze_Decide(&e)==NAV_ACT_RIGHT,  "only right open -> right");

  /* --- drive the arena ----------------------------------------------- */
  buildArena();
  MazeMap_Init();
  printf("\nstep  pose  dir  F L R  action\n");
  const char *AN[]={"none","FWD","LEFT","RIGHT","AROUND"};
  const char *DN[]={"N","E","S","W"};
  int reached_1_2 = 0;
  for (int step=0; step<14; step++){
    WallReading_t w; sense(&w);
    MazeMap_UpdateWalls(w.front,w.left,w.right);
    if (mouse_x==1 && mouse_y==2) reached_1_2 = 1;
    unsigned char act = Maze_Decide(&w);
    printf("%3d  (%d,%d)  %s   %d %d %d  %s\n",
           step, mouse_x, mouse_y, DN[mouse_dir], w.front,w.left,w.right, AN[act]);
    /* Every action turns (maybe) and THEN advances one cell. */
    if (act==NAV_ACT_RIGHT){ MazeMap_TurnRight(); }
    else if (act==NAV_ACT_LEFT){ MazeMap_TurnLeft(); }
    else if (act==NAV_ACT_AROUND){ MazeMap_TurnRight(); MazeMap_TurnRight(); }
    MazeMap_Advance();
  }
  CHECK(reached_1_2, "right-hand rule reaches (1,2)");

  /* the map must agree with the arena wherever it was surveyed */
  CHECK(MazeMap_WallNorth(0,2)==1, "wall north of (0,2) recorded");
  CHECK(MazeMap_WallEast(0,2)==0,  "opening east of (0,2) left clear");
  CHECK(MazeMap_WallEast(0,0)==1,  "wall east of (0,0) recorded");
  CHECK(MazeMap_WallEast(1,2)==1,  "dead end east of (1,2) recorded");

  printf("\n%s (%d failures)\n", fails? "FAILED":"ALL CHECKS PASSED", fails);
  return fails!=0;
}
