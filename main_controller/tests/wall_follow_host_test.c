/* Host check of the wall follower's error sign and the reverse gain split. */
#include <stdio.h>
#include <math.h>
#include "control_config.h"
static int fails=0;
#define CHECK(c,m) do{ if(!(c)){ printf("FAIL: %s\n",m); fails++; } }while(0)

/* Mirrors the arithmetic in WallFollow_Update(). */
static float err_both(float L,float R){
    return ((L-R)-(WALL_FOLLOW_SETPOINT_LEFT_MM-WALL_FOLLOW_SETPOINT_RIGHT_MM))*0.5f; }
static float err_left(float L){ return L-WALL_FOLLOW_SETPOINT_LEFT_MM; }
static float err_right(float R){ return -(R-WALL_FOLLOW_SETPOINT_RIGHT_MM); }
static float tilt(float e,float dir){
    float kp  = dir<0? WALL_FOLLOW_KP_REVERSE_DEG_PER_MM : WALL_FOLLOW_KP_DEG_PER_MM;
    float cap = dir<0? WALL_FOLLOW_MAX_TILT_REVERSE_DEG  : WALL_FOLLOW_MAX_TILT_DEG;
    float t = dir*kp*e;
    if (t >  cap) t =  cap;
    if (t < -cap) t = -cap;
    return t;
}

int main(void){
  /* positive error must always mean "the robot needs to move LEFT" */
  CHECK(err_both(80,50)>0, "both: closer to the right wall -> move left");
  CHECK(err_both(50,80)<0, "both: closer to the left wall  -> move right");
  CHECK(fabsf(err_both(62,62)-0.5f)<0.01f, "both: equal readings -> ~the sensor trim");
  CHECK(err_left(80)>0,  "left only: too far from left  -> move left");
  CHECK(err_left(40)<0,  "left only: too close to left  -> move right");
  CHECK(err_right(80)<0, "right only: too far from right -> move right");
  CHECK(err_right(40)>0, "right only: too close to right -> move left");

  /* the difference must be immune to a common-mode sensor bias */
  CHECK(fabsf(err_both(58,64)-err_both(58+27,64+27))<0.001f,
        "both: a common +27mm over-read cancels exactly");
  CHECK(fabsf(err_left(58)-err_left(58+27))>20.0f,
        "left only: the same bias does NOT cancel");

  /* forwards tilts toward the correction, backwards tilts away from it */
  float e = err_left(40);                 /* too close to the left */
  CHECK(tilt(e,+1.0f)<0, "forwards: nose right to leave the left wall");
  CHECK(tilt(e,-1.0f)>0, "backwards: nose left to leave the left wall");
  CHECK(fabsf(tilt(e,-1.0f))<fabsf(tilt(e,+1.0f)),
        "reverse demand is smaller than forward for the same error");
  CHECK(WALL_FOLLOW_MAX_TILT_REVERSE_DEG < WALL_FOLLOW_MAX_TILT_DEG,
        "reverse clamp is tighter");
  CHECK(WALL_FOLLOW_MAX_TILT_DEG <= 60.0f/8.0f,
        "tilt clamp stays inside the heading loop's linear range");

  printf("%s (%d failures)\n", fails?"FAILED":"ALL CHECKS PASSED", fails);
  return fails!=0; }
