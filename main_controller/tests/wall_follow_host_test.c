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
/* asymmetric: `far` is the ambiguous direction on a single wall */
static float tilt_single(float e,int far){
    float kp  = far? WALL_FOLLOW_SINGLE_FAR_KP : WALL_FOLLOW_KP_DEG_PER_MM;
    float cap = far? WALL_FOLLOW_SINGLE_FAR_TILT_DEG : WALL_FOLLOW_MAX_TILT_DEG;
    float t = kp*e;
    if (t >  cap) t =  cap;
    if (t < -cap) t = -cap;
    return t; }
static int pair_ok(float L,float R){
    return fabsf((L+R)-WALL_FOLLOW_SPAN_MM) <= WALL_FOLLOW_SPAN_TOL_MM; }
static float tilt(float e){
    float t = WALL_FOLLOW_KP_DEG_PER_MM*e;
    if (t >  WALL_FOLLOW_MAX_TILT_DEG) t =  WALL_FOLLOW_MAX_TILT_DEG;
    if (t < -WALL_FOLLOW_MAX_TILT_DEG) t = -WALL_FOLLOW_MAX_TILT_DEG;
    return t;
}

int main(void){
  /* positive error must always mean "the robot needs to move LEFT" */
  CHECK(err_both(80,50)>0, "both: closer to the right wall -> move left");
  CHECK(err_both(50,80)<0, "both: closer to the left wall  -> move right");
  CHECK(fabsf(err_both(62,62)-0.5f)<0.01f, "both: equal readings -> ~the sensor trim");
  /* Single wall corrects only AWAY from a wall that is too close; a long
     reading is ambiguous at a junction and must produce no command. */
  CHECK(err_left(40)<0,  "left only: too close to left -> move right");
  CHECK(err_right(40)>0, "right only: too close to right -> move left");
  /* the far direction still acts, but weakly and tightly capped */
  CHECK(tilt_single(err_left(90),1) > 0.0f, "left only: a long reading still pulls");
  CHECK(fabsf(tilt_single(err_left(90),1)) <= WALL_FOLLOW_SINGLE_FAR_TILT_DEG,
        "far pull is capped");
  CHECK(fabsf(tilt_single(err_left(90),1)) < fabsf(tilt_single(err_left(36),0)),
        "far pull is weaker than the near push for a similar error");

  /* The span test is what separates a same-cell wall from something beyond an
     opening, which no absolute distance threshold can do. */
  CHECK(pair_ok(60,64),  "span: an ordinary pair is accepted");
  CHECK(pair_ok(90,34),  "span: badly off-centre but consistent -> accepted");
  CHECK(!pair_ok(41,108),"span: the (41,108) junction pair is rejected");
  CHECK(!pair_ok(60,239),"span: a wall a cell away is rejected");

  /* the difference must be immune to a common-mode sensor bias */
  CHECK(fabsf(err_both(58,64)-err_both(58+27,64+27))<0.001f,
        "both: a common +27mm over-read cancels exactly");
  CHECK(fabsf(err_left(30)-err_left(30+27))>20.0f,
        "left only: the same bias does NOT cancel");

  /* the tilt always points the way the correction needs to go */
  CHECK(tilt(err_left(40))<0, "too close to the left -> nose right");
  CHECK(tilt(err_right(40))>0, "too close to the right -> nose left");
  CHECK(tilt(err_both(90,34))>0, "closer to the right wall -> nose left");
  CHECK(WALL_FOLLOW_MAX_TILT_DEG <= 60.0f/8.0f,
        "tilt clamp stays inside the heading loop's linear range");

  /* The integral must actually null a standing error the P term only balances.
     Replays the measured case: a constant +8 mm offset with both walls seen. */
  {
    float integ = 0.0f, e = 8.0f;
    const float dt = WALL_FOLLOW_UPDATE_S;
    int n = 0;
    while (n < 2000 && integ < WALL_FOLLOW_KI_LIMIT_DEG) {
      integ += WALL_FOLLOW_KI_DEG_PER_MM_S * e * dt;
      if (integ > WALL_FOLLOW_KI_LIMIT_DEG) integ = WALL_FOLLOW_KI_LIMIT_DEG;
      n++;
      if (integ >= WALL_FOLLOW_KP_DEG_PER_MM * e) break;   /* matched P */
    }
    float secs = n * dt;
    CHECK(integ >= WALL_FOLLOW_KP_DEG_PER_MM * e,
          "integral can supply what the P term was holding");
    CHECK(secs > 2.0f && secs < 15.0f,
          "and takes seconds, not milliseconds, to do it");
  }

  /* THE RECOVERY CEILING. The integral goes to the heading target, not into
     the tilt, so it must be allowed to exceed the tilt clamp -- otherwise a
     yaw estimate that has drifted further than the clamp can never be
     corrected, which is the observed "yawed left, wall in view, no
     correction" failure. This check is the inverse of the one it replaced,
     and deliberately so. */
  CHECK(WALL_FOLLOW_KI_LIMIT_DEG > WALL_FOLLOW_MAX_TILT_DEG,
        "the integral can out-reach the tilt clamp");
  /* The linear-range rule applies to the heading loop's ERROR, not to how far
     the setpoint has moved: a setpoint the robot is tracking produces no
     error at all. So what has to stay small is the integral's RATE, and the
     tracking error it costs. At the largest lateral error the gate admits,
     the integral moves the target this fast: */
  {
    const float worst_mm   = (float)WALL_FOLLOW_USABLE_MAX_MM
                             - WALL_FOLLOW_SETPOINT_LEFT_MM;
    const float ramp_dps   = WALL_FOLLOW_KI_DEG_PER_MM_S * worst_mm;
    /* Turn rate the heading loop buys per degree of error, from the measured
       command-to-rate gain (TURN_FF_GAIN is units per deg/s). */
    const float dps_per_deg = STRAIGHT_YAW_KP / TURN_FF_GAIN;
    const float track_err   = ramp_dps / dps_per_deg;
    CHECK(track_err < 0.5f * (STRAIGHT_YAW_LIMIT / STRAIGHT_YAW_KP),
          "tracking the integral costs far less than the linear range");
  }

  /* With a reference error bigger than the tilt clamp the robot must still be
     able to command a heading that turns it back. Replays the failure: the
     estimate reads REF_ERR degrees low, so the loop has to command +REF_ERR
     just to drive straight. */
  {
    const float ref_err = WALL_FOLLOW_MAX_TILT_DEG + 2.0f;
    float integ = 0.0f;
    const float dt = WALL_FOLLOW_UPDATE_S;
    /* Drifting into the left wall: the reading falls below setpoint, so the
       error is negative -- the unambiguous direction, which integrates. */
    for (int i = 0; i < 4000 && integ < ref_err; i++)
      integ += WALL_FOLLOW_KI_DEG_PER_MM_S * 6.0f * dt;
    CHECK(integ >= ref_err,
          "a reference error past the tilt clamp is still reachable");
    /* The old arrangement, integral inside the clamp, could not: */
    CHECK(WALL_FOLLOW_MAX_TILT_DEG < ref_err,
          "and the tilt clamp alone provably could not reach it");
  }

  printf("%s (%d failures)\n", fails?"FAILED":"ALL CHECKS PASSED", fails);
  return fails!=0; }
