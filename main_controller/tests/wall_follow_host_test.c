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
/* Mirrors far_confidence() in wall_follow.c: 1.0 at or inside the setpoint,
   falling linearly to the floor at the usable gate. */
static float conf_of(float reading,float setpoint){
    float far = reading - setpoint;
    if (far <= 0.0f) return 1.0f;
    float span = (float)WALL_FOLLOW_USABLE_MAX_MM - setpoint;
    float t = far/span;
    if (t > 1.0f) t = 1.0f;
    return 1.0f - t*(1.0f - WALL_FOLLOW_FAR_CONF_FLOOR); }

/* Tilt from a single-wall reading, with gain and clamp both scaled by it. */
static float tilt_single(float e,float conf){
    float kp  = WALL_FOLLOW_KP_DEG_PER_MM * conf;
    float cap = WALL_FOLLOW_MAX_TILT_DEG  * conf;
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
  /* CONFIDENCE IS A RAMP, and its two ends are the two things already known. */
  CHECK(conf_of(63.0f, 63.0f) == 1.0f,
        "a reading at the setpoint is certain");
  CHECK(conf_of(40.0f, 63.0f) == 1.0f,
        "and a reading inside it is too -- only a wall returns close");
  CHECK(fabsf(conf_of((float)WALL_FOLLOW_USABLE_MAX_MM, 63.0f)
              - WALL_FOLLOW_FAR_CONF_FLOOR) < 0.001f,
        "a reading at the usable gate is worth the floor and no less");
  CHECK(conf_of(80.0f, 63.0f) < conf_of(70.0f, 63.0f),
        "and confidence falls monotonically between them");

  /* THE CELL THAT COST A RUN: a right wall reading 76 against a 64 setpoint,
     with the robot 14 mm off. The old binary rule capped this at 2.5 deg. */
  {
    const float c = conf_of(76.0f, WALL_FOLLOW_SETPOINT_RIGHT_MM);
    const float t = tilt_single(-14.0f, c);
    CHECK(fabsf(t) > 4.0f,
          "the 76 mm reading now buys a real correction, not a token one");
    CHECK(fabsf(t) < WALL_FOLLOW_MAX_TILT_DEG,
          "but still less than a two-wall reference would get");
  }

  /* A reading at the edge of usable range must stay timid: that is the case
     the caution was written for, and it has to survive the taper. */
  {
    const float c = conf_of(94.0f, WALL_FOLLOW_SETPOINT_LEFT_MM);
    CHECK(fabsf(tilt_single(31.0f, c)) < 4.0f,
          "a reading at the edge of range still cannot lunge");
    CHECK(c < WALL_FOLLOW_TRUST_CONF,
          "and is not trusted enough for the integral to learn from");
  }

  /* The integral's threshold has to admit the case it exists for. */
  CHECK(conf_of(68.0f, 63.0f) >= WALL_FOLLOW_TRUST_CONF,
        "a few mm long is an off-centre robot, and still teaches the integral");

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
  /* THE CASCADE RULE, in symbols rather than in numbers, so the two constants
     can never drift apart again. The outer loop's output is the inner loop's
     setpoint, so it must stay inside the range the inner loop can answer
     linearly -- past that the cascade is bang-bang. */
  CHECK(WALL_FOLLOW_MAX_TILT_DEG <= STRAIGHT_YAW_LIMIT / STRAIGHT_YAW_KP,
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

  /* THE SLEW COSTS LINEAR RANGE TOO, which is easy to miss because a slew
     limit reads like a safety measure rather than a load. The inner loop buys
     STRAIGHT_YAW_KP / TURN_FF_GAIN degrees per second of turn rate for each
     degree of heading error, so tracking a setpoint that ramps at R deg/s
     costs R/that many degrees of standing error -- and that error is spent
     before the tilt itself asks for anything. */
  {
    const float dps_per_deg = STRAIGHT_YAW_KP / TURN_FF_GAIN;
    const float ramp_cost   = WALL_FOLLOW_TILT_SLEW_DPS / dps_per_deg;
    CHECK(ramp_cost < 0.5f * (STRAIGHT_YAW_LIMIT / STRAIGHT_YAW_KP),
          "slewing the tilt costs less than half the linear range");
  }

  /* THE RECOVERY CEILING. The integral goes to the heading target, not into
     the tilt, so the total authority is the clamp PLUS the integral limit and
     is not bounded by the clamp at all. That is the fix for the observed
     "yawed left, wall in view, no correction" failure, where the clamp alone
     had to answer a demand of 13.5 degrees. */
  CHECK(WALL_FOLLOW_MAX_TILT_DEG + WALL_FOLLOW_KI_LIMIT_DEG > 13.5f,
        "clamp plus integral out-reach the worst demand recorded");
  CHECK(WALL_FOLLOW_KI_LIMIT_DEG > 5.5f,
        "the integral alone covers the standing heading error recorded");
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
    const float ref_err = 5.5f;   /* the standing error the run actually showed */
    float integ = 0.0f;
    const float dt = WALL_FOLLOW_UPDATE_S;
    /* Drifting into the left wall: the reading falls below setpoint, so the
       error is negative -- the unambiguous direction, which integrates. */
    for (int i = 0; i < 4000 && integ < ref_err; i++)
      integ += WALL_FOLLOW_KI_DEG_PER_MM_S * 6.0f * dt;
    CHECK(integ >= ref_err,
          "a reference error past the tilt clamp is still reachable");
    CHECK(integ <= WALL_FOLLOW_KI_LIMIT_DEG,
          "and never past its own limit while doing so");
  }

  /* ANTI-WINDUP. The measured failure: a single-wall cell 23 mm off centre.
     The P term asks for more than the clamp, so the loop is already doing all
     it can, and anything the integrator adds on top is windup rather than a
     bias it has learned. Replays the cell that moved the term 3.3 degrees. */
  {
    const float e   = -23.0f;                       /* as recorded */
    const float p   = WALL_FOLLOW_KP_DEG_PER_MM * e;
    const int   sat = (fabsf(p) > WALL_FOLLOW_MAX_TILT_DEG);

    CHECK(sat, "a 23 mm single-wall error does saturate the tilt clamp");

    /* THE LOWER GAIN ALONE IS NOT ENOUGH, which is the reason the gate is
       there rather than just a smaller number. The wall was in view for about
       1.4 s of that cell and the term moved 3.3 degrees. The reduced gain cuts
       that to roughly 1.3 -- better, still far too much for something meant to
       learn a property of the robot over a whole run. */
    const float seen_s  = 1.4f;
    float       ungated = WALL_FOLLOW_KI_DEG_PER_MM_S * e * seen_s;

    CHECK(fabsf(ungated) < 3.3f,
          "the reduced gain alone cuts the measured windup");
    CHECK(fabsf(ungated) > 0.5f,
          "but leaves more than a bias estimator should move in one cell");

    /* Gated on saturation it does not move at all, which is the point. */
    float gated = sat ? 0.0f : ungated;
    CHECK(gated == 0.0f, "gated on saturation it does not move");

    /* And the gate must NOT fire in the case the term exists for: a small
       standing offset with both walls in view, where P is nowhere near its
       clamp and the residual really is evidence of a bias. */
    const float small = 8.0f;
    CHECK(fabsf(WALL_FOLLOW_KP_DEG_PER_MM * small) < WALL_FOLLOW_MAX_TILT_DEG,
          "an 8 mm standing offset leaves the clamp alone, so it still learns");
  }

  /* The two freezes are independent and both must hold. A single wall reading
     SHORT is unambiguous, so it is not single_far -- only the clamp can stop
     it there, which is exactly the case above. */
  CHECK(err_left(40) < 0.0f && err_left(90) > 0.0f,
        "short reads negative and long reads positive on a left wall");

  printf("%s (%d failures)\n", fails?"FAILED":"ALL CHECKS PASSED", fails);
  return fails!=0; }
