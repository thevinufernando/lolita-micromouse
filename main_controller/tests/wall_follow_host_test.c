/* Host check of the wall follower's error sign and the reverse gain split. */
#include <stdio.h>
#include <math.h>
#include "control_config.h"
#include "navigator.h"
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
  /* Equal readings leave exactly the trim between the two sensors, whatever
     that trim happens to measure. Pinned to the constants rather than to a
     literal: the setpoints are measured values and they move. */
  CHECK(fabsf(err_both(62,62)
              - (WALL_FOLLOW_SETPOINT_RIGHT_MM - WALL_FOLLOW_SETPOINT_LEFT_MM)*0.5f)
        < 0.01f, "both: equal readings -> ~the sensor trim");
  /* The setpoints and the span are two views of the SAME measurement, so a
     robot centred by the setpoints must be accepted by the pair test. Guards
     against one being re-measured and the other left behind. */
  CHECK(pair_ok(WALL_FOLLOW_SETPOINT_LEFT_MM, WALL_FOLLOW_SETPOINT_RIGHT_MM),
        "setpoints and span must describe the same cell");
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
     the caution was written for, and it has to survive the taper.
     EXPRESSED AT THE GATE rather than at a fixed distance -- it was written as
     94 mm when the gate was 95, and silently stopped testing the edge the
     moment the gate moved. What it means is "the furthest reading still
     accepted", so that is what it should say. */
  {
    const float edge = (float)WALL_FOLLOW_USABLE_MAX_MM;
    const float c    = conf_of(edge, WALL_FOLLOW_SETPOINT_LEFT_MM);
    CHECK(fabsf(tilt_single(edge - WALL_FOLLOW_SETPOINT_LEFT_MM, c)) < 4.0f,
          "a reading at the edge of range still cannot lunge");
    CHECK(c < WALL_FOLLOW_TRUST_CONF,
          "and is not trusted enough for the integral to learn from");
  }

  /* THE READINGS THE GATE WAS THROWING AWAY. Across one 17-cell run the side
     sensors returned 33..97 for walls and 192..575 for openings, with nothing
     in between -- and the gate sat at 95, inside the wall cluster. A robot
     30 mm off centre in this corridor IS what a 93 mm reading looks like, and
     going blind at exactly that error is the opposite of what is wanted. */
  {
    CHECK((float)WALL_FOLLOW_USABLE_MAX_MM > 97.0f,
          "a 97 mm reading is a wall of this corridor and must be usable");
    CHECK((float)WALL_FOLLOW_USABLE_MAX_MM < 192.0f,
          "and an opening at 192 mm must still be rejected");

    /* The gate is also the far end of the ramp, so widening it buys authority
       as well as admission -- which is the half that actually corrects. */
    const float c = conf_of(93.0f, WALL_FOLLOW_SETPOINT_LEFT_MM);
    CHECK(fabsf(tilt_single(30.0f, c)) > 4.0f,
          "and 30 mm off with only a far wall now buys a real lean");
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

  /* ---- THE MAP'S VETO ----
     Mirrors vetoed() in wall_follow.c. A cell with no opinion contributes
     nothing; a surveyed cell that records no wall on a side drops it. */
  {
    /* known, wall -> keep | known, no wall -> veto | unknown -> keep */
    #define VETO(known, wall) ((known) ? ((wall) ? 0 : 1) : 0)

    CHECK(VETO(0, 0) == 0, "an unknown cell vetoes nothing");
    CHECK(VETO(0, 1) == 0, "and cannot assert a wall either");
    CHECK(VETO(1, 1) == 0, "a surveyed cell with a wall keeps the reference");
    CHECK(VETO(1, 0) == 1, "a surveyed cell with no wall drops it");

    /* The direction of inference is the safety argument. The veto can only
       ever REMOVE a reference the sensor offered -- there is no combination
       that creates one, so a corrupted map makes the loop decline to correct
       rather than correct toward a wall that is not there. */
    int creates = 0;
    for (int k = 0; k < 2; k++)
      for (int w = 0; w < 2; w++)
        if (VETO(k, w) != 0 && VETO(k, w) != 1) creates = 1;
    CHECK(!creates, "the veto only ever withholds, never adds");
    #undef VETO
  }

  /* ---- WHICH CELL THE SENSORS ARE LOOKING AT ----
     They lead the axle by TOF_SIDE_AHEAD_CM, so they cross the boundary
     before the robot does. Everything after that point is about the cell
     being ENTERED. */
  {
    const float cross = NAV_CELL_CM * 0.5f - TOF_SIDE_AHEAD_CM;

    CHECK(cross > 0.0f,
          "the sensors cross the boundary during the move, not before it");
    CHECK(cross < NAV_CELL_CM * 0.5f,
          "and they cross it EARLIER than the axle does");
    CHECK(cross < NAV_CELL_CM * 0.4f,
          "so most of a move is spent looking at the next cell");

    /* 5.6 cm of a 19.2 cm move on the measured geometry. */
    CHECK(fabsf(cross - 5.6f) < 0.01f, "cross point is where the tape says");
  }

  /* THE MOVE THAT COST A RUN, replayed.
     Leaving a cell whose right wall is recorded, entering one that is known
     and open on that side. The right reference must be dropped the moment the
     sensors cross, not followed 3.4 cm into the next cell. */
  {
    const float cross = NAV_CELL_CM * 0.5f - TOF_SIDE_AHEAD_CM;

    /* The cell being left had a right wall; the cell being entered is known
       and has none. Both are surveyed, as they would be on a second pass. */
    struct { int this_known, right_this, next_known, right_next; } ctx =
        { 1, 1, 1, 0 };

    /* Mirrors vetoed() end to end: pick the cell by travel, then apply it. */
    #define DROPPED(travel) ( (travel) >= cross                                \
        ? (ctx.next_known && !ctx.right_next)                                  \
        : (ctx.this_known && !ctx.right_this) )

    CHECK(!DROPPED(3.0f),
          "before the crossing the wall being left is a valid reference");
    CHECK(DROPPED(9.0f),
          "after it the reference is dropped, not chased into the next cell");
    CHECK(!DROPPED(cross - 0.01f) && DROPPED(cross + 0.01f),
          "and the change happens exactly at the crossing");

    /* The first-pass case: the next cell has never been surveyed, so it has no
       opinion and the reference survives. This is the common case and it must
       leave the loop exactly as it was before any of this existed. */
    ctx.next_known = 0;
    CHECK(!DROPPED(9.0f),
          "an unsurveyed next cell changes nothing");

    /* And a next cell that genuinely has a wall keeps the reference too. */
    ctx.next_known = 1; ctx.right_next = 1;
    CHECK(!DROPPED(9.0f),
          "a next cell that does have a wall keeps it");
    #undef DROPPED
  }

  /* ---- THREE GATES ON THE INTEGRAL, GUARDING THREE DIFFERENT THINGS ----
     conf    : the reference is not worth believing
     clamped : the loop is already asking for everything it can
     moving  : the loop asked correctly and the ROBOT did not answer
     A term that learns a property of the robot must update under none. */
  {
    #define LEARNS(conf, clamped, moving) \
        ((conf) >= WALL_FOLLOW_TRUST_CONF && !(clamped) && (moving))

    CHECK(LEARNS(1.0f, 0, 1), "all three clear: it learns");
    CHECK(!LEARNS(0.5f, 0, 1), "a doubtful reference stops it");
    CHECK(!LEARNS(1.0f, 1, 1), "a saturated proportional term stops it");
    CHECK(!LEARNS(1.0f, 0, 0), "and a robot that is not moving stops it");

    /* Independence: no gate can rescue another. */
    int rescued = 0;
    for (int c = 0; c < 2; c++)
      for (int k = 0; k < 2; k++)
        for (int mv = 0; mv < 2; mv++)
          if (LEARNS(c ? 1.0f : 0.5f, k, mv) && (!c || k || !mv)) rescued = 1;
    CHECK(!rescued, "every gate is independently sufficient to stop it");
    #undef LEARNS
  }

  /* The movement gate has to admit ordinary travel and reject a wedge. The
     measured wedge was 3.8 cm/s against a profile asking for 10. */
  CHECK(WALL_FOLLOW_MIN_TRAVEL_CMS < STRAIGHT_PROFILE_MAX_CMS * 0.5f,
        "the movement threshold is well under cruise");
  CHECK(WALL_FOLLOW_MIN_TRAVEL_CMS > 0.5f,
        "and above the noise on a differenced encoder reading");

  /* ---- GIVING UP ON A WEDGE ----
     Distinct from the breakaway, which only arms after the profile ends. The
     abort window must outlast the entire breakaway sequence, or a move would
     be abandoned while the recovery it already has is still being tried. */
  {
    const float breakaway_ms = (float)STRAIGHT_BREAKAWAY_MAX
                             * ((float)STRAIGHT_BREAKAWAY_MS
                                + (float)STRAIGHT_BREAKAWAY_CYCLES
                                  * CONTROL_SAMPLE_TIME_S * 1000.0f);

    CHECK((float)STRAIGHT_STALL_ABORT_MS > breakaway_ms,
          "the stall abort outlasts the whole breakaway sequence");
    CHECK(STRAIGHT_STALL_RATE_CMS < STRAIGHT_PROFILE_MAX_CMS * 0.25f,
          "and only fires well below the commanded speed");
    /* It also has to be shorter than the grind that prompted it. */
    CHECK((float)STRAIGHT_STALL_ABORT_MS < 2000.0f,
          "while still cutting the two seconds of grinding short");
  }

  /* ---- FINISHED MEANS CLOSE ENOUGH *AND* STOPPED ----
     The completion test was position only, so a robot crossing the band at
     cruise called the move done and coasted on. Measured at 11.2 cm/s against
     a 1.5 cm band, which is 22 to 25 mm of overshoot -- and a 90 degree turn
     converts that into lateral error almost one for one. */
  {
    const float cruise = STRAIGHT_PROFILE_MAX_CMS;

    CHECK(STRAIGHT_SETTLE_SPEED_CMS < cruise * 0.3f,
          "the settle speed is a small fraction of cruise");
    CHECK(STRAIGHT_SETTLE_SPEED_CMS > STRAIGHT_STALL_RATE_CMS,
          "and above the stall threshold, so stopping is not read as wedging");

    /* What the old test allowed, stated as the distance it let through. */
    const float coast_old = cruise * CONTROL_SAMPLE_TIME_S
                            * (float)CONTROL_SETTLE_CYCLES;
    const float coast_new = STRAIGHT_SETTLE_SPEED_CMS * CONTROL_SAMPLE_TIME_S
                            * (float)CONTROL_SETTLE_CYCLES;

    CHECK(coast_new < coast_old * 0.25f,
          "and it cuts the distance travelled during settling by 4x or more");
  }

  printf("%s (%d failures)\n", fails?"FAILED":"ALL CHECKS PASSED", fails);
  return fails!=0; }
