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
/* Mirrors the WALL_FOLLOW_ANGLED branch. */
static float err_angled(float L45,float R45){
    return (L45-R45)*0.5f*TOF_ANGLED_LATERAL_GAIN; }
/* Forward model: what the angled pair READS for a lateral offset e (mm,
   positive = robot displaced RIGHT) in a corridor of inner width W.
   Includes TOF_NEAR_FIELD_BIAS_MM, because TOF_OFFSET_* are 0 so every
   reading the firmware sees carries it -- and every constant compared against
   a reading must therefore be in the same space. */
#define TOF_NEAR_FIELD_BIAS_MM 17.0f
static float a_left (float e){ return ((MAZE_CORRIDOR_INNER_MM*0.5f)
    + (-(TOF_SIDE_SPAN_MM*0.5f - TOF_ANGLED_INBOARD_MM)) + e)/TOF_ANGLED_COS45
    + TOF_NEAR_FIELD_BIAS_MM; }
static float a_right(float e){ return ((MAZE_CORRIDOR_INNER_MM*0.5f)
    - ( (TOF_SIDE_SPAN_MM*0.5f - TOF_ANGLED_INBOARD_MM)) - e)/TOF_ANGLED_COS45
    + TOF_NEAR_FIELD_BIAS_MM; }
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
  /* !! THE SETPOINTS AND THE SPAN ARE NO LONGER IN THE SAME SPACE !!
     Until 2026-09-18 both were measured READINGS, so a robot centred by the
     setpoints summed to the span and this test asserted exactly that. The
     setpoints are now the TRUE geometric centre (35 mm) while the span is
     still reading-space (104 mm), because the raw-reading thresholds have not
     been re-derived yet -- see the note at WALL_FOLLOW_SETPOINT_LEFT_MM.

     What must still hold is that a real centred robot, whose SENSORS read
     about 52 each, is accepted by the pair test. That is the property the
     span actually guards, and it is unaffected by where the setpoint aims. */
  {
    const float reads_when_centred =
        (MAZE_CORRIDOR_INNER_MM - TOF_SIDE_SPAN_MM) * 0.5f + 17.0f;

    CHECK(pair_ok(reads_when_centred, reads_when_centred),
          "a centred robot's READINGS are still accepted by the span check");

    /* Span and setpoints are BOTH reading-space, so half the span is the
       setpoint. They were briefly in different spaces and that is exactly the
       bug this file now guards against. */
    CHECK(fabsf((WALL_FOLLOW_SPAN_MM * 0.5f)
                - WALL_FOLLOW_SETPOINT_LEFT_MM) < 2.0f,
          "half the span equals the setpoint: both are readings");
  }
  /* Single wall corrects only AWAY from a wall that is too close; a long
     reading is ambiguous at a junction and must produce no command. */
  CHECK(err_left(WALL_FOLLOW_SETPOINT_LEFT_MM - 12.0f)<0,
        "left only: too close to left -> move right");
  CHECK(err_right(WALL_FOLLOW_SETPOINT_RIGHT_MM - 12.0f)>0,
        "right only: too close to right -> move left");
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
  /* And that non-cancelling bias is exactly what put the setpoint 17 mm out
     until 2026-09-18 -- see the block at the end of this file. */

  /* the tilt always points the way the correction needs to go */
  /* EXPRESSED RELATIVE TO THE SETPOINT, not as a literal. These used to say
     40 mm, which meant "too close" only while the setpoint was 52; when it
     moved to 35 the literal silently started meaning "too far" and the tests
     failed for the right reason. Same rot Hiruna hit with the hard-coded
     94 mm usable-gate case. */
  CHECK(tilt(err_left(WALL_FOLLOW_SETPOINT_LEFT_MM - 12.0f))<0,
        "too close to the left -> nose right");
  CHECK(tilt(err_right(WALL_FOLLOW_SETPOINT_RIGHT_MM - 12.0f))>0,
        "too close to the right -> nose left");
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
  CHECK(err_left(WALL_FOLLOW_SETPOINT_LEFT_MM - 12.0f) < 0.0f &&
        err_left(WALL_FOLLOW_SETPOINT_LEFT_MM + 38.0f) > 0.0f,
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

  /* ================= ANGLED-PAIR CENTRING (the 45 sensors) ==============
     Added 2026-09-18. The side pair goes blind below ~30 mm and a centred
     robot is only 35 mm from each wall, so it stops measuring exactly the
     errors it exists to correct. These pin the geometry the new path rests
     on -- all derived, so a wrong constant fails here and not in the arena. */
  {
    const float centred_side_gap =
        (MAZE_CORRIDOR_INNER_MM - TOF_SIDE_SPAN_MM) * 0.5f;

    CHECK(fabsf(centred_side_gap - 35.0f) < 0.51f,
          "a centred robot is 35 mm from each side wall");
    CHECK(centred_side_gap - 30.0f < 6.0f,
          "which clears the sensor's ~30 mm floor by under 6 mm -- the problem");

    CHECK(fabsf(a_left(0.0f) - TOF_ANGLED_NOMINAL_MM) < 1.0f,
          "centred, an angled sensor reads TOF_ANGLED_NOMINAL_MM");
    CHECK(a_left(0.0f) > 2.0f * 30.0f,
          "which is more than twice the sensor floor");

    /* ZERO WHEN CENTRED. Wrong here and the robot holds to one side of every
       corridor, which looks like a gain problem. */
    CHECK(fabsf(err_angled(a_left(0.0f), a_right(0.0f))) < 0.01f,
          "centred gives exactly zero angled error");

    /* SIGN. Positive must mean "move left", matching every other branch;
       backwards makes the loop positive feedback. */
    CHECK(err_angled(a_left(10.0f), a_right(10.0f)) > 0.0f,
          "displaced RIGHT gives positive error (move left), as the side pair does");
    CHECK(err_angled(a_left(-10.0f), a_right(-10.0f)) < 0.0f,
          "and displaced LEFT gives negative");

    /* THE GAIN CONVERSION -- silently 41% hot if dropped. The result must be
       mm of LATERAL offset, not mm along the beam, so that
       WALL_FOLLOW_KP_DEG_PER_MM carries over from the side pair unchanged. */
    for (float e = -25.0f; e <= 25.0f; e += 5.0f) {
      CHECK(fabsf(err_angled(a_left(e), a_right(e)) - e) < 0.05f,
            "angled error equals true lateral offset in mm across the range");
    }

    /* HEADROOM -- the whole claim of the change, both halves. */
    CHECK(a_right(20.0f) - TOF_NEAR_FIELD_BIAS_MM > 30.0f,
          "at 20 mm off centre the near ANGLED beam is still above the floor");
    CHECK((MAZE_CORRIDOR_INNER_MM*0.5f) - (TOF_SIDE_SPAN_MM*0.5f) - 20.0f < 30.0f,
          "while the near SIDE reading is already below it");

    /* The sum is fixed by corridor width alone, which is what lets the span
       check reject a beam that has left the corridor. */
    for (float e = -25.0f; e <= 25.0f; e += 12.5f) {
      CHECK(fabsf((a_left(e) + a_right(e)) - TOF_ANGLED_SPAN_MM) < 1.0f,
            "the angled pair sums to TOF_ANGLED_SPAN_MM at any offset");
    }

    /* Every legitimate reading must pass the window, or genuine pairs fall
       through to the fallback -- the failure Hiruna recorded for the side
       pair's span tolerance. */
    for (float e = -25.0f; e <= 25.0f; e += 5.0f) {
      CHECK(a_left(e)  >= (float)TOF_ANGLED_MIN_MM &&
            a_left(e)  <= (float)TOF_ANGLED_MAX_MM &&
            a_right(e) >= (float)TOF_ANGLED_MIN_MM &&
            a_right(e) <= (float)TOF_ANGLED_MAX_MM,
            "readings across +/-25 mm of offset sit inside the usable window");
    }

    CHECK(fabsf((MAZE_CORRIDOR_INNER_MM + MAZE_WALL_THICKNESS_MM)
                - NAV_CELL_CM * 10.0f) < 0.1f,
          "corridor + wall thickness equals the cell pitch");

    CHECK((float)STRAIGHT_TOF_DIVIDER == 5.0f,
          "the poll rotation covers all five sensors");
    CHECK(WALL_FOLLOW_UPDATE_S * 1000.0f < (float)TOF_MAX_SAMPLE_AGE_MS,
          "and a full rotation still completes inside the staleness cap");
  }

  /* ============ EVERYTHING THE LOOP COMPARES IS A READING ==============
     Rewritten 2026-09-19 after the previous version asserted the opposite and
     cost half a run.

     wf_error_mm = reading - setpoint, and the readings are RAW: TOF_OFFSET_*
     are all 0, so every reading carries the sensor's ~17 mm near-field
     over-read. Both sides of that subtraction must live in the same space.
     The setpoints are therefore MEASURED READINGS (52 mm), not true gaps
     (35 mm) -- with 35 a centred robot reads 52, is told it is 17 mm too far
     from the wall, and drives into it. That is what took 26 cells down to
     13. */
  {
    const float reads_when_centred = 52.0f;   /* measured, robot centred */
    const float true_centred =
        (MAZE_CORRIDOR_INNER_MM - TOF_SIDE_SPAN_MM) * 0.5f;   /* 35 */

    CHECK(fabsf(WALL_FOLLOW_SETPOINT_LEFT_MM - reads_when_centred) < 2.0f,
          "left setpoint is a READING, matching what a centred robot sees");
    CHECK(fabsf(WALL_FOLLOW_SETPOINT_RIGHT_MM - reads_when_centred) < 2.0f,
          "and so is the right setpoint");

    /* The trap, pinned so it cannot be re-introduced. */
    CHECK(fabsf(WALL_FOLLOW_SETPOINT_LEFT_MM - true_centred) > 10.0f,
          "the setpoint is NOT the true gap -- that mixes reading and true space");

    /* A centred robot must be told it is centred. */
    CHECK(fabsf(err_left(reads_when_centred)) < 2.0f,
          "a centred robot following the LEFT wall reads ~zero error");
    CHECK(fabsf(err_right(reads_when_centred)) < 2.0f,
          "and ~zero following the RIGHT wall");

    /* The bias must cancel, which is the entire reason for reading-space. */
    CHECK(fabsf(err_left(true_centred + 17.0f)) < 2.0f,
          "a true 35 mm gap reads 52 and still gives zero error");

    /* Setpoints equal, so the two-wall difference path is untouched. */
    CHECK(fabsf(WALL_FOLLOW_SETPOINT_LEFT_MM - WALL_FOLLOW_SETPOINT_RIGHT_MM)
              < 0.01f,
          "the setpoints stay equal, so the two-wall difference is unchanged");
    CHECK(fabsf(err_both(50.0f, 30.0f) - 10.0f) < 0.01f,
          "and the two-wall error is still half the raw difference");

    /* The span check and the setpoints are both reading-space again, so a
       centred robot's readings must satisfy both. */
    CHECK(pair_ok(WALL_FOLLOW_SETPOINT_LEFT_MM, WALL_FOLLOW_SETPOINT_RIGHT_MM),
          "setpoints and span describe the same cell once more");
  }

  /* ============ THE ANGLED CONSTANTS ARE READING-SPACE TOO ==============
     Same bug, same fix. TOF_ANGLED_NOMINAL_MM was 70.7 (true geometry) while
     the beams actually read ~88, so the pair summed to ~175 against a window
     centred on 141 -- it sat on the very edge and was usually rejected. That
     is why wf_side never once showed ANGLED in a real run. */
  {
    const float true_nominal =
        ((MAZE_CORRIDOR_INNER_MM * 0.5f)
         - (TOF_SIDE_SPAN_MM * 0.5f - TOF_ANGLED_INBOARD_MM)) / TOF_ANGLED_COS45;

    CHECK(fabsf(TOF_ANGLED_NOMINAL_MM - (true_nominal + TOF_NEAR_FIELD_BIAS_MM)) < 2.0f,
          "angled nominal is a READING: true geometry plus the over-read");
    CHECK(fabsf(TOF_ANGLED_SPAN_MM - 2.0f * TOF_ANGLED_NOMINAL_MM) < 2.0f,
          "and the angled span is twice it, so the two agree");

    /* A centred robot's actual angled readings must pass the span check with
       margin -- not sit on its edge, which is what failed before. */
    const float centred_read = a_left(0.0f);
    const float margin =
        TOF_ANGLED_SPAN_TOL_MM - fabsf(2.0f * centred_read - TOF_ANGLED_SPAN_MM);
    CHECK(margin > 20.0f,
          "a centred robot clears the angled span check by a wide margin");

    /* The pair difference is bias-immune -- the reason angled centring works
       without any offset calibration. Both beams gain the same 17 mm. */
    for (float e = -25.0f; e <= 25.0f; e += 5.0f) {
      CHECK(fabsf(err_angled(a_left(e), a_right(e)) - e) < 0.05f,
            "the angled difference still reports true lateral offset with bias");
    }

    /* Single-beam readings across the working range must sit inside the
       one-beam window, or the new single-angled fallback never engages. */
    for (float e = -25.0f; e <= 25.0f; e += 5.0f) {
      CHECK(a_left(e)  >= (float)TOF_ANGLED_MIN_MM &&
            a_left(e)  <= (float)TOF_ANGLED_MAX_MM &&
            a_right(e) >= (float)TOF_ANGLED_MIN_MM &&
            a_right(e) <= (float)TOF_ANGLED_MAX_MM,
            "single angled readings stay inside the one-beam window");
    }
  }

  /* ============ THE NEAR-FIELD GATE ON SIDE SENSORS ==================== */
  {
    CHECK(WALL_FOLLOW_USABLE_MIN_MM > 30U,
          "the side floor sits ABOVE the sensor's 30 mm limit, not at it");
    CHECK((float)WALL_FOLLOW_USABLE_MIN_MM < WALL_FOLLOW_SETPOINT_LEFT_MM,
          "but below the setpoint, so a centred robot is never rejected");

    /* The 5 mm reading that ended a run must now be refused outright. */
    CHECK(5U < WALL_FOLLOW_USABLE_MIN_MM,
          "the 5 mm reading from the stuck run is now rejected");

    /* And the angled beam that replaces it is nowhere near its own floor. */
    CHECK(TOF_ANGLED_NOMINAL_MM > 2.0f * 30.0f,
          "the angled reference sits at more than twice the sensor floor");
  }

  /* ============ URGENT SLEW AFTER A PIVOT (2026-09-19) ================
     WallFollow_Reset() zeroes the tilt at every turn. At the normal 20 deg/s
     the lean takes 0.5 s to rebuild -- 70 mm of a 192 mm cell -- so the cell
     that inherits the pivot's error is the one with the least authority to
     remove it. A run entered a cell 25 mm off and was still 29 mm off a cell
     later. */
  {
    const float clamp = WALL_FOLLOW_MAX_TILT_DEG;
    const float t_norm = clamp / WALL_FOLLOW_TILT_SLEW_DPS;
    const float t_urg  = clamp / WALL_FOLLOW_URGENT_SLEW_DPS;
    const float speed_mm_s = STRAIGHT_PROFILE_MAX_CMS * 10.0f;

    CHECK(WALL_FOLLOW_URGENT_SLEW_DPS > WALL_FOLLOW_TILT_SLEW_DPS,
          "the urgent slew is faster than the normal one");

    /* The ramp must fit in a small fraction of a cell, or it is not a fix. */
    CHECK(t_urg * speed_mm_s < NAV_CELL_CM * 10.0f * 0.2f,
          "at the urgent rate the lean is up within 20% of a cell");
    CHECK(t_norm * speed_mm_s > NAV_CELL_CM * 10.0f * 0.3f,
          "whereas the normal rate takes over 30% of it -- the problem");

    /* THE CASCADE RULE STILL HAS TO HOLD. A ramping target costs the inner
       heading loop R/(KP/FF) degrees of standing error; spend more than its
       linear range and the loop saturates, delivering LESS correction. */
    const float inner_rate = STRAIGHT_YAW_KP / TURN_FF_GAIN;   /* deg/s per deg */
    const float ramp_cost  = WALL_FOLLOW_URGENT_SLEW_DPS / inner_rate;
    CHECK(ramp_cost < STRAIGHT_YAW_LIMIT / STRAIGHT_YAW_KP,
          "and the urgent ramp still fits inside the heading loop's linear range");

    /* IT MUST NOT FIRE ON NOISE -- which is the real constraint, and is what
       the old "> 10 mm" assertion was standing in for. Filtered side-reading
       noise is about 0.94 mm sigma (host-measured in tof_filter_host_test),
       so the threshold has to be many sigma clear of it. Expressed against
       the noise rather than a literal, so lowering the threshold to arm
       earlier does not silently become "arms on noise". */
    const float read_noise_sigma_mm = 0.94f;
    CHECK(WALL_FOLLOW_URGENT_ERR_MM > 5.0f * read_noise_sigma_mm,
          "the urgent threshold is several sigma clear of reading noise");

    /* And it must arm while there is still clearance to recover in. At 20 mm
       it armed with only 15 mm of the 35 mm nominal gap left, by which point
       the robot was nearly touching -- that cost a run. */
    const float nominal_clear =
        (MAZE_CORRIDOR_INNER_MM - TOF_SIDE_SPAN_MM) * 0.5f;

    CHECK(WALL_FOLLOW_URGENT_ERR_MM < nominal_clear * 0.5f,
          "and arms before half the clearance is gone, leaving room to recover");
    CHECK(WALL_FOLLOW_URGENT_ERR_MM < nominal_clear,
          "and well below the nominal clearance, so it fires before contact");
  }

  /* ======== THE COMMANDED HEADING MAY NOT LEAVE THE MAZE AXIS =========
     The tilt and the drift are clamped separately and then ADDED, and nothing
     bounded the sum. A logged run reached tilt -10.00 (pinned) plus drift
     -5.69, giving a measured yaw of -108.15 against a -90.00 target: the robot
     drove a corridor 18 degrees crabbed and wedged. It looked like a turn
     overshooting by 18 degrees and was not -- the turn target was a clean
     -90.00 at every cell. */
  {
    const float worst_sum = WALL_FOLLOW_MAX_TILT_DEG + WALL_FOLLOW_KI_LIMIT_DEG;

    CHECK(worst_sum > 15.0f,
          "tilt and drift can together ask for more than 15 deg of lean");
    CHECK(STRAIGHT_MAX_AXIS_LEAN_DEG < worst_sum,
          "so the axis clamp must be tighter than their sum -- that is its job");

    /* Above the tilt clamp, or the clamp silently caps ordinary cornering. */
    CHECK(STRAIGHT_MAX_AXIS_LEAN_DEG > WALL_FOLLOW_MAX_TILT_DEG,
          "but looser than the tilt clamp, so full cornering authority survives");

    /* The lean it permits must still be geometrically sane: at this angle one
       cell of travel must not sweep the robot across the corridor. */
    const float half_gap =
        (MAZE_CORRIDOR_INNER_MM - TOF_SIDE_SPAN_MM) * 0.5f;
    const float sweep =
        NAV_CELL_CM * 10.0f * sinf(STRAIGHT_MAX_AXIS_LEAN_DEG * 3.14159265f / 180.0f);
    CHECK(sweep < MAZE_CORRIDOR_INNER_MM,
          "a cell at max lean does not sweep wider than the corridor");
    CHECK(half_gap > 0.0f, "corridor geometry is sane");

    /* The 18 deg that actually wedged the robot must now be refused. */
    CHECK(18.0f > STRAIGHT_MAX_AXIS_LEAN_DEG,
          "the 18 deg lean from the wedged run is now clamped away");
  }

  /* ====== WHY THERE IS NO CORROBORATION GATE ON THE ALIGNMENT ========
     A "two consecutive front readings must agree" gate was added on
     2026-09-19 and reverted the same day: it made sl_align_applied read 0 for
     an entire run. These pin the arithmetic that makes such a gate
     unworkable, so the idea is not re-tried from scratch. */
  {
    /* The align check runs once per ToF rotation, and the robot closes real
       distance in that time -- the reading is SUPPOSED to change. Any
       agreement window must therefore exceed the closing distance, which
       immediately makes it too wide to reject the noise it was meant to
       catch. */
    const float closing_per_check =
        STRAIGHT_PROFILE_MAX_CMS * 10.0f * WALL_FOLLOW_UPDATE_S;

    CHECK(closing_per_check > 5.0f,
          "the front reading moves several mm between align checks by design");

    /* And the filter cannot smooth that away: on a ramp an EMA lags by
       roughly step/alpha, which here is many times the noise amplitude. */
    const float ema_lag = closing_per_check / TOF_FILTER_EMA_ALPHA;
    CHECK(ema_lag > 20.0f,
          "and the EMA lags a closing target by more than any sane window");

    /* Worse, the EMA lag itself exceeds the jump threshold, so when the
       filter finally snaps to raw the output moves further in one sample
       than any usable agreement window -- resetting the count at exactly the
       moment the alignment most needs to fire. */
    CHECK(ema_lag > (float)TOF_FILTER_JUMP_THRESHOLD_MM,
          "the EMA lag exceeds the jump threshold, so a snap breaks any agreement run");

    /* The guard that then refuses the late, large correction. */
    CHECK(WALL_FRONT_ALIGN_MAX_CM > 0.0f && WALL_FRONT_ALIGN_MAX_CM < 8.0f,
          "so a gate that only opens late gets refused as BIG_DELTA");
  }

  /* ===== A CHAINED SEGMENT MUST BE ABLE TO STOP FOR A FRONT WALL =====
     A 57-cell run ended by driving into a wall it had detected correctly --
     5 of 5 front votes at 81 mm. Chaining had committed the segment to exit
     at cruise, and the chained front target is
     WALL_FRONT_ALIGN_MM + CELL_DECISION_OFFSET_CM*10 = 135 mm, so at 81 mm
     the segment was already 54 mm past the point where it could finish. */
  {
    const float offset_cm =
        (CELL_CHAIN_SPEED_CMS * CELL_CHAIN_SPEED_CMS)
            / (2.0f * STRAIGHT_PROFILE_ACCEL_CMS2)
        + CELL_DECISION_MARGIN_CM;

    const float chained_target_mm = WALL_FRONT_ALIGN_MM + offset_cm * 10.0f;

    /* The offset really is a braking distance, not a fudge. */
    CHECK(offset_cm > CELL_DECISION_MARGIN_CM,
          "the decision offset includes a real braking distance");

    /* !! THE GUARD PROJECTS TO THE END OF THE SEGMENT !!
     *
     * Two runs fired it ZERO times because it compared the CURRENT front
     * reading against the target. It runs at the START of a segment, standing
     * a full cell pitch from the wall the segment will finish at, so the
     * reading there is ~192 mm larger than the threshold BY CONSTRUCTION.
     * "Is the wall close now" is guaranteed to answer no at the one moment it
     * is asked. The test is on the reading the sensor will have when the
     * segment ENDS. */
    const float cell_mm = NAV_CELL_CM * 10.0f;
    const float trip    = chained_target_mm - CELL_CHAIN_WALL_MARGIN_MM;

    /* What the front reads at a decision point with a wall N cells ahead. */
    #define FRONT_AT(n) (chained_target_mm + ((float)(n) - 1.0f) * cell_mm)

    /* One cell ahead: the segment ends at the wall, so it must NOT exit at
       cruise. This is the case that drove the robot into a wall. */
    CHECK(FRONT_AT(1) - cell_mm < trip,
          "a wall one cell ahead forces a rest exit");

    /* Two cells ahead: the next segment re-evaluates one cell closer and
       still inherits the full braking offset, so cruising on is correct --
       and stopping here would cost a full stop for most cells in a maze. */
    CHECK(FRONT_AT(2) - cell_mm >= trip,
          "but a wall two cells ahead still allows a cruise exit");
    CHECK(FRONT_AT(3) - cell_mm > trip,
          "and three cells ahead certainly does");

    /* The margin exists so the two-cell boundary does not turn on
       floating-point equality. */
    CHECK(CELL_CHAIN_WALL_MARGIN_MM > 0.0f,
          "the boundary case is resolved by a margin, not by exact equality");
    CHECK(CELL_CHAIN_WALL_MARGIN_MM < chained_target_mm * 0.5f,
          "and the margin is small enough not to mask a genuine wall");
    #undef FRONT_AT

    /* The threshold has to leave the following segment room to brake. */
    CHECK(chained_target_mm - WALL_FRONT_ALIGN_MM > offset_cm * 10.0f - 1.0f,
          "and the margin it preserves is the braking distance itself");

    /* THE GUARD MUST NOT DEPEND ON THE SIDE SENSORS.
     *
     * The first version gated on ToF_ReadAllLatest() == TOF_OK, which is
     * TOF_OK only when ALL THREE navigation sensors serve a fresh reading.
     * A side sensor looking at an opening -- most corridors -- makes it
     * return TOF_ERROR, so the && short-circuited and the front reading was
     * never examined. tm_chain_wall_stops read 0 for a whole run while the
     * robot drove into a wall.
     *
     * Expressed as the property that matters: a front wall must be actionable
     * regardless of what the sides are doing, and the sides routinely read
     * far beyond any wall distance. */
    const float opening_mm = 700.0f;   /* measured: right sensor read 701 */
    CHECK(opening_mm > WALL_SIDE_THRESHOLD_MM,
          "a side opening reads far past the wall threshold");
    CHECK(opening_mm > (float)WALL_FOLLOW_USABLE_MAX_MM,
          "and past the follower's usable gate, so the sweep reports an error");
    CHECK(chained_target_mm < opening_mm,
          "yet the front guard must still act -- it cannot depend on that sweep");
  }

  /* ======== A BLOCKING INDICATOR MUST NOT RUN WHILE ROLLING ==========
     The goal LED pattern blocked for 4.2 s. With chaining the robot is still
     at cruise when the milestone fires, so tm_chain_gap_ms_max measured
     4226 ms of open-loop travel -- about 59 cm, three cells, blind. */
  {
    const float blink_ms =
        (float)MAZE_GOAL_BLINK_REPEATS
        * (float)(MAZE_GOAL_BLINK_LONG_MS + MAZE_GOAL_BLINK_GAP_MS
                  + 2U * (MAZE_GOAL_BLINK_SHORT_MS + MAZE_GOAL_BLINK_GAP_MS)
                  + MAZE_GOAL_BLINK_PAUSE_MS);

    const float open_loop_cm = CELL_CHAIN_SPEED_CMS * blink_ms * 0.001f;

    CHECK(open_loop_cm > NAV_CELL_CM,
          "the blink is long enough to cross a cell -- so it must stop first");
    CHECK(blink_ms > 1000.0f,
          "and is far longer than any gap the chain margin can absorb");
  }

  /* ====== WEDGED IS NOT ALWAYS STOPPED: THE PROGRESS TEST ==========
     A run reached the goal and failed on the way back, creeping 140 mm in
     9.04 s -- 1.55 cm/s against a 1.50 stall threshold. It missed by five
     hundredths and burned the whole 8 s timeout while jammed on something no
     ToF beam could see. */
  {
    const float crept_cms = 14.0f / 9.04f;          /* 140 mm in 9.04 s */
    const float cell_cm   = NAV_CELL_CM;

    /* The creep that was missed really is above the stall threshold -- this
       is the gap the new test fills, stated as a number. */
    CHECK(crept_cms > STRAIGHT_STALL_RATE_CMS,
          "the measured creep evades the stall threshold");

    /* And really is too slow to finish a cell inside the timeout. */
    const float need_to_finish =
        cell_cm / ((float)CONTROL_MOVE_TIMEOUT_MS * 0.001f);
    CHECK(crept_cms < need_to_finish,
          "yet cannot cover a cell before the timeout -- so it must be failed");

    /* The new gate must catch it. */
    CHECK(crept_cms < STRAIGHT_PROGRESS_MAX_CMS,
          "the creep is inside the progress test's speed window");

    /* But the window must not swallow a healthy move. */
    CHECK(STRAIGHT_PROGRESS_MAX_CMS < STRAIGHT_PROFILE_MAX_CMS * 0.5f,
          "while a cruising robot is far outside it");

    /* The margin must actually relax the requirement, or a move marginally
       behind schedule gets failed for no good reason. */
    CHECK(STRAIGHT_PROGRESS_MARGIN > 0.0f && STRAIGHT_PROGRESS_MARGIN < 1.0f,
          "the margin only ever relaxes the required rate");

    /* The end-of-move guard has to cover more than one control cycle, or the
       required rate blows up in the final milliseconds and fails everything. */
    CHECK((float)STRAIGHT_PROGRESS_MIN_MS
              > CONTROL_SAMPLE_TIME_S * 1000.0f * 10.0f,
          "and the test is disabled well before the deadline, not at it");
  }

  /* ====== THE TRACE MUST OUTLAST THE RUN ========== */
  {
    /* 82 moves filled a 64-record buffer, so the failure was in the
       unrecorded tail and had to be reconstructed from live globals. */
    CHECK(MAZE_TRACE_CAPACITY >= 128U,
          "the trace holds a full explore + return + speed run");
    CHECK((uint32_t)MAZE_TRACE_CAPACITY * 60U < 16384U,
          "and still costs a small fraction of RAM");
  }

  printf("%s (%d failures)\n", fails?"FAILED":"ALL CHECKS PASSED", fails);
  return fails!=0; }
