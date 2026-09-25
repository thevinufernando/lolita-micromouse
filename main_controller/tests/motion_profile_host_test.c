/* Host-side verification of the trapezoidal profile generator.
 *
 * Worth testing off-target because the failure mode is quiet: a profile that
 * lands slightly short, or whose velocity does not integrate to its own
 * position, produces a turn that is consistently a degree or two off with
 * nothing in the logs to say why. Here it can be checked against exact maths.
 *
 * Runs on the host, no hardware. Exit code 0 = pass. */
#include "motion_profile.h"
#include <stdio.h>
#include <math.h>

static int failures = 0;

static void check(const char *name, float got, float expect, float tol)
{
    int ok = fabsf(got - expect) <= tol;
    if (!ok) failures++;
    printf("  [%s] %-46s got=%9.4f expect=%9.4f\n",
           ok ? "PASS" : "FAIL", name, got, expect);
}

static void check_true(const char *name, int cond, const char *detail)
{
    if (!cond) failures++;
    printf("  [%s] %-46s %s\n", cond ? "PASS" : "FAIL", name, detail);
}

/* Numerically integrate the velocity and compare against the reported
 * position. These must agree or the feedforward is fighting the feedback.
 *
 * Accumulates in DOUBLE and derives t from the step index rather than adding
 * dt repeatedly. With float accumulation over ~60k steps the harness itself
 * loses about 0.1 units, which looks exactly like a profile that lands short.
 * The test has to be more precise than the thing it is testing. */
static float integrate(const MotionProfile_t *p, float t_end, double dt)
{
    double s = 0.0;
    long n = (long)(t_end / dt);
    for (long i = 0; i < n; i++) {
        s += (double)MotionProfile_Velocity(p, (float)((i + 0.5) * dt)) * dt;
    }
    return (float)s;
}

int main(void)
{
    MotionProfile_t p;

    printf("\n===== motion profile host tests =====\n");

    printf("\nTEST 1: a 90 deg turn at 200 deg/s, 1200 deg/s^2 (trapezoid)\n");
    MotionProfile_Init(&p, 90.0f, 200.0f, 1200.0f);
    check("peak rate reaches the limit", p.v_peak, 200.0f, 0.01f);
    check("ramp time = v/a", p.t_ramp, 200.0f / 1200.0f, 1e-4f);
    check("total duration", MotionProfile_Duration(&p), 2.0f * p.t_ramp + p.t_cruise, 1e-6f);
    check_true("it does cruise", p.t_cruise > 0.0f, "long enough to reach v_max");
    check("lands exactly on target", MotionProfile_Position(&p, p.t_total), 90.0f, 1e-3f);
    check("starts at zero", MotionProfile_Position(&p, 0.0f), 0.0f, 1e-6f);
    check("velocity is zero at both ends",
          fabsf(MotionProfile_Velocity(&p, 0.0f)) + fabsf(MotionProfile_Velocity(&p, p.t_total)),
          0.0f, 1e-6f);

    printf("\nTEST 2: velocity integrates to position (feedforward consistency)\n");
    check("integral of v = reported position", integrate(&p, p.t_total, 1e-5f), 90.0f, 0.02f);
    check("midpoint agrees", integrate(&p, p.t_total * 0.5f, 1e-5f),
          MotionProfile_Position(&p, p.t_total * 0.5f), 0.02f);

    printf("\nTEST 3: a short move degenerates to a triangle, not a clamp\n");
    MotionProfile_Init(&p, 5.0f, 200.0f, 1200.0f);
    check_true("no cruise phase", p.t_cruise == 0.0f, "too short to reach v_max");
    check_true("peak is below the limit", p.v_peak < 200.0f, "sqrt(a*s)");
    check("peak = sqrt(accel*distance)", p.v_peak, sqrtf(1200.0f * 5.0f), 1e-3f);
    check("still lands on target", MotionProfile_Position(&p, p.t_total), 5.0f, 1e-3f);
    check("integral still agrees", integrate(&p, p.t_total, 1e-6f), 5.0f, 0.01f);

    printf("\nTEST 4: sign is carried, magnitude is not\n");
    MotionProfile_t n;
    MotionProfile_Init(&p,  90.0f, 200.0f, 1200.0f);
    MotionProfile_Init(&n, -90.0f, 200.0f, 1200.0f);
    check("same duration either way", MotionProfile_Duration(&n), MotionProfile_Duration(&p), 1e-6f);
    check("right turn lands on -90", MotionProfile_Position(&n, n.t_total), -90.0f, 1e-3f);
    check("mirrored mid-move position",
          MotionProfile_Position(&n, 0.1f), -MotionProfile_Position(&p, 0.1f), 1e-6f);
    check("mirrored mid-move velocity",
          MotionProfile_Velocity(&n, 0.1f), -MotionProfile_Velocity(&p, 0.1f), 1e-6f);

    printf("\nTEST 5: position is monotonic and never overshoots\n");
    MotionProfile_Init(&p, 90.0f, 200.0f, 1200.0f);
    float prev = -1.0f;
    int monotonic = 1, bounded = 1;
    for (float t = 0.0f; t <= p.t_total + 0.2f; t += 0.001f) {
        float s = MotionProfile_Position(&p, t);
        if (s < prev - 1e-4f) monotonic = 0;
        if (s > 90.0f + 1e-3f) bounded = 0;
        prev = s;
    }
    check_true("never goes backwards", monotonic, "a reversing setpoint would stall the robot");
    check_true("never exceeds the target", bounded, "no commanded overshoot");

    printf("\nTEST 6: degenerate inputs do not produce NaN or hang\n");
    MotionProfile_Init(&p, 0.0f, 200.0f, 1200.0f);
    check("zero move has zero duration", MotionProfile_Duration(&p), 0.0f, 1e-9f);
    check("zero move reports the target at once", MotionProfile_Position(&p, 0.0f), 0.0f, 1e-9f);
    MotionProfile_Init(&p, 90.0f, 0.0f, 1200.0f);
    check_true("zero v_max is not NaN", !isnan(MotionProfile_Position(&p, 1.0f)), "handled");
    MotionProfile_Init(&p, 90.0f, 200.0f, 0.0f);
    check_true("zero accel is not NaN", !isnan(MotionProfile_Position(&p, 1.0f)), "handled");
    check_true("NULL is safe", MotionProfile_Position(0, 1.0f) == 0.0f, "returns 0");

    /* ---- RETARGETING MID-MOVE: rebuild, never translate ----
     *
     * The front-wall alignment moves a move's endpoint once it can see the
     * wall. The obvious implementation adds the correction to the profile's
     * output, and it is wrong: that shifts the ORIGIN by the same amount as
     * the endpoint, so a correction that SHORTENS the move commands the robot
     * backwards before it has gone anywhere. Measured on the robot: a -2.70 cm
     * correction started the reference at -2.58 and the command sat at -45 for
     * 300 ms.
     *
     * Rebuilding from the reference position already reached keeps the
     * reference continuous, which is the property that actually matters --
     * the position loop closes on it, and a step backwards in it is a command
     * to reverse. */
    {
        const float V = 10.0f, A = 20.0f;
        const float original = 19.2f, correction = -2.70f;

        MotionProfile_Init(&p, original, V, A);

        const float t_align  = 0.11f;                        /* as observed */
        const float ref_at   = MotionProfile_Position(&p, t_align);
        const float new_end  = original + correction;

        /* The rejected approach, for the record. */
        const float translated = MotionProfile_Position(&p, 0.0f) + correction;
        check_true("translating the profile does step the reference back",
                   translated < 0.0f, "reproduces the defect");

        /* The rebuild. */
        MotionProfile_t q;
        MotionProfile_Init(&q, new_end - ref_at, V, A);

        check("rebuilt reference starts exactly where it was",
              ref_at + MotionProfile_Position(&q, 0.0f), ref_at, 1e-6f);
        check_true("and never goes backwards from there",
                   ref_at + MotionProfile_Position(&q, 0.01f) >= ref_at,
                   "monotonic");
        check("and still lands on the corrected endpoint",
              ref_at + MotionProfile_Position(&q, MotionProfile_Duration(&q)),
              new_end, 1e-3f);

        /* A correction that LENGTHENS the move must work the same way. */
        MotionProfile_t r;
        const float longer = original + 2.74f;
        MotionProfile_Init(&r, longer - ref_at, V, A);
        check("a lengthening correction lands too",
              ref_at + MotionProfile_Position(&r, MotionProfile_Duration(&r)),
              longer, 1e-3f);
    }

    /* ---- STARTING AT SPEED ----
     *
     * The retarget case. A move already at cruise gets a new endpoint, and the
     * new profile has to pick up the velocity the old reference actually has.
     * Rebuilding from rest instead drops the feedforward to zero mid-move,
     * which commands a brake and a fresh start. */
    {
        const float V = 10.0f, A = 20.0f, v0 = 10.0f, D = 9.6f;
        MotionProfile_t q;

        check_true("a move that fits reports that it fits",
                   MotionProfile_InitFrom(&q, D, v0, V, A) == 1U, "feasible");
        check("it starts at v0, not at rest",
              MotionProfile_Velocity(&q, 0.0f), v0, 1e-4f);
        check("it still ends at rest",
              MotionProfile_Velocity(&q, q.t_total), 0.0f, 1e-4f);
        check("and lands exactly on target",
              MotionProfile_Position(&q, q.t_total), D, 1e-4f);
        check("velocity integrates to position",
              integrate(&q, q.t_total, 1e-6f), D, 0.01f);

        /* Never faster than asked, whatever shape it came out. */
        int over = 0;
        for (float t = 0.0f; t <= q.t_total; t += q.t_total / 500.0f)
            if (MotionProfile_Velocity(&q, t) > V + 1e-3f) over = 1;
        check_true("never exceeds v_max", !over, "within limit");

        /* Starting at speed must be QUICKER than starting from rest. If it is
           not, the v_start term is not reaching the position maths. */
        MotionProfile_t r;
        MotionProfile_Init(&r, D, V, A);
        check_true("and beats the same move started from rest",
                   q.t_total < r.t_total, "faster");
    }

    /* ---- THE RETARGET IS CONTINUOUS IN BOTH POSITION AND VELOCITY ----
     *
     * This is the property the whole thing exists for. A retarget at the
     * halfway point of a cell must not show up as a step in either. */
    {
        const float V = 10.0f, A = 20.0f;
        MotionProfile_t a_p;
        MotionProfile_Init(&a_p, 19.2f, V, A);

        const float t_cut  = a_p.t_total * 0.5f;
        const float pos_at = MotionProfile_Position(&a_p, t_cut);
        const float vel_at = MotionProfile_Velocity(&a_p, t_cut);
        const float new_end = 19.2f - 2.70f;      /* the measured correction */

        MotionProfile_t b_p;
        check_true("a halfway retarget fits",
                   MotionProfile_InitFrom(&b_p, new_end - pos_at, vel_at, V, A)
                   == 1U, "feasible");
        check("position is continuous across the retarget",
              pos_at + MotionProfile_Position(&b_p, 0.0f), pos_at, 1e-5f);
        check("velocity is continuous across the retarget",
              MotionProfile_Velocity(&b_p, 0.0f), vel_at, 1e-4f);
        check("and it lands on the corrected endpoint",
              pos_at + MotionProfile_Position(&b_p, b_p.t_total), new_end, 1e-3f);

        /* Rebuilding from rest is what this replaced: same endpoint, but the
           feedforward collapses. Asserted so the old behaviour cannot return
           quietly. */
        MotionProfile_t c_p;
        MotionProfile_Init(&c_p, new_end - pos_at, V, A);
        check_true("rebuilding from rest does drop the feedforward",
                   MotionProfile_Velocity(&c_p, 0.0f) < 0.5f * vel_at,
                   "reproduces the defect");
    }

    /* ---- NO ROOM TO STOP ----
     *
     * Asked to finish nearer than the braking distance. It must say so, and
     * what it builds must still be sane -- a reference that reverses to make
     * the arithmetic work would drive the robot backwards. */
    {
        const float V = 10.0f, A = 20.0f, v0 = 10.0f;
        const float d_stop = (v0 * v0) / (2.0f * A);      /* 2.5 cm */
        MotionProfile_t q;

        check_true("a move shorter than the braking distance is refused",
                   MotionProfile_InitFrom(&q, 0.5f * d_stop, v0, V, A) == 0U,
                   "reported");
        check("and the profile built instead is the hardest stop",
              MotionProfile_Position(&q, q.t_total), d_stop, 1e-3f);
        check_true("which never runs backwards",
                   MotionProfile_Position(&q, q.t_total * 0.5f) > 0.0f,
                   "monotonic");
        check("starting, as it must, at v0",
              MotionProfile_Velocity(&q, 0.0f), v0, 1e-4f);
    }

    /* A v0 pointing the wrong way cannot be modelled and must be read as rest,
       not as a profile whose velocity contradicts the caller's. */
    {
        MotionProfile_t q;
        (void)MotionProfile_InitFrom(&q, 10.0f, -5.0f, 10.0f, 20.0f);
        check("a v0 opposing the move is treated as rest",
              MotionProfile_Velocity(&q, 0.0f), 0.0f, 1e-6f);
    }

    /* ---------------------------------------------------------------------
     * A MOVE THAT ENDS AT SPEED
     *
     * This is what chained cell motion is built on, and it is the one place a
     * mistake would be invisible on the bench and expensive in the maze: a
     * profile that quietly decelerated to rest anyway would simply look like
     * the robot being slow, while one that overshot its distance would put it
     * past the point it can still stop at before a turn.
     * ------------------------------------------------------------------- */
    printf("\n TEST 8: a segment that hands over still moving\n");
    {
        const float V = 14.0f, A = 28.0f, D = 19.2f;
        MotionProfile_t q;

        check_true("a cruise-to-cruise segment fits",
                   MotionProfile_InitFromTo(&q, D, V, V, V, A) == 1U, "feasible");
        check("it starts at cruise",  MotionProfile_Velocity(&q, 0.0f), V, 1e-4f);
        check("and ENDS at cruise",   MotionProfile_Velocity(&q, q.t_total), V, 1e-4f);
        check("landing exactly on target",
              MotionProfile_Position(&q, q.t_total), D, 1e-3f);
        check("velocity still integrates to position",
              integrate(&q, q.t_total, 1e-5f), D, 0.02f);

        /* No ramps at all: at constant speed the whole segment is cruise, and
           its duration is the distance over the speed and nothing else. */
        check("no acceleration phase", q.t_ramp,  0.0f, 1e-6f);
        check("no deceleration phase", q.t_decel, 0.0f, 1e-6f);
        check("so it takes exactly distance/speed",
              MotionProfile_Duration(&q), D / V, 1e-4f);

        /* And it beats the rest-to-rest move it replaces, which is the entire
           justification for any of this. */
        MotionProfile_t rest;
        MotionProfile_Init(&rest, D, V, A);
        check_true("and beats the same cell driven from rest to rest",
                   MotionProfile_Duration(&q) < MotionProfile_Duration(&rest),
                   "faster");
    }

    printf("\n TEST 9: the two ends of a chained corridor\n");
    {
        const float V = 14.0f, A = 28.0f;
        MotionProfile_t q;

        /* Leaving a cell centre: accelerate to cruise and hand over there. */
        const float lead_in = 19.2f - 5.0f;
        check_true("the first segment fits",
                   MotionProfile_InitFromTo(&q, lead_in, 0.0f, V, V, A) == 1U,
                   "feasible");
        check("starts from rest",     MotionProfile_Velocity(&q, 0.0f), 0.0f, 1e-6f);
        check("hands over at cruise", MotionProfile_Velocity(&q, q.t_total), V, 1e-4f);
        check("it accelerates and never decelerates", q.t_decel, 0.0f, 1e-6f);
        check("lands on target", MotionProfile_Position(&q, q.t_total), lead_in, 1e-3f);

        /* Arriving at one: the braking offset, cruise down to rest. */
        const float lead_out = (V * V) / (2.0f * A) + 1.5f;
        check_true("and the stop that follows a turn request fits",
                   MotionProfile_InitFromTo(&q, lead_out, V, 0.0f, V, A) == 1U,
                   "feasible");
        check("it comes to rest", MotionProfile_Velocity(&q, q.t_total), 0.0f, 1e-6f);
        check("exactly at the cell centre",
              MotionProfile_Position(&q, q.t_total), lead_out, 1e-3f);
        check_true("with room to spare, which is the margin",
                   q.t_cruise > 0.0f, "did not need the whole segment to brake");
    }

    printf("\n TEST 10: a speed change with no room for it is refused\n");
    {
        /* The failure that matters: being asked to reach the cell centre from
           cruise in less than the braking distance. There is no profile for
           it, and inventing one that reverses would be worse than saying so. */
        const float V = 14.0f, A = 28.0f;
        const float d_brake = (V * V) / (2.0f * A);
        MotionProfile_t q;

        check_true("too short to shed the speed is reported",
                   MotionProfile_InitFromTo(&q, 0.5f * d_brake, V, 0.0f, V, A) == 0U,
                   "reported");
        check("and what is built is the hardest stop, not a reversal",
              MotionProfile_Position(&q, q.t_total), d_brake, 1e-3f);
        check_true("which never runs backwards",
                   MotionProfile_Position(&q, q.t_total * 0.5f) > 0.0f, "monotonic");

        /* The mirror: too short to GAIN the speed asked for. */
        check_true("too short to reach the exit speed is reported too",
                   MotionProfile_InitFromTo(&q, 0.5f * d_brake, 0.0f, V, V, A) == 0U,
                   "reported");
        check("and it still ends at the speed it promised",
              MotionProfile_Velocity(&q, q.t_total), V, 1e-4f);

        /* An exit speed pointing backwards is as unmodellable as a v0 that
           does, and must be read the same way. */
        (void)MotionProfile_InitFromTo(&q, 10.0f, 0.0f, -5.0f, 10.0f, 20.0f);
        check("a v_end opposing the move is treated as rest",
              MotionProfile_Velocity(&q, q.t_total), 0.0f, 1e-6f);
    }

    printf("\n===== %s (%d failures) =====\n\n",
           failures ? "FAILURES" : "ALL CHECKS PASSED", failures);
    return failures ? 1 : 0;
}
