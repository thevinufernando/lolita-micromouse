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

    printf("\n===== %s (%d failures) =====\n\n",
           failures ? "FAILURES" : "ALL CHECKS PASSED", failures);
    return failures ? 1 : 0;
}
