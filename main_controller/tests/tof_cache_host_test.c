/* Host check of the held-reading age gate in ToF_ReadAllLatest().
 *
 * The driver itself cannot run here -- it pulls in the ST API and the HAL --
 * so this mirrors the decision the function makes, the same way
 * wall_follow_host_test.c mirrors the wall follower's arithmetic. What is
 * being pinned is the RULE, not the plumbing:
 *
 *   a fresh reading replaces the held one and restarts its clock
 *   no new reading, held one still young    -> serve it
 *   no new reading, held one too old        -> report invalid, count a drop
 *   no new reading, never had one           -> report invalid, count nothing
 *
 * The middle two are the whole point. Without the age limit a sensor that has
 * died is indistinguishable from one that is merely between measurements, and
 * its last reading would be served forever -- the robot steering to a wall
 * that is no longer there. Without the held reading at all, three polls in
 * four hand the wall follower an invalid measurement and it drops and
 * re-acquires its reference several times a second.
 *
 * Runs on the host, no hardware needed. Exit code 0 = pass. */
#include "control_config.h"
#include <stdio.h>
#include <stdint.h>

static int fails = 0;
#define CHECK(c, m) do { if (!(c)) { printf("  [FAIL] %s\n", m); fails++; } \
                         else       { printf("  [PASS] %s\n", m); } } while (0)

/* The cached state, mirroring s_last / s_last_ms in tof_sensors.c. */
typedef struct {
    uint16_t distance_mm;
    uint8_t  valid;
    uint32_t stamp_ms;
} Held_t;

static uint32_t fresh_count, cached_count, stale_drops;

/* Mirrors one sensor's branch of ToF_ReadAllLatest(). `got` is 1 when the
 * sensor produced a new measurement this poll. Returns the reported distance,
 * or 0 when the output is invalid. */
static uint16_t latest(Held_t *h, int got, uint16_t new_mm,
                       uint32_t now, uint32_t max_age_ms)
{
    if (got) {
        h->distance_mm = new_mm;
        h->valid       = 1U;
        h->stamp_ms    = now;
        fresh_count++;
        return new_mm;
    }

    /* Unsigned subtraction on purpose: it stays correct across the tick
     * counter's 32-bit wrap, where `now < stamp` and a signed comparison
     * would declare a brand-new reading ancient. */
    if (h->valid && (uint32_t)(now - h->stamp_ms) <= max_age_ms) {
        cached_count++;
        return h->distance_mm;
    }

    if (h->valid) {
        h->valid = 0U;
        stale_drops++;
    }

    return 0U;   /* invalid */
}

int main(void)
{
    printf("\nTOF HELD-READING AGE GATE\n");

    /* The constants have to be compatible or the rule is nonsense: a limit at
     * or below the measurement period would age out every ordinary poll. */
    CHECK(TOF_MAX_SAMPLE_AGE_MS > TOF_INTER_MEASUREMENT_MS,
          "the age limit outlasts one measurement period");
    CHECK(TOF_MAX_SAMPLE_AGE_MS < 1000U,
          "and is short enough that a dead sensor is noticed quickly");

    {
        Held_t h = {0U, 0U, 0U};
        fresh_count = cached_count = stale_drops = 0;

        CHECK(latest(&h, 0, 0, 1000U, TOF_MAX_SAMPLE_AGE_MS) == 0U,
              "nothing held yet reports invalid");
        CHECK(stale_drops == 0U,
              "and does not count as a sensor that stopped");

        CHECK(latest(&h, 1, 63U, 1000U, TOF_MAX_SAMPLE_AGE_MS) == 63U,
              "a fresh reading is reported");
        CHECK(fresh_count == 1U, "and counted as fresh");

        /* The normal case: a 10 ms loop against a 40 ms sensor. */
        CHECK(latest(&h, 0, 0, 1010U, TOF_MAX_SAMPLE_AGE_MS) == 63U,
              "polled early, the held reading is served");
        CHECK(latest(&h, 0, 0, 1030U, TOF_MAX_SAMPLE_AGE_MS) == 63U,
              "still served one measurement period later");
        CHECK(cached_count == 2U, "both counted as held, not fresh");
        CHECK(stale_drops == 0U, "and neither raised an alarm");

        /* Right at the boundary it is still good; one past it is not. */
        CHECK(latest(&h, 0, 0, 1000U + TOF_MAX_SAMPLE_AGE_MS,
                     TOF_MAX_SAMPLE_AGE_MS) == 63U,
              "the limit itself is inclusive");
        CHECK(latest(&h, 0, 0, 1001U + TOF_MAX_SAMPLE_AGE_MS,
                     TOF_MAX_SAMPLE_AGE_MS) == 0U,
              "one millisecond past it, the reading goes invalid");
        CHECK(stale_drops == 1U, "and the drop is counted exactly once");

        CHECK(latest(&h, 0, 0, 9999U, TOF_MAX_SAMPLE_AGE_MS) == 0U,
              "a sensor that stays dead keeps reporting invalid");
        CHECK(stale_drops == 1U,
              "but is not counted again -- one drop per reading lost");
    }

    /* A fresh reading must restart the clock, or the cache would expire on a
     * schedule set by the first sample it ever saw. */
    {
        Held_t h = {0U, 0U, 0U};
        fresh_count = cached_count = stale_drops = 0;

        (void)latest(&h, 1, 60U, 1000U, TOF_MAX_SAMPLE_AGE_MS);
        (void)latest(&h, 1, 61U, 1000U + TOF_MAX_SAMPLE_AGE_MS,
                     TOF_MAX_SAMPLE_AGE_MS);
        CHECK(latest(&h, 0, 0, 1000U + TOF_MAX_SAMPLE_AGE_MS + 10U,
                     TOF_MAX_SAMPLE_AGE_MS) == 61U,
              "a new reading restarts the age clock");
        CHECK(stale_drops == 0U, "so nothing expires while samples keep coming");
    }

    /* The tick counter wraps after about 49 days. It is not a situation this
     * robot will meet, but the idiom that survives it is no harder to write
     * than the one that does not, and this is where that gets decided. */
    {
        Held_t h = {0U, 0U, 0U};
        fresh_count = cached_count = stale_drops = 0;

        const uint32_t before_wrap = 0xFFFFFFF0U;

        (void)latest(&h, 1, 62U, before_wrap, TOF_MAX_SAMPLE_AGE_MS);
        CHECK(latest(&h, 0, 0, 0x00000010U, TOF_MAX_SAMPLE_AGE_MS) == 62U,
              "a reading taken across the tick wrap is still young");
        CHECK(stale_drops == 0U, "and is not mistaken for a dead sensor");
    }

    printf("\n===== %s (%d failure%s) =====\n\n",
           fails ? "FAILURES PRESENT" : "ALL CHECKS PASSED",
           fails, fails == 1 ? "" : "s");

    return fails ? 1 : 0;
}
