# Host-side tests

Seven suites live here. Most compile real firmware sources on the development
machine and run them against simulated data, so they catch maths errors that
on-target testing cannot: on hardware you can only watch the output, whereas
here it can be compared against a known ground truth. The rest mirror logic that
cannot be linked on the host because it needs the HAL or the ST API, and pin the
RULE rather than the plumbing.

| Suite | Covers | Run after changing |
|---|---|---|
| `ekf_host_test.c` | Yaw EKF (gyro + encoder fusion) | `EKF.c` |
| `tof_filter_host_test.c` | VL53L0X noise filter | `tof_filter.c` |
| `tof_cache_host_test.c` | Held-reading age gate | `tof_sensors.c`, `TOF_MAX_SAMPLE_AGE_MS` |
| `motion_profile_host_test.c` | Trapezoidal profiles, mid-move retargeting | `motion_profile.c` |
| `maze_map_host_test.c` | Pose, wall bookkeeping, known-vs-open | `maze_map.c` |
| `navigator_host_test.c` | Reactive wall-following rule | `navigator.c` |
| `wall_follow_host_test.c` | Lateral loop and the cascade rule | `wall_follow.c`, the `WALL_FOLLOW_*` constants |
| `floodfill_diff.sh` | **That the ported flood fill IS the original** | anything under `Core/*/Maze/floodfill/` |

> **The maze state moved.** `v_walls`, `h_walls` and the pose are defined by the
> algorithm now (`Core/Src/Maze/floodfill/floodfill_run.c`), not by
> `maze_map.c`, so any test that links the map also has to link the algorithm
> and a stub API. The build lines below already do.

---

## EKF

`ekf_host_test.c` compiles the real `Core/Src/Control/LowLevel/EKF.c` on the
development machine and runs it against simulated sensor data. The EKF has no
hardware dependencies (it includes only `EKF.h`, `control_config.h` and
`math.h`), so it can be exercised without flashing the robot.

This is the only practical way to verify the filter maths. On-target you can
only observe the output; here you can compare it against a known ground truth.

## Running

Any host GCC will do (MSYS2 UCRT64 is what this project was developed with):

```sh
gcc -O1 -o ekf_test.exe tests/ekf_host_test.c Core/Src/Control/LowLevel/EKF.c \
    -I Core/Inc/Control/LowLevel -lm
./ekf_test.exe
```

Exit code is 0 when everything passes, 1 otherwise.

## What it covers

| Test | Checks |
|------|--------|
| 1 | Gyro bias is recovered from a stationary run where the gyro reports rotation but the encoders do not |
| 2 | A clean 90 deg turn produces 90 deg of fused yaw |
| 3 | A 30% wheel-slip scale error is rejected: encoder-only would read 117 deg, fused reads ~90 |
| 4 | Yaw stays continuous past 360 deg and `EKF_NormalizeAngle` wraps correctly |
| 5 | `EKF_Reset` zeroes yaw but preserves the learned gyro bias |
| 6 | Zero, negative, huge and NULL inputs cannot corrupt the state |

Every test also asserts that the covariance matrix stays symmetric and
positive-definite, which is the usual early warning that a Kalman filter has
gone numerically wrong.

## Note on test 3

Test 3 is what drove the `EKF_R_SLIP_COEFF` design. With a fixed measurement
noise the filter tracked the slipping encoders to 100.4 deg, because
innovation gating only rejects sudden outliers and a steady slip ramp does not
look like one. Making R grow with the square of the rotation rate brought that
to 90.45 deg. If you ever set `EKF_R_SLIP_COEFF` to 0, expect this test to
fail - that is the test doing its job, not a bug.

---

## ToF noise filter

`tof_filter_host_test.c` exercises `Core/Src/Sensors/VL53L0X_Driver/tof_filter.c`.
It depends only on `tof_filter.h` and `control_config.h`, so it needs no
hardware and no ST API.

### Running

```sh
gcc -O1 -o tof_filter_test.exe tests/tof_filter_host_test.c     Core/Src/Sensors/VL53L0X_Driver/tof_filter.c     -I Core/Inc/Sensors/VL53L0X_Driver -I Core/Inc/Control/LowLevel -lm
./tof_filter_test.exe
```

The simulated noise band (83-90 mm for a true 80 mm target) is taken from an
actual bench measurement on this robot, so the test reflects the real sensor
rather than an invented noise model.

### What it covers

| Test | Checks |
|------|--------|
| 1 | Jitter is actually reduced (sigma 2.27 mm -> 0.94 mm) and the mean is preserved |
| 2 | The filter does **not** remove bias - it settles at the biased mean, not the truth |
| 3 | A real wall transition (86 -> 250 mm) is passed through immediately, not ramped |
| 4 | Ordinary jitter never trips the jump detector |
| 5 | A lone outlier does not persist beyond one sample |
| 6 | A freshly-reset filter never reports a false close wall |
| 7 | `Invalidate()` re-seeds cleanly without blending across the gap |
| 8 | Zero, huge and NULL inputs cannot corrupt the filter |

### Note on tests 2, 3 and 5

**Test 2 asserts a limitation, deliberately.** Noise and bias are different
problems: no filter can remove a constant offset, because the average of
biased samples is equally biased. It exists so nobody tries to fix a constant
error by tuning filter constants. Bias belongs to `TOF_OFFSET_*_MM`.

**Test 3 caught a real design defect.** The jump detector originally compared
the *median* against the threshold. But a genuine step arrives as one new
value against a window of old ones - `{86, 86, 250}` medians to `86` - so the
median stage suppressed the first sample of every real transition exactly as
if it were an outlier, and the robot stayed blind to an opening for two
further samples. The detector now tests the **raw** sample. Do not "tidy" it
to use the median.

**Test 5 documents the cost of that fix.** A median cannot distinguish a bad
reflection from the first sample of a real change - they are the same shape -
so an outlier larger than the threshold produces one sample of overshoot
before the median pulls it back. The test asserts it does not persist rather
than that it never happens. A one-sample blip is much cheaper than being blind
to a real opening in the maze.


## tof_cache_host_test.c - the held-reading age gate

```sh
gcc -O1 -Wall -Wextra -o tof_cache_test tests/tof_cache_host_test.c \
    -I Core/Inc/Control/LowLevel -lm && ./tof_cache_test
```

Mirrors the decision `ToF_ReadAllLatest()` makes when a free-running sensor has
nothing new yet. The driver cannot run on the host -- it pulls in the ST API and
the HAL -- so what is pinned here is the rule rather than the plumbing.

The rule exists because continuous ranging and a 10 ms control loop do not tick
together: three polls in four legitimately find nothing new. Reporting that as
an invalid measurement is honest and useless, because the wall follower reads
invalid as "no wall" and would drop and re-acquire its reference several times a
second. So the newest good reading is held and served.

The age limit is the other half. Without it a dead sensor is indistinguishable
from one merely between measurements, and its last reading would be served
forever while the robot steers to a wall that is no longer there.

Also pins the unsigned-subtraction idiom against the tick counter's 32-bit wrap,
where a signed comparison would call a brand-new reading ancient.

## maze_map_host_test.c - pose, wall bookkeeping, known-vs-open

```sh
gcc -O1 -Wall -Wextra -o maze_map_test tests/maze_map_host_test.c \
    Core/Src/Maze/maze_map.c Core/Src/Maze/floodfill/maze.c \
    Core/Src/Maze/floodfill/floodfill_run.c tests/floodfill_sim_api.c \
    -DSIM_PROVIDE_DEBUG_LOG -I Core/Inc/Maze -I Core/Inc/Maze/floodfill \
    && ./maze_map_test
```

Covers the direction arithmetic in both directions -- `MazeMap_CellWalls()`
must read back exactly what `MazeMap_UpdateWalls()` wrote, from all four
headings -- and the distinction between a cell with no walls and a cell nobody
has looked at. Getting the first wrong produces a map that is plausible,
self-consistent and mirrored; getting the second wrong makes every unexplored
cell read as wide open.

## floodfill_diff.sh - is the ported algorithm the same algorithm?

```sh
tests/floodfill_diff.sh [path-to-MicroMouseAlgorithm]
```

The one test that matters for the port. It builds `MicroMouseAlgorithm`'s
`maze.c` + `Main.c` into one binary and the copies under
`Core/{Inc,Src}/Maze/floodfill/` into another, links BOTH against the same
simulated maze in `floodfill_sim_api.c`, and compares their transcripts action
for action across seven seeded mazes.

Everything about the two builds is identical except which copy of the algorithm
they contain. So "the algorithm did not change" stops being a claim and becomes
something the build either proves or fails. Run it after touching anything
under `floodfill/`, and if it ever diverges, change
`MicroMouseAlgorithm` first and re-port rather than patching the copy.

It also runs every maze twice, once serving wall readings fresh and once from a
per-cell snapshot. The snapshot is the single adaptation the robot needed -- a
real sensor read costs about 200 ms and the algorithm asks several times per
cell -- and the transcripts must match, or the caching is changing behaviour
rather than just saving time.

The mazes come from a seeded recursive backtracker, so both binaries carve the
identical one and there are plenty of dead ends for `checkDeadEnd()` to find.

## navigator_host_test.c - reactive wall-following rule

```sh
gcc -O1 -Wall -Wextra -o nav_test tests/navigator_host_test.c \
    Core/Src/Maze/maze_map.c Core/Src/Maze/floodfill/maze.c \
    Core/Src/Maze/floodfill/floodfill_run.c tests/floodfill_sim_api.c \
    -DSIM_PROVIDE_DEBUG_LOG -I Core/Inc/Maze -I Core/Inc/Maze/floodfill \
    && ./nav_test
```

Checks the decision rule's truth table, then drives it around a simulated
arena through the real `maze_map.c` and asserts the recorded walls match.

**It caught the defining bug in that rule.** The first version returned "turn
right" as a complete action. The trace showed the robot reaching the opening,
turning into it, seeing another open right, and pivoting straight back down
the corridor it came from without ever entering the new cell. Every action a
wall follower takes must end in one cell of forward motion; the turn only
chooses which way to leave. This is the classic way the rule is written wrong,
and on hardware it would have looked like a turn-tuning problem rather than a
logic error.

The rule itself is copied into the test rather than linked, because
`navigator.c` needs the HAL and cannot build on the host. The constants come
from the real `navigator.h`, so the test also proves that header parses
standalone and that `MazeTrace_t` still satisfies its 40-byte stride assert.
