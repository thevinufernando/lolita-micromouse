# Host-side tests

Two suites live here. Both compile real firmware sources on the development
machine and run them against simulated data, so they catch maths errors that
on-target testing cannot: on hardware you can only watch the output, whereas
here it can be compared against a known ground truth.

| Suite | Covers | Run after changing |
|---|---|---|
| `ekf_host_test.c` | Yaw EKF (gyro + encoder fusion) | `EKF.c` |
| `tof_filter_host_test.c` | VL53L0X noise filter | `tof_filter.c` |

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
