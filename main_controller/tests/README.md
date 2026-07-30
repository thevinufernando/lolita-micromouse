# Host-side tests

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
