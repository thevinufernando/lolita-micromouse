# CLAUDE.md — Lolita Micromouse Main Controller

Context for Claude Code sessions working on this repository.

**Keep this file current.** When you change architecture, add a module, change
a units convention, or discover a hardware quirk, update the relevant section
and add a line to [Change log](#change-log). That log is how the next session
knows what happened without re-reading every file.

---

## 1. What this is

Firmware for a differential-drive micromouse robot, built on STM32CubeMX +
HAL. The robot is in **low-level controls bring-up**. Straight-line motion and
pivot turns work; maze solving does not exist yet and is explicitly out of
scope until the motion primitives are trusted.

### Hardware

| Part | Detail |
|------|--------|
| MCU | STM32F405RGTx, Cortex-M4F |
| Clock | HSE → PLL, SYSCLK 96 MHz, **HCLK 48 MHz** (AHB divider /2) |
| Motors | 2× N20 gear motor with quadrature encoder, differential drive |
| Motor driver | DRV8833, IN/IN mode, 4 PWM channels |
| IMU | ICM-42688-P accel + gyro over SPI1. **No magnetometer.** |
| Range | 3× VL53L0X ToF (front/left/right) behind a TCA9548A I2C mux |
| Power | 3S LiPo |

Note HCLK is 48 MHz, not 96. `SystemCoreClock` reflects HCLK and the DWT cycle
counter runs at it. Do not assume 96 MHz when doing cycle math.

### Peripheral map

| Peripheral | Use |
|-----------|-----|
| TIM1 | Right encoder, quadrature (TI12), PA8/PA9 |
| TIM2 | Left encoder, quadrature (TI12), PA0/PA1 |
| TIM3 | Motor PWM ×4, period 4799, PC6–PC9 |
| SPI1 | IMU, master, mode 0, prescaler /2 → **24 MHz** (PA5/6/7 + PA4 CS) |
| I2C1 | TCA9548A mux → 3× VL53L0X, 100 kHz, PB6 SCL / PB7 SDA |
| GPIO | PC3 `MCU_LED`, PB14 `DRV_STBY`, PA4 `IMU_NCS` |
| SWD | PA13/PA14 + PB3 SWO |

**There is no UART.** All observation goes through the ST-Link debugger's
live-watch panel. This is why the code exposes so many `volatile` globals —
they are the only user interface. `.vscode/launch.json` has the watch list
pre-populated.

---

## 2. Build

The STM32Cube VSCode extension builds this. **Do not modify `.vscode/launch.json`
build settings** (`preBuild`, `imagesAndSymbols`) — only the `liveWatch`
expressions array is safe to touch.

To build from a shell, the ARM toolchain is not on PATH by default:

```sh
BUNDLES="$LOCALAPPDATA/STM32Cube/bundles"
export PATH="$BUNDLES/gnu-tools-for-stm32/14.3.1+st.2/bin:$BUNDLES/ninja/1.13.2+st.1/bin:$PATH"
cmake --preset Debug        # only after adding/removing source files
cmake --build --preset Debug
```

Build must be **warning-free**. If you add a test function that only one build
configuration calls, mark it `TEST_FN` (see `main.c`) so `-Wunused-function`
stays meaningful.

### Adding a source file

Two places, both required:
1. `CMakeLists.txt` → `target_sources` and, for a new folder, `target_include_directories`
2. Re-run `cmake --preset Debug`

`cmake/stm32cubemx/CMakeLists.txt` is **CubeMX-generated** — it gets rewritten
on regeneration, so never put user sources there. HAL driver `.c` files do
belong there and CubeMX adds them when you enable a peripheral.

### Host-side tests

`tests/` contains host harnesses for the EKF and the ToF noise filter; see
`tests/README.md`. Run the relevant one after any change to `EKF.c` or
`tof_filter.c` — they catch maths errors that on-target testing cannot, and
the ToF suite has already caught one real design defect (see §9).

---

## 3. Layout

```
Core/Inc, Core/Src
├─ Control/LowLevel/    PID.c  EKF.c  straightline_controller.c  turn_controller.c
│                       control_config.h  ← ALL tuning lives here
├─ Maze/                maze_map.c  wall_sense.c  wall_follow.c
│                       navigator.c           ← reactive exploration (no solver)
├─ Encoders/            encoders.c
├─ Motors/              DRV8833.c
├─ Sensors/ICM-42688-P/ ICM42688.c
├─ Sensors/TCA9548A/    TCA9548A.c            ← I2C mux channel select
├─ Sensors/VL53L0X_Driver/ tof_sensors.c      ← our ToF driver (this is the one to edit)
│                       tof_filter.c          ← range noise filter (host-tested)
├─ Sensors/VL53L0X/     stock ST API — DO NOT EDIT (see §9)
│  ├─ Core/             vl53l0x_api*.c        ← ST, verbatim
│  └─ Platform/         vl53l0x_platform.c    ← ST's file, REWRITTEN for STM32 HAL
├─ Tests/               test_harness.c  ← ACTIVE_TEST + all on-target Test_* routines
└─ Utils/               dwt_timer.c
tests/                  host-side EKF verification (separate from Core/Src/Tests/ above)
```

`main.c` only brings up hardware and calls `TestHarness_RunCycle()` from its
`while(1)` loop; the test selector and every `Test_*` routine live in
`Core/Src/Tests/test_harness.c` / `Core/Inc/Tests/test_harness.h` instead, so
that CubeMX-adjacent file doesn't keep absorbing application logic.

CubeMX owns `main.c`'s generated regions. **All hand-written code in `main.c`
must live inside `/* USER CODE BEGIN x */ … /* USER CODE END x */`** or it is
destroyed on the next regeneration.

---

## 4. Conventions that matter

Get these wrong and the robot drives into a wall.

### Sign conventions

- **Positive yaw = anticlockwise = LEFT turn.** Everything follows this:
  encoder-derived yaw, gyro after `IMU_GYRO_Z_SIGN`, EKF state, PID setpoint.
- Left turn = right wheel forward, left wheel backward.
- `Motor_runSignedSpeed(left, right)` takes **signed** speeds and resolves
  direction per wheel. A pivot is `Motor_runSignedSpeed(-v, +v)`.
- Encoder polarity is fixed in `encoders.c` (`left = +1`, `right = -1`) so both
  counts increase when driving forward.

### Units

| Quantity | Unit | Where |
|----------|------|-------|
| Distance | **cm** | encoders, straight-line PID |
| Straight-line heading | **raw encoder ticks** (`left − right`) | heading PID |
| Turn angle | **degrees** | turn PID setpoint/measurement |
| EKF yaw | **radians, continuous (never wrapped)** | `EKF.c` |
| Motor speed | **0–255 signed** | mapped to 0–4799 PWM |

The turn PID measures **degrees of fused yaw**. It used to measure
differential wheel arc in cm — gains from before that change are not
comparable. Conversion: `K_deg = K_arc × (π·wheelbase/360) = K_arc × 0.09774`.

EKF yaw is deliberately **not** wrapped to [−π, π]. A wrapped state forces
wrapped innovations and produces a filter that locks up at the boundary during
360° turns. Use `EKF_NormalizeAngle()` only for display.

---

## 5. Control architecture

```
                 ┌──────────────┐
   encoders ────►│              │
                 │   EKF (yaw,  │──► fused yaw ──► turn PID ──► motors
   gyro Z ──────►│  gyro bias)  │
                 └──────────────┘

   encoders ────► distance PID ─┐
                                ├──► motors     (straight line, no IMU)
   encoders ────► heading  PID ─┘
```

- **Straight line is encoder-only by design.** Deliberate decision, do not add
  the IMU to it without being asked.
- **Turns are IMU + encoder fused.**

### Timing

`turn_controller.c` is multi-rate:

- **~1 kHz** EKF prediction — polls gyro Z only (2-byte SPI read), integrates
  rate with true elapsed time from the DWT counter. Rate-limited to
  `IMU_PREDICT_PERIOD_US` because the gyro ODR is 1 kHz; polling faster would
  integrate the same sample twice and inflate the rotation.
- **100 Hz** encoder correction + PID + motor output. The PID's `T` is fixed,
  so this rate must stay constant.

Both `runForwardDistance()`/`turnLeftAngle()` etc. are **blocking** — they spin
until the move completes or `CONTROL_MOVE_TIMEOUT_MS` fires. They return
`1` on success, `0` on timeout. Check the return value.

### The EKF

2 states: `[yaw, gyro_bias]`. Gyro is the control input, encoder-derived yaw is
the measurement. Full derivation is in the header comment of `EKF.h`.

The models are linear, so it is mathematically a Kalman filter; it is written
in EKF form (explicit Jacobians, separate predict/update) so a nonlinear state
can be added later without restructuring.

Three things in it are load-bearing and should not be "simplified" away:

1. **Joseph-form covariance update.** Keeps P symmetric and positive-definite
   over the thousands of updates per turn that the short form degrades on.
2. **Rate-adaptive measurement noise**
   (`R = R_encoder + R_slip_coeff · rate²`). Wheel slip is systematic and grows
   with spin rate. Innovation gating alone does **not** catch it — a steady
   slip ramp is not an outlier. With fixed R the host test's 30%-slip case
   tracked to 100.4°; with adaptive R it reads 90.45°.
3. **Bias survives `EKF_Reset()`.** Yaw resets each turn; the bias belongs to
   the sensor and stays valid.

---

## 6. Tuning and bring-up

**Every tunable constant is in `Core/Inc/Control/LowLevel/control_config.h`.**
There are no magic numbers scattered in the controllers. Change values there,
rebuild, flash.

`Core/Inc/Tests/test_harness.h` has an `ACTIVE_TEST` switch selecting one of
15 test routines (implemented in `Core/Src/Tests/test_harness.c`). Set it,
rebuild, flash, and read results in live-watch.

| # | Test | Purpose |
|---|------|---------|
| 0 | `TEST_MOTORS_OPEN_LOOP` | Motor wiring + encoder polarity, no PID |
| 1 | `TEST_ENCODERS_ONLY` | Push by hand; verifies distance calibration |
| 2 | `TEST_STRAIGHT_FORWARD` | Tune `STRAIGHT_DIST_*` and `STRAIGHT_HEADING_*` |
| 3 | `TEST_STRAIGHT_FWD_BACK` | Should return to start |
| 4/5 | `TEST_TURN_LEFT_90` / `RIGHT_90` | Tune `TURN_*` |
| 6 | `TEST_TURN_360` | 4×90°; magnifies per-turn bias |
| 7 | `TEST_SQUARE` | Both controllers combined |
| 8 | `TEST_IMU_RAW` | **Run first after IMU wiring** — gyro sign check |
| 9 | `TEST_YAW_ESTIMATE` | Rotate by hand, watch fusion |
| 10 | `TEST_GYRO_BIAS` | Bias + stationary drift measurement |
| 11 | `TEST_TOF_SINGLE` | **Run first after ToF wiring** — mux + single-shot ranging |
| 12 | `TEST_TOF_CONTINUOUS` | Free-running ranging, same distances |
| 13 | `TEST_TOF_MODE_CYCLE` | Mode switching and the stop path |
| 14 | `TEST_MAZE_RUN` | **Moves.** Reactive navigation, wall map, one run |

### Bring-up order for the IMU

1. `TEST_IMU_RAW`. Confirm `tm_imu_ok == 1`. Then rotate the robot
   **anticlockwise** by hand and confirm `tm_gyro_z_dps` is **positive**. If
   negative, set `IMU_GYRO_Z_SIGN` to `-1.0f`. Everything downstream depends
   on this.
2. `TEST_GYRO_BIAS`. `tm_bias_drift_deg` under ~1°/10 s is healthy.
3. `TEST_YAW_ESTIMATE`. Rotate 90° by hand; `tm_yaw_deg` ≈ +90.
4. `TEST_TURN_LEFT_90`, then `TEST_TURN_360`.

### Calibration vs gains

If turns are off by a consistent **ratio**, fix `ROBOT_WHEEL_BASE_CM`, not the
PID gains. Overshoot ⇒ the configured wheel base is too small. Same logic for
`WHEEL_DIAMETER_MM` / gear ratios in `encoders.h` for straight-line distance.

### Startup requirements

`TurnController_Init()` runs a stationary gyro bias calibration, so **the robot
must be still and level at power-on.** It rejects the calibration if it detects
motion above `IMU_GYRO_BIAS_MAX_DPS`.

LED at boot: **3 slow blinks = IMU up**, **6 fast blinks = IMU not found,
running encoder-only**. The turn controller degrades gracefully rather than
failing, so a silent fallback is possible — check `tm_imu_ok`.

---

## 7. Hardware quirks and gotchas

- **CubeMX drives `IMU_NCS` LOW in `MX_GPIO_Init()`.** SPI chip select must
  idle HIGH. `main.c` deasserts it in `USER CODE BEGIN 2` before touching the
  IMU. If CubeMX is regenerated and IMU reads break, check this first.
- **CubeMX does not start the timers.** `HAL_TIM_Encoder_Start` (TIM1, TIM2)
  and `HAL_TIM_PWM_Start` (TIM3 ×4) are called in `USER CODE BEGIN 2`. Without
  them nothing moves and no encoder counts.
- **SPI1 runs at 24 MHz**, which is the ICM-42688-P's maximum. If IMU reads are
  intermittent, drop the prescaler to /4 (12 MHz) in CubeMX.
- **ICM-42688-P register order** is TEMP(0x1D) → ACCEL(0x1F) → GYRO(0x25).
  Burst reads for temperature must start at `TEMP_DATA1`, not `ACCEL_DATA_X1`.
- **DWT counter wraps every ~89.5 s** at 48 MHz. Always measure with
  `DWT_ElapsedUs(start)` (wrap-safe unsigned subtraction), never by comparing
  absolute `DWT_Micros()` values.
- **Motor "stop" is a brake**, not coast — both PWM pins driven high.
- `CONTROL_MIN_MOVE_SPEED` boosts small commands past gearbox stiction. If the
  robot creeps instead of stopping cleanly, this is the knob.

---

## 8. Scope boundaries

Currently **out of scope** unless explicitly requested:

- **Any goal-seeking maze solver** (flood fill, DFS, …). The robot explores by
  wall following, which visits cells but does not aim at anything. Porting the
  flood fill from `MicroMouseAlgorithm` replaces `decide()` in
  `Maze/navigator.c` and nothing underneath it.
- Magnetometer — **not present in hardware**, so absolute heading is
  impossible. Yaw is always relative to the last reset. The wall follower's
  standing tilt is the closest thing to an absolute reference: holding a wall
  at its setpoint while the gyro drifts requires a persistent tilt, so that
  tilt *is* the drift, and it is bled back into the heading target.

**No longer out of scope**, as of 2026-09-12 — earlier revisions of this file
said all four were deliberately absent:

- Wall detection — `Maze/wall_sense.c` turns distances into booleans.
- ToF in the straight-line controller — `runForwardFused()` centres on
  whichever side wall is in range.
- IMU in the straight-line controller — it holds fused heading, not encoder
  tick difference.
- Reactive navigation — `Maze/navigator.c` chooses each move from live sensor
  readings.

---

## 9. ToF ranging (VL53L0X + TCA9548A)

Three VL53L0X sensors (front, left, right) provide distance readings for wall
detection. **Distances only** — see §8 for what is deliberately absent.

### The ST API is vendor code

Everything under `Sensors/VL53L0X/` is ST's official API (STSW-IMG005 v1.0.4),
included verbatim so a future ST release is a drop-in replacement. **Do not
edit it**, and do not reformat it to match house style.

The one exception is `Platform/vl53l0x_platform.c`. ST ships that file as a
Win32 reference port that drives a Nucleo over a COM port via
`ranging_sensor_comms.dll`; it cannot build for this target. It has been
replaced wholesale with a direct STM32 HAL I2C implementation. If the API is
ever updated, this is the only file that needs re-porting.

Three other files from ST's `Platform/` folder are Win32-only and are
**deliberately not in `CMakeLists.txt`**: `vl53l0x_i2c_platform.c`,
`vl53l0x_i2c_win_serial_comms.c` and `vl53l0x_platform_log.c`. Adding them
breaks the build. Their declarations are bypassed by the HAL port.

Write application code against `tof_sensors.h`, never against the ST API
directly — the wrapper is what guarantees the mux is on the right channel.

### The mux

All VL53L0X parts share factory address `0x29`, so each sits on its own
TCA9548A channel and the MCU opens exactly one at a time. The alternative —
reassigning addresses at boot — needs one XSHUT GPIO per sensor and must be
redone on every power cycle, so it was not used.

Consequences worth knowing:

- **Every access is channel-scoped.** Each `ToF_*` entry point selects the
  channel before touching a sensor. This is also why the ST API must not be
  called directly: an API call on the wrong channel silently talks to a
  different sensor at the same address and returns a perfectly plausible
  number.
- **A swapped channel mapping is nearly invisible.** `TOF_CHANNEL_*` in
  `control_config.h` must match the Main PCB. Get it wrong and every reading
  is valid but attributed to the wrong direction.
- The mux driver caches the active channel, so re-selecting the same one
  costs nothing. `TCA9548A_DisableAll()` parks the bus.

### Accuracy: noise and bias are separate problems

This trips people up, so it is worth being explicit. The pipeline is:

```
raw from sensor  ──►  + TOF_OFFSET_*_MM  ──►  filter  ──►  distance_mm
                          (fixes bias)      (fixes noise)
```

**Noise** is random spread between consecutive readings of a stationary
target. Measured on this robot: a wall at a true 80 mm reads 83–90 mm.
Handled by `tof_filter.c`.

**Bias** is a consistent over- or under-read — in that same measurement, every
sample was high. **Filtering cannot fix this.** The average of biased samples
is equally biased; you get a beautifully stable wrong number. Handled by the
per-sensor `TOF_OFFSET_*_MM` constants.

> If readings are stable but wrong → adjust the offsets.
> If they are centred but jumpy → adjust the filter.
> Never try to cancel a constant error by tuning filter constants.

Offsets are per-sensor because bias comes from cover glass, mounting depth and
the module's own calibration. Measure each with `TEST_TOF_SINGLE` against a
ruler at a distance you actually care about (80–100 mm for maze walls) and set
the offset to `(true − measured)`. VL53L0X error is not perfectly constant with
range, so a single offset is a linear fix to a mildly nonlinear problem —
calibrate near the distance you drive at.

### The noise filter

Two stages, each aimed at a different failure mode (full rationale in
`tof_filter.h`):

1. **Median** over the last 3 raw samples — rejects the occasional wild
   outlier from a bad reflection. A minority of arbitrarily-wrong values
   cannot move a median at all.
2. **EMA** over the median output — smooths the dense small jitter.

A plain moving average was rejected: its lag would feed a wall-following
controller a stale distance, which is a textbook route to oscillation, and the
lag gets worse exactly when you lengthen the window to reduce noise.

Measured on the host harness against the real noise band: **σ 2.27 mm → 0.94 mm**
(2.4× reduction) with the mean preserved.

**The jump detector is the part that matters for maze solving.** Smoothing
assumes the signal is roughly constant, and that assumption breaks at the most
important moment — when a side wall ends and the reading legitimately jumps
from ~80 mm to ~250 mm. That jump is *signal*, not noise; an EMA would ramp
across it and the robot would believe in a wall that is no longer there for the
whole ramp. So a step larger than `TOF_FILTER_JUMP_THRESHOLD_MM` snaps the
filter to the new value instead.

⚠️ **The jump detector tests the RAW sample, not the median — do not "tidy"
this.** The host test caught the defect: a real transition arrives as one new
value against a window of old ones, so `{86, 86, 250}` medians to `86` and the
median stage suppresses the first sample of every genuine step exactly as if it
were an outlier. The cost of testing raw is one sample of overshoot on a large
lone outlier, which the median then pulls back — much cheaper than being blind
to an opening.

Call `ToF_ResetFilter()` after anything that breaks continuity between
readings, such as a pivot turn that leaves a sensor facing a different wall.

### Units

Distances are **millimetres**, unsigned — the ST API's native unit. Note the
motion controllers use **cm**; converting is the caller's job.

`ToF_Measurement_t` carries both `distance_mm` (filtered — use this) and
`raw_mm` (offset-corrected, unfiltered — for diagnosis). Both include the
offset.

An invalid reading is `TOF_DISTANCE_INVALID` (0xFFFF), never a stale value, so
ignoring a return code yields an obviously-wrong number rather than a
plausible old one. `TOF_ERROR_RANGE` means the sensor answered but the
measurement is unusable (usually nothing in range) — a normal condition, not a
fault. The raw ST `RangeStatus` is kept in the measurement struct for
diagnosis.

### Modes

Both are exposed because they suit different phases:

| Mode | Call | Use |
|---|---|---|
| Single | `ToF_ReadSingle()` | Blocks ~30 ms per read. Bring-up, stationary checks. |
| Continuous | `ToF_StartContinuous()` then `ToF_ReadContinuous()` | Sensor free-runs; reads are cheap and bounded. What a moving robot wants. |

Continuous uses `CONTINUOUS_TIMED_RANGING`, not back-to-back: back-to-back
pins the sensor at full duty and floods the bus when the consumer reads slower
than the sensor produces. `TOF_INTER_MEASUREMENT_MS` sets the pace.

`ToF_ReadContinuous(..., wait_for_new = 0)` is the non-blocking form — it
returns `TOF_ERROR_TIMEOUT` when no new sample has landed yet. In a loop faster
than the sensor that is expected, not an error.

### Init and failure behaviour

`ToF_Init()` runs the full ST sequence per sensor — `DataInit`, `StaticInit`,
`PerformRefSpadManagement`, `PerformRefCalibration` — then applies the profile
from `control_config.h`. Budget ~50–100 ms per sensor at boot.

Two ordering rules inside that sequence are load-bearing:

1. `DataInit` → `StaticInit` → reference calibrations. Mandated by the API.
2. **VCSEL periods before the timing budget.** Changing a VCSEL period changes
   how long a measurement takes, and the API recomputes the budget against the
   current periods — set the budget first and it is silently readjusted.

Reference calibration runs at boot rather than loading stored constants,
because these sensors have not been characterised on this chassis yet. Once
they have, caching the results would remove most of that boot cost.

Like the IMU, **failure is not fatal**: a bad sensor is marked not-ready and
skipped by every later call (so it cannot stall a loop with repeated I2C
timeouts), while the healthy ones stay usable. `ToF_Init()` still returns
`TOF_ERROR` if any sensor failed — use `ToF_IsSensorReady()` to find out which.
**2 fast LED blinks at boot** flags a ToF init failure.

### Bring-up order

1. `TEST_TOF_SINGLE` (test 11). Check `tm_tof_ready` **first**: bit0 front,
   bit1 left, bit2 right. An all-zero mask means the *mux* never answered —
   that is a wiring or address problem, not a sensor problem.
2. With `tm_tof_ready == 0x07`, hold a wall at a known distance in front of
   each sensor and check `tm_tof_front_mm` / `_left_mm` / `_right_mm` against
   a ruler. Confirm each sensor responds to the direction it is named for —
   this is the check that catches a swapped `TOF_CHANNEL_*` mapping.
3. **Calibrate the offsets.** With a target at a known distance, compare the
   settled `tm_tof_*_mm` against a ruler and set `TOF_OFFSET_*_MM` to
   `(true − measured)`. Do this per sensor; do it before judging the filter.
4. **Check the filter is working.** Watch `tm_tof_*_raw_mm` next to
   `tm_tof_*_mm`: raw should visibly jitter while filtered sits still. If both
   jitter equally the filter is not engaging.
5. **Check `tm_tof_*_jumps` does not climb while the robot is stationary.** If
   it does, `TOF_FILTER_JUMP_THRESHOLD_MM` is below the actual noise floor and
   the smoothing is being defeated — raise it.
6. `TEST_TOF_CONTINUOUS` (test 12). Same distances, free-running.
   `tm_tof_error_count` rising while `tm_tof_sample_count` stays static is the
   real fault signal; both rising together just means polling outpaced the
   sensor.

### Tuning

All in `control_config.h` under the ToF section. The master knob is
`TOF_TIMING_BUDGET_US` (speed vs. accuracy) — for a moving micromouse, sample
rate matters more than the last millimetre, so lower this before touching
anything else. `TOF_VCSEL_PERIOD_*` are left at ST's defaults deliberately:
maze walls are under 20 cm away, and buying range the robot will never use
costs ambient-light immunity.

Filter knobs:

| Constant | Effect |
|---|---|
| `TOF_FILTER_EMA_ALPHA` | Smoothing vs. lag. Lower = smoother/slower. 1.0 disables. Governs small jitter only — large steps bypass it. |
| `TOF_FILTER_JUMP_THRESHOLD_MM` | Step size treated as real. Must sit above the noise spread (~7 mm) and below the smallest real transition (~100 mm). |
| `TOF_OFFSET_*_MM` | Per-sensor bias. Not a filter knob — see the accuracy note above. |

After changing `tof_filter.c`, run the host suite (`tests/README.md`) —
several of its cases encode design decisions that are easy to undo by
accident.

---

## Change log

### 2026-09-12 (newest) - The lean never got built, and the last 5 cm were blind

Two measurements finally separated what had been one confused symptom.

**The front-wall stop spread is 16 to 83 mm against a 75 mm target**, across
nine walled stops, every one of which DID align. So the alignment fires and the
robot still stops anywhere across 67 mm. The cause is that the alignment
happens on the long segment, which ends a braking offset short of the cell
centre, and the five centimetres that follow were pure odometry with the
front-wall correction explicitly disabled. Every bit of slip, carried residual
and arrival slop in those five centimetres landed straight in the gap.

**The stop segment now aligns too, against the real 75 mm target.** The old
reasoning -- that the long segment already applied the correction and a second
would double it -- was wrong. The long segment's alignment places the DECISION
POINT; this is a second and much better look, with the wall going from 125 mm
to 75, which is the closest and most accurate reading the front sensor ever
gets. It needs room to decelerate into, so `CELL_DECISION_MARGIN_CM` goes
1.5 -> 3.0 and the stop segment is 6.5 cm rather than 5.0. The in-flight wall
window shrinks from 8.6 cm of travel to 7.1, still about 13 rotations.

**And the wall follower was not failing to correct, it was never getting to
act.** Pairing each move's exit error against its entry error splits cleanly by
segment type:

```
segments that CONTINUE at speed (lean kept)      error moves 24 to 38 mm
segments that START from rest   (lean zeroed)    error moves 1 mm
```

Ten cells in a row entered at -22 and left at -22, entered at 23 and left at
23. A pivot resets the tilt to zero, so every move after a turn rebuilds its
lean from nothing, and at `WALL_FOLLOW_TILT_SLEW_DPS` of 15 with a 33 ms sweep
that is half a degree per update -- 0.4 s to reach 6 degrees, on a 1.26 s move
whose first 0.5 s is spent accelerating.

`WALL_FOLLOW_TILT_SLEW_DPS` 15 -> 20, with `STRAIGHT_YAW_LIMIT` 80 -> 88 to pay
for it. **30 was the first attempt and the host test refused it, correctly.**
The ramp cost as a fraction of the inner loop's linear range is
`R * TURN_FF_GAIN / STRAIGHT_YAW_LIMIT` -- the proportional gain cancels, so no
amount of retuning `STRAIGHT_YAW_KP` buys any of it back, and 30 against a
limit of 80 spends 75% of the range before the tilt asks for anything. 20
against 88 is 5.0 degrees of 11.0, which keeps the margin. 88 is itself the
most that coexists with cruise: feedforward at 14 cm/s is 112 units and
112 + 88 is exactly `CONTROL_MAX_SPEED`.

**A real ordering bug, found while reading that path.** `CellMotion_Forward()`
installed the wall follower's cell context and then called the move, whose
`WallFollow_Reset()` promptly cleared it. So every move starting from rest --
after a pivot, most of them -- ran with no map veto and a crossing distance of
zero, which told the follower its side sensors were already looking into the
next cell from the first millimetre. The context now travels WITH the move, in
`StraightMove_t.cells`, and is applied after the reset, which is the only
ordering that cannot go wrong.

Also: a cell the robot chained straight through has no stop of its own, and was
reporting the previous cell's. Two rows of the trace carried the same number
and a cell with no wall ahead appeared to have stopped 60 mm from one.

### 2026-09-12 (previous) - Committing to a stop is not the same as stopping driving

The user's observation was that the front-wall gap varies noticeably from cell
to cell, and that a cell which stopped short put the robot into a post. The
cause is one line, and the exit-error measurement added last time is what made
it findable.

**The last 15 mm of every move was a coast.** `arriving` latched the moment the
robot first came within `DISTANCE_TOLERANCE_CM` and the command was zeroed from
there. The profile already plans a deceleration that reaches zero exactly at
the target; releasing the drive 15 mm early throws that plan away and lets
friction finish the move instead. How far a coast carries depends on the speed
the robot happened to have when it crossed the band, and that varies with how
much it was lagging -- 6 to 9 cm/s in the logs, which on this chassis is 12 to
20 mm of roll. So the robot stopped anywhere across about a centimetre, with a
front wall right there to make it obvious.

The command is now CLAMPED AGAINST THE DIRECTION OF TRAVEL rather than removed.
That is what the latch was always for: refusing to drive backwards into a band
the robot has passed, since reversing is the one direction this chassis has no
lateral sensing for. Driving forward to a target it has not yet reached was
never the thing to prevent.

No stiction floor in that branch, deliberately. The floor exists to raise a
command too small to move the robot, and at the end of a stop a command too
small to move the robot is the correct answer -- flooring it would put back the
overshoot this removes.

**Narrowing the tolerance instead would have been a trap**, and it is worth
writing down why. `short_of_it`, which arms the breakaway, uses the same
constant. Tighten the completion band alone and a robot resting 5 mm short
neither completes nor gets a pulse, and the move runs to the 8 s timeout.
Tighten both and every cell ends with a full-scale breakaway lurch. The band is
not the problem; what happens inside it was.

**`stop_front_mm` now records the outcome.** `align_delta_cm` says how far the
endpoint was moved and `F_mm` says what the sensor predicted on the way in, but
nothing said where the robot actually came to rest -- which is the only thing
that decides whether the pivot happens at the cell centre. It is read from the
front sensor at rest, after the settle, and the reader prints the spread. That
spread is where every subsequent move begins.

`MazeTrace_t` is 56 bytes. 64 records is 3584 bytes of a 128 KB part.

From the run itself, the alignment is now working and the lateral loop is not:

```
front alignment     11 of 19 cells, mean -1.02 cm
reasons             fired x11, no wall x5, too far x3
entry lateral error worst -40 mm, 9 of 17 cells over 15 mm
```

All three "too far" cells read about 265 mm predicted at the centre, which is a
wall two cells away being correctly refused. The reason codes are earning their
place.

**And the exit errors say the wall follower is doing almost nothing.** On
eleven of seventeen moves the lateral error at the end matched the error at the
start to within 2 mm -- entered at -19 and left at -19, entered at 25 and left
at 25, entered at 33 and left at 34. A whole cell of travel with no correction
at all. That is the next thing to chase, and it is a different problem from the
alignment.

### 2026-09-12 (previous) - The gate was inside the wall cluster

The alignment fixes worked. It fired on 10 of 17 cells against 1 of 19, mean
correction +0.54 cm rather than a single 3.5 cm lunge, and the reasons are
honest now: five cells had no wall in range, two had one too far to use. The
longitudinal axis is no longer the problem.

**The robot still clipped a post, and the trace names the cycle.** On the last
move, 7.1 cm in, fused yaw jumped from -91.5 to -85.1 in about 100 ms and to
-80.3 in the next -- eleven degrees anticlockwise in 200 ms, with the encoder
distance spiking to 19 cm/s against a 14 cm/s cruise. That is a body pivoting
about a contact point with the far wheel running free, at exactly the travel
where the post sits between the cell being left and the one being entered.

**It went in blind.** `err_mm` reads 0.0 for the first 830 ms of that move --
the cell it was crossing has no side walls at all, so the follower had nothing
to hold. The error was not accumulated there; the move STARTED 34 mm out, and
so did the one before it.

**`WALL_FOLLOW_USABLE_MAX_MM` 95 -> 110, and the run's own readings are the
argument.** The side sensors returned 33, 41, 47, 51, 54, 56, 60, 64, 67, 68,
70, 83, 84, 88, 93, 96 and 97 for walls, and 192, 229, 498, 534 and 575 for
openings. There is a clean gap between 97 and 192 -- and the gate sat at 95,
INSIDE the wall cluster, discarding the 96 and the 97.

That is the worst possible place to go blind. A reading near the gate means a
large lateral error, which is when the correction matters most. And because the
gate is also the far end of the confidence ramp, a reading that did survive at
93 mm was worth only 0.30 of full gain, capping the lean at 3 degrees against an
error asking for 15 -- about 10 mm of correction per cell, while the robot was
entering cells 25 to 38 mm out. At 110 the same reading is worth 0.52 and may
lean 5.2 degrees.

**And the measurement that should have existed three runs ago.** `entry_err_mm`
says what a move started with; nothing said what the previous one ENDED with,
so "the pivot throws the robot sideways" has been inference every time. A move
that ends centred followed by one that starts 25 mm out convicts the pivot; a
move that ends 25 mm out convicts the move. Those want opposite fixes.

`exit_err_mm` is the last lateral reading a move had a reference for.
`MazeTrace_t` is 52 bytes for it, which is what the stride assert is for, and
the reader now prints the mean and worst jump across each cell boundary. 64
records at 52 bytes is 3328 bytes of a 128 KB part.

One test was quietly not testing anything. `wall_follow_host_test.c` checked
that "a reading at the edge of usable range stays timid" using a hard-coded
94 mm, written when the gate was 95 -- it stopped testing the edge the moment
the gate moved. It is expressed at the gate now, plus two new cases pinning the
97-versus-192 separation the change rests on.

### 2026-09-12 (earlier) - "Late" is not "in trouble"

Second run with the reason codes, and they earned their place immediately --
though the first thing they proved was that they were lying.

**Fourteen of nineteen cells reported the alignment as FIRED while exactly one
correction had been applied.** `CellMotion_Record()` read `sl_align_reason`
live, and a cell that ends in a turn runs a SECOND segment to come to rest
before the record is written. That segment deliberately does no alignment, so
it reset the reason on the way past. Same class of bug as the in-flight walls,
same fix: the outcome is latched in `cell_motion.c` the moment the forward
returns, along with the delta and the applied flag. A log that confidently
reports the opposite of what happened is worse than one that reports nothing.

**The five honest cells said the new gate was refusing good chances.** Four of
them declined with the profile already over -- and that gate was mine, added
last time to stop the alignment firing on a wedged robot. It conflated two
different things. The robot routinely lags its reference by several
centimetres, so the profile finishes while the robot is still travelling at
cruise with the front wall at the best reading it will ever give. Refusing
there throws away the single best chance of every move.

`stall_since_ms` is the honest test and already existed: non-zero only while
the robot is commanded above the stiction floor and going nowhere. That is what
"in trouble" means; "past the end of the plan" is just late.

**And the room test was pairing the robot's distance with the reference's
speed.** `has_room` asks whether the ROBOT can stop in the distance IT has
left, so the momentum in that question is the robot's. Using `ref_vel` was
inconsistent in both directions: it demanded room the robot did not need early
in a move, when the reference was still ramping, and then demanded none at all
once the profile ended -- which is exactly when the robot is still at cruise.
It now brakes from a lightly filtered measured speed, seeded from the segment's
entry speed so a chained continuation does not begin by believing it is
stationary.

Where the run actually stands, with the alignment still barely working:

```
walls read in flight   17 of 19      longest think gap   3 ms
worst loop cycle       12 ms         straight cell       1.31 s
entry lateral error    worst -53 mm, 9 of 17 cells over 15 mm
gyro vs encoder yaw    43 deg apart, 234 rejected updates
```

**The wall it clipped is a lateral failure, not a longitudinal one.** Entry
error after a pivot reached -53 mm in a 124 mm corridor, which puts a corner of
the chassis into the wall line. The chain is: the alignment does not fire, so
the robot pivots off the cell centre, and a pivot converts longitudinal offset
into lateral offset almost one for one. Cells 15 and 16 show it happening in an
open region with no side walls at all -- error went from -24 mm to -53 mm
across one pivot and one cell, with nothing able to correct it.

### 2026-09-12 (previous) - The alignment window closed when the speed went up

Chaining worked. 89% of wall readings came free from the in-flight vote, the
longest the motors ran open-loop waiting for the solver was 3 ms against the
107 ms the margin buys, no loop cycle exceeded 13 ms, and a straight cell took
1.39 s against 3.5 before. The run ended wedged at (4,7) after 19 cells.

**The front-wall alignment fired once in nineteen cells, and that once was
wrong.** Three separate faults, all found from that one number.

**It fired on a move that had already failed.** The room test scales the
braking requirement by the REFERENCE velocity, so once the profile runs out
that term is zero and the requirement collapses to
`WALL_FRONT_ALIGN_ROOM_CM` alone. The gate therefore springs open on exactly
the moves that are going badly. The wedged move sat grinding at 4.7 cm/s
against a profile asking 14, long past the end of its plan, and the alignment
chose that moment to extend the target by another 3.54 cm. It is now gated on
the profile still running: retargeting rebuilds a plan, and there is no plan
left to rebuild.

**`WALL_FRONT_ALIGN_BEST_MM` was an absolute distance and should have been a
margin.** The window in which the alignment may fire is bounded below by the
room it needs to stop and above by the reading it is willing to wait for. A
chained segment ends a braking offset short of the cell centre, which moves its
target from 75 mm to 125 mm -- so the lower bound followed the endpoint while
the upper bound stayed pinned to the sensor, and the window narrowed from 85 mm
of travel to 25. It is `WALL_FRONT_ALIGN_BEST_MARGIN_MM` now, 125 mm above
whatever the move is actually aiming at, which reproduces the old behaviour
exactly at the old target.

**A rebuilt profile dropped the exit speed.** The retarget called
`MotionProfile_InitFrom()`, which always ends at rest, so an alignment firing
on a chained segment would have braked the robot to a stop at the decision
point -- and the stop segment that followed would have built its feedforward
believing it started at cruise. Rebuilding may change WHERE a segment ends,
never HOW it ends.

**And the reason it declined is now recorded per cell.** `SL_ALIGN_*` says
which of the six tests stopped it, packed into the three spare bits above the
vote tallies because `MazeTrace_t` has no padding left and its stride is
load-bearing. This run could not distinguish "an arena with nothing to align
against" from "a window that has closed", and those want opposite responses --
the answer turned out to be both, and it took an hour of inference.

Two smaller things from the same log:

- **The vote tallies did not rotate with the walls.** A cell entered and then
  turned in showed `0 0 1` beside `5/1/5`, because `rotateCell()` in the shim
  turned the flags and the distances and left the counts behind. Telemetry
  only, but a log that contradicts itself is worse than one that says nothing.
- **Entry lateral error got worse, not better**: worst -40 mm with 9 of 16
  cells over 15 mm, against 6 of 49 before. That is the next thing to look at
  and it is not an alignment problem -- the alignment fixes the longitudinal
  axis, and a cell that STARTS 40 mm off centre was placed there by the turn
  before it.

### 2026-09-12 (earlier) - The robot stops being stopped

A 51-cell run took 3.5 s per cell. Of that, 0.8 s was `NAV_SETTLE_MS` standing
still on purpose, 0.3 s was a five-vote wall read standing still to look, and
1.0 s of the 2.4 s of driving was ramping to and from a speed held for barely a
second. A third of every cell was spent not moving, and most of the rest was
spent changing speed.

**`MAZE_CONTINUOUS_CELLS` makes a forward end at cruise instead of at rest.**
It stops driving `CELL_DECISION_OFFSET_CM` short of the cell centre -- the last
point from which the robot can still stop AT the centre -- and returns with the
robot rolling. Whatever the solver decides next is still available: another
forward simply continues, and a turn is preceded by a stop segment that drives
the remaining offset. A chained cell is one cell pitch at cruise, 1.37 s, with
no ramps in it at all.

The solver is an ordinary blocking loop and could not be asked to decide in
advance, because what it decides depends on walls the robot has not reached. So
the move ends early rather than the decision happening late. Between the two
the motors hold their last command open-loop for however long the solver takes
-- about a millisecond in the exploration phases -- and
`CELL_DECISION_MARGIN_CM` is what pays for it. `tm_chain_gap_ms_max` is the
number that says whether it still does; 1.5 cm buys 107 ms at cruise.

**Nothing above the motion layer changed.** Every entry point that needs the
robot standing still calls the stop itself -- both pivots, `CellMotion_Observe()`
and `CellMotion_EndRun()` -- so a caller cannot forget, and the reactive
navigator, which observes at every cell, never chains and behaves exactly as it
did. The flood fill is untouched and `floodfill_diff.sh` still passes on all
seven mazes.

**Walls are read while moving.** The side sensors lead the axle by
`TOF_SIDE_AHEAD_CM`, so they cross into the cell being entered a third of the
way through the move and are still inside it when the segment ends -- about
8 cm of travel, or 15 round-robin rotations at cruise. Votes are counted
exactly as the stationary read counts them, strict majority with an invalid
reading voting "no wall", so the answer does not depend on whether the robot
happened to be moving. **The front sensor is compensated for the distance still
to run**, because a segment that deliberately ends short would otherwise miss
every front wall: one at the far side of the next cell reads about 190 mm from
where the segment ends, against a 150 mm threshold meant for a robot at the
centre. Fewer than `WALL_FLIGHT_MIN_SAMPLES` rotations and the robot stops and
votes, which is slow and right. `tm_chain_flight_reads` against
`tm_chain_stop_reads` says how often that happens.

**A turn no longer re-reads the cell.** The snapshot is robot-relative so a
pivot invalidated it, and the shim responded by observing again -- a settle and
a five-vote sweep, better than a second, to rediscover something the algorithm
had already written into `v_walls`/`h_walls` a moment earlier. It now reads the
rotated view back out of the map. The distances rotate with the flags, with the
side turned away from reported as unmeasured rather than filled in with a
number that means something else.

**`NAV_PIVOT_SETTLE_MS` split off from `NAV_SETTLE_MS`.** They were the same
800 ms constant guarding different things. The wall-reading pause is now rare
and can stay generous; the pause after a pivot is on the critical path of every
turn and was spending two thirds of a second re-confirming what the turn
controller had just confirmed with `TURN_PROFILE_SETTLE_MS`.

**Speed 10 -> 14 cm/s, acceleration 20 -> 28.** The ceiling here has never been
top speed -- it is the 130-unit knee where this motor stops answering a larger
command, above which the feedback has no authority. Feedforward at 14 cm/s is
112 units, still under it, with 88 of the 200-unit budget left for the loop.
16 cm/s would put the feedforward AT the knee and is where this drivetrain
needs gearing rather than tuning. Raising `CONTROL_MAX_SPEED` does not buy it
back.

Supporting changes:

- **`MotionProfile_InitFromTo()`** -- the general form, with a terminal
  velocity. `Init` and `InitFrom` are wrappers, deliberately, so an error in
  the general form shows up in the existing rest-to-rest tests. It reports
  infeasible when the distance is shorter than `|v0^2 - v_end^2| / 2a` and
  builds that minimum instead, because a reference that reverses to make the
  arithmetic work would drive the robot backwards. Three new host tests cover
  cruise-to-cruise, the two ends of a corridor, and both refusals.
- **`StraightMove_t`** replaces the loose arguments to the fused move.
  `runForwardFused()` is now every option at its default.
- **`keep_odometry` is separate from `keep_wall_follow`, and that separation is
  load-bearing.** The chained caller computes each segment's length from an
  absolute odometer; zeroing the encoders inside the segment would move that
  frame out from under it between the caller reading it and the segment
  starting. **`YawEstimator_RebaseEncoders()` is now tied to the reset**, since
  it adds the current yaw to an origin whose "since reset" term it assumes is
  zero -- calling it without having reset double-counts the whole heading the
  robot has turned through since the last real reset.
- **`WallFollow_NewSegment()`** restarts only the movement detector's travel
  baseline. Calling `WallFollow_Reset()` at a cell boundary instead would drop
  a good reference and step the tilt to zero, which is the exact discontinuity
  the slew limit exists to prevent.

What to read first in the next run: `tm_chain_gap_ms_max` against the 107 ms
the margin buys, the flight-vs-stop read ratio, and whether the per-cycle
period histogram still tops out near 12 ms now that the loop is doing the wall
vote as well.

### 2026-09-12 (earlier) - The flood fill, ported

`MicroMouseAlgorithm/maze.c` and `Main.c` now drive the robot, under
`Core/{Inc,Src}/Maze/floodfill/`. They came across essentially unchanged:
`main()` is `FloodFill_Run()` and `debug_log()`'s body moved out, because it was
`fprintf(stderr)`. Every line of `floodfill_phase()`, `updateWalls()`,
`getBestDirection()`, `checkDeadEnd()` and `floodfill_speed_run()` is the
original. `ACTIVE_TEST` is `TEST_FLOODFILL_RUN`.

**`tests/floodfill_diff.sh` is the reason to trust that.** It builds the
original and the port into two binaries, links both against the same simulated
maze, and compares their transcripts action for action over seven seeded mazes.
Everything about the builds is identical except which copy of the algorithm
they contain, so "the algorithm did not change" is checked rather than
asserted. Run it after touching anything under `floodfill/`; if it diverges,
change `MicroMouseAlgorithm` first and re-port.

**The one adaptation: wall readings are cached per cell.**
`API_wallFront/Left/Right` are called several times per cell -- once in
`updateWalls()`, again in `getBestDirection()` -- which is free on the simulator
and about 200 ms a time here. `mms_api.c` reads the cell once on arrival and
serves every query from that snapshot. The differential test runs each maze
both ways and the transcripts match, so this saves time without changing a
decision.

**A failed turn is reported one call late.** `API_turnLeft/Right` return void,
so a timed-out pivot is latched and returned from the next `API_moveForward()`,
which `Main.c` already treats as a crash. The trace shows the failure on the
move after the turn.

**THE ALGORITHM OWNS THE POSE AND THE MAP.** `mouse_x`, `mouse_y`, `mouse_dir`,
`v_walls` and `h_walls` are defined in `floodfill_run.c`; `maze_map.c` used to
define its own copies under the same names, which was harmless only while the
two never met in one binary. It is now a view over that state, keeping only
what the algorithm has no use for: walls by compass side, robot-relative walls
of an arbitrary cell, the next cell along a heading, and a visited bitmap. The
types are the algorithm's -- `bool` walls, `int` pose -- and the reader follows
the pose width change.

Nothing may write the pose except whoever is driving. Two writers would
disagree the first time a move failed, and every wall recorded afterwards would
land in a cell the robot never entered.

**`cell_motion.c` is the layer both drivers stand on**: observe, turn, forward,
plus the residual carry, the front-wall alignment interaction, the wall
follower's cell context and the per-cell trace. It was private to
`navigator.c`; re-implementing it for the flood fill would have meant
re-learning every hard-won rule in it. The right-hand rule stays in
`navigator.c`, so `TEST_MAZE_RUN` still works.

Costs about 4.9 KB of bss, against roughly 16 KB used of 128 KB.

### 2026-09-12 (head) - A move was ending while the robot was still moving

21 cells, ended on a genuine wedge. The per-cell entry error, added last time,
paid for itself immediately.

**The completion test checked position and not speed.** A robot crossing into
the tolerance band at cruise declared the move finished and then carried on for
however far it took to stop. Measured at a median 11.2 cm/s against a profile
asking for 10, exiting a 1.5 cm band, landing 22 to 25 mm past target.

That would be a rounding error if the robot only drove straight. It does not. A
90 degree turn converts longitudinal error into LATERAL error almost one for
one, and the new per-cell record shows it plainly: a cell that finished 22 mm
off was followed by a move beginning 17 mm off centre, where every other entry
error in that run was inside 7 mm. The completion tolerance was setting the
floor on how well placed the robot could be after any turn.

`STRAIGHT_SETTLE_SPEED_CMS` now requires the robot to be stopped as well as
close. **Arrival is latched**, which is the part that matters: without it the
speed condition makes things worse, because a robot that coasts through the
band coasts back out, the test un-arms, the command returns, and it hunts --
eventually backwards on a breakaway pulse, which is the one direction this
chassis has no lateral sensing for. Latching turns "close enough" into a
decision made once, and the command goes to zero from that moment.

**The tolerance stays at 1.5.** Narrowing it now would fight the latch rather
than help: where the robot comes to rest is decided by braking, and a smaller
band only delays the commitment until later in the deceleration, leaving less
room to stop. The band decides WHEN to commit; committing early is what makes
the stop accurate.

**The stall abort now waits for the breakaway to have had its turn.** It ran on
a private 1200 ms clock and could cut the pulse sequence off before it had
spent its budget, which is how a move gets abandoned somewhere the robot could
plainly have driven on. It is gated on `sl_breakaway_count` reaching
`STRAIGHT_BREAKAWAY_MAX` now.

Still open: the alignment fired on 11 of 21 cells with a mean correction of
+1.73 cm and three clamped at the 4 cm limit. Systematically positive
corrections of that size mean the robot arrives consistently short of where the
front wall says it should be, and that bias is not yet explained. It was
deprioritised earlier as a longitudinal problem; the entry-error data is what
makes it a lateral one.

### 2026-09-12 (current) - Stop grinding, stop guessing

Four changes off one run, three of them fixes and one a measurement.

**The alignment was asking the wrong body about room to stop.** The rebuilt
profile starts where the REFERENCE is, so that is the distance it covers, but
whether there is room to decelerate is a question about the ROBOT -- which is
behind the reference by whatever it is lagging, 2 to 3 cm normally and 9 cm
when it is fighting something. Asking the reference made the alignment refuse
itself on exactly the moves that were going badly, and front-wall stops went
from an 11 mm spread to 54 in one run. Now `remaining_robot` decides whether it
may fire and `remaining_ref` is what the profile is built from.

**A move that is not happening is now abandoned.** The existing breakaway only
arms once the profile has FINISHED, so the case it was never written for is a
robot wedged in the MIDDLE of a move. One was measured at 3.8 cm/s against a
profile asking for 10, command pinned between 155 and 176 of 200, steering
clipped on 124 of 200 cycles, reference 9.3 cm ahead, grinding for over two
seconds. `STRAIGHT_STALL_RATE_CMS` and `STRAIGHT_STALL_ABORT_MS` end the move,
and `NAV_END_STALLED` says so -- a wedge and a timeout are different failures
wanting different answers, and reporting both as MOVE FAILED sent a run's worth
of investigation at the steering when the cause was mechanical.

**The integral has a third gate.** `conf` guards against a reference not worth
believing and the clamp guards against the loop already asking for everything
it can. Neither covers the loop asking correctly and the ROBOT not answering.
That run drove the term to -7.66 of a +/-8 limit, most of it manufactured while
wedged. `WALL_FOLLOW_MIN_TRAVEL_CMS` stops it learning when the wheels are not
making ground -- lateral authority comes from leaning while moving forward, so
with no forward motion there is nothing to learn from the correction failing to
arrive. The travel sample is taken at the TOP of `WallFollow_Update`, before
the no-reference path returns, or a stretch of cells with no wall comes back as
one enormous step over a single interval and reads as a robot sprinting.

**And the measurement, which is why this run was worth it.** A robot came out
of a dead end 46 mm further from the same wall than it went in, across one 180
and one cell of travel, and nothing recorded which of the two did it. The
per-cycle trace only survives the last move. `MazeTrace_t` is 48 bytes now and
carries `entry_err_mm`, the lateral error each arriving move STARTED with, plus
`align_delta_cm` so which moves aligned is a fact rather than an inference from
where the robot stopped. Paired with `move_error_cm` this says whether a cell's
offset was inherited or created.

If pivots turn out to be throwing the robot tens of millimetres sideways, the
lateral loop is being asked to clean up after a far larger disturbance than
anything it has been tuned against, and no amount of gain work fixes that. The
reader prints the worst entry error and how many cells exceed 15 mm.

### 2026-09-12 (newest) - Align late, and tell the follower where it is

Two faults from one move in the last run.

**The alignment never fired, and never could have.** It looked only during the
first quarter of the move -- exactly when the front wall is furthest and its
reading worst. That move began with the wall at 407 mm and the window shut at
5.1 cm of travel with the wall still 356 mm off, six millimetres outside range.
About half the forward moves in that run could never align at all.

The real limit on firing late is physical, not a fraction: there must be room
to decelerate to the new endpoint from the speed the reference is doing. So the
window is gone and the gate is now `remaining >= v^2/(2a) + WALL_FRONT_ALIGN_ROOM_CM`,
with the alignment preferring to wait until the wall is inside
`WALL_FRONT_ALIGN_BEST_MM` and falling back on a distant reading only when it
is running out of room.

That needs a profile that starts at the reference's current velocity, so
`MotionProfile_InitFrom()` was added and `MotionProfile_Init()` became a
`v0 = 0` wrapper over it -- deliberately, so an error in the general form shows
up in the existing rest-to-rest tests rather than only in the arena. It returns
0 when the move is shorter than the braking distance and builds the hardest
stop instead, because a reference that reverses to make the arithmetic work
would drive the robot backwards.

**The side sensors are 4 cm ahead of the axle (measured), so they cross the
cell boundary at 5.6 cm of a 19.2 cm move.** They spend more than two thirds of
every move looking at the cell being ENTERED, and nothing in the firmware knew
it. In the failing move the follower tracked the right wall of the cell it was
leaving for 3.4 cm past the boundary, read that wall's recession as the robot
drifting, and leaned 4.68 degrees into the opposite wall. The left sensor
touched it. Lateral error sat between -21 and -26 mm for all 200 cycles and
never improved, even with the tilt pinned at the clamp for 63 of them.

`WallFollowCells_t` now carries what the map knows about both cells plus the
crossing distance, set by the navigator before each forward move and cleared by
`WallFollow_Reset()`.

**THE MAP MAY ONLY WITHHOLD TRUST, NEVER ADD IT.** A reference is dropped when
the applicable cell is surveyed and records no wall there. A wall the map
believes in but the sensor cannot see is never conjured into one. That
direction is the whole safety argument: a wrong pose makes the loop more
cautious rather than more confident, and this robot has driven off the edge of
its own map before. The host test asserts no combination of inputs can create a
reference.

This needed the map to be able to say "I do not know". `MazeMap_UpdateWalls()`
only ever sets a wall to 1, so a zero meant "no wall seen here", which reads
identically to open -- every unexplored cell would have reported as wide open
and vetoed everything. A 32-byte visited bitmap and `MazeMap_IsKnown()` close
that. `MazeMap_CellWalls()` is the read mirror of `UpdateWalls`, kept beside it
because a left/right swap in that arithmetic produces a map that is plausible,
self-consistent and mirrored. `MazeMap_Advance()` is now written in terms of a
new `MazeMap_NextCell()` so the two cannot disagree about where the edge is.

An unsurveyed next cell contributes no opinion, which is the common case on a
first pass and leaves the follower exactly as it behaved before.

### 2026-09-12 (head) - The alignment was commanding the robot backwards

The round-robin poll did what it was meant to: every cycle 10-13 ms, nothing
over 25, the loop running free for a whole move. The run reached 43 cells
against 29 the time before and failed only on its last turn. Front-wall stops
held at 73-91 mm, mean 85, across 19 cells.

**With the loop fast, a defect that had always been there became visible.** The
front-wall alignment added its correction to the profile's OUTPUT, which moves
the origin by the same amount as the endpoint. A correction that shortens the
move therefore commands the robot backwards before it has gone anywhere:

    t(ms)   ref_cm   act_cm    base   steer      yaw
      110    -2.58     0.00   -45.0   -12.3  -1444.71
      322    -1.66     0.01    45.0   -35.6  -1444.85
      832     3.12     0.24   137.6   -33.4  -1446.29

Standing still was not the expensive part -- stiction held the robot, so it
never actually reversed. The expensive part is that the lateral loop ramps its
tilt through those 800 ms, and a heading correction with no forward motion is
not a translation, it is a PIVOT. The robot turned 1.6 degrees on the spot and
entered the cell already yawed, which is the opposite of what the alignment
exists to do. It bites on every negative correction, and therefore on the turn
cells, which are the ones with a front wall to align against.

The profile is now REBUILT rather than translated: a new profile for what
remains, anchored at the reference position the move has already reached, with
a clock to match. Anchoring on the reference rather than on the robot is the
part worth stating -- anchoring on the robot would step the reference back by
however far it currently lags, which is the same defect in a smaller size.

Its velocity restarts from zero, which is why the alignment stays confined to
the opening of a move: there the reference is barely moving and the
discontinuity lands in a term the position loop covers easily. Position itself
never jumps, and that is the property the distance loop actually closes on.

`motion_profile_host_test.c` pins it, including a check that reproduces the old
translation and asserts it does step backwards, so the defect cannot return
quietly.

Still open after this run: the robot ran 23-28 mm toward the left wall for six
cells in the bottom-left corridor and could not recover inside a cell, with the
loop holding -5.76 of learned drift plus a pinned -10 of tilt. The 16 degree
heading errors there are the commanded state, not error. And 11 stale ToF drops
appeared for the first time -- a sensor going 120 ms without producing, 11 times
in about 25000 reads.

### 2026-09-12 (current) - A taper instead of a cliff, and the poll unbunched

The anti-windup worked: the integral stayed between +0.92 and -2.14 across 28
cells against the previous run's pin at -8.00, and front-wall stops came in at
74-89 mm, mean 83, against the 87 mm target. Neither of those is the problem any
more.

**The single-wall rule was keying on the sign of the error when it should have
keyed on the size.** Any long reading got a fifth of the gain and a 2.5 degree
cap. The robot then carried a 14 mm error for a full second against a right wall
reading 76 mm -- twelve millimetres long, where an opening reads 240 and the
95 mm usable gate already rejects one -- and the rule throttled a correction
that was entirely correct. The proof arrived a moment later: when the left wall
came into range it agreed, too close on the left by 14 where the right had said
too far by 12. That cell ended 28 mm off centre and the turn out of it jammed
44 degrees short.

Confidence is now a ramp: 1.0 at the setpoint, falling to
`WALL_FOLLOW_FAR_CONF_FLOOR` at the usable gate, multiplying both the gain and
the tilt clamp. The ramp's two ends are the two things already known -- a close
return can only be a wall, and a reading at the gate is about to be discarded --
so only the middle is interpolated.

The integral gets a threshold rather than a taper, at `WALL_FOLLOW_TRUST_CONF`.
The proportional term may act on a doubtful reference in proportion to the
doubt, because it forgets the moment the reference changes; an integrator does
not forget, and a wrong guess accumulated into it is held until something else
unwinds it.

**The ToF poll went round-robin.** `ToF_PollOneLatest()` talks to one sensor per
control cycle and serves the other two from the held-reading cache, replacing a
sweep of all three every fourth cycle. Same I2C work, same per-sensor rate,
unbunched: 35 ms of blocked loop becomes about 12. `STRAIGHT_TOF_DIVIDER` went
4 -> 3 and now means one complete rotation, with a static assert tying it to
`TOF_SENSOR_COUNT` -- if those disagree the wall follower either sees repeated
samples or misses some, and neither failure announces itself in the arena.

The side pair stops being simultaneous by at most two cycles, which at cruise is
a couple of millimetres along the corridor and a fraction of one across it, well
inside what the span check already tolerates.

Expect the period histogram to read about 12 ms throughout with nothing above
25. If it still shows 35s, the rotation is not happening.

### 2026-09-12 (newest) - The integral was the thing moving, not the alignment

First run on continuous ranging. Mode switching worked: both failure flags read
0, 3870 fresh reads against 3 held, no stale drops. The run reached 33 cells,
up from 27.

**Front-wall alignment got much better and is not the problem.** Stop distances
against the 87 mm target spanned 67-92 mm, mean 81. The previous run spanned
42-112. Nothing about the alignment gain wants reducing; it was starved of
sweeps, and now it is not.

**The lateral integral was winding on position error.** Per-cell it moved as
much as 3.3 degrees, pinned at WALL_FOLLOW_KI_LIMIT_DEG for a stretch, and
since it is added to the heading target it then held the robot 7 degrees off the
maze. The heading errors of +11 to +14.7 degrees in the late half of that run
were this term's own output, not something it was correcting.

The cause is that a single-wall cell routinely shows 20-25 mm of lateral error
where the two-wall corridor the gain was sized against shows 8. Two fixes, and
both are needed:

- **Anti-windup**, the ordinary kind: stop integrating while the proportional
  term is clamped. A large lateral error is a POSITION error and belongs
  entirely to P. Only what P cannot remove is evidence of a standing bias, and
  while P is pinned there is no such evidence to be had.
- **`WALL_FOLLOW_KI_DEG_PER_MM_S` 0.10 -> 0.04.** The host test shows why the
  gain alone was not enough: at 0.04 the measured cell still moves the term
  about 1.3 degrees, better than 3.3 and still far too much for something
  learning a property of the robot. The gate is what removes it.

**The loop is four times better and not yet fixed.** Periods now read 10 ms for
150 cycles and 35 ms for 49, so 53% of the move still has the loop stopped,
down from 81%. The 35 ms is I2C traffic for three sensors rather than ranging
latency, so the remaining move is to read one sensor per control cycle and let
the held-reading cache cover the other two: four cycles would cost about 48 ms
instead of 65, and no single cycle would exceed about 12.

### 2026-09-12 (last) - The control loop was open 81% of the time

Measured from the raw straight trace: 60 cycles at 10 ms and 19 at 138 ms. Of
3222 ms in one move, 2622 were spent inside a blocking sensor read with the
motors holding a stale command.

`ToF_ReadAll()` does three blocking single-shot reads at about 46 ms each. The
driver has had a non-blocking continuous mode all along -- its own header calls
it "what a moving robot wants" -- and the maze run simply never turned it on.
Only two test-harness functions did.

**This was not a tuning problem, it was why several tuning problems could not be
fixed.** Every PID was told each 138 ms gap was 10 ms, which multiplies the
derivative by 13.8. From the trace, across one sweep the heading error moved
5.66 deg, the D term computed `0.20 * 5.66 / 0.010` = 113 units, P added 42, and
the steering clamped at its limit. With the true period it is 8 units and
nothing saturates. Every steering slam in the late half of that run was this
arithmetic, not a collision. The wall follower had the same bug at a quarter
scale: its slew and integral used a nominal 40 ms against a real 168.

Switching the mode is one call. These were the consequences, and two of them
would have broken the robot outright:

- **A non-blocking read with nothing new invalidates the measurement.** Polling
  a 40 ms sensor from a 10 ms loop means three reads in four come back invalid,
  and `usable()` in `wall_follow.c` reads invalid as "no wall" -- the loop would
  drop its reference, slew to zero, re-acquire, and flicker at the poll rate.
  `ToF_ReadAllLatest()` holds the newest good reading and serves it with an age.
  The age limit is what tells a slow sensor from a dead one; without it a failed
  sensor's last reading is served forever and the robot steers to a wall that is
  not there. `tof_stale_drops` counts that and should be zero.
- **`WallSense_ReadCell()` votes five times with no delay.** Single-shot made
  each vote an independent measurement. Free-running, five back-to-back reads
  return one sample five times and report it as a unanimous 5/5.
  `ToF_ReadAllFresh()` waits for a genuinely new measurement per vote. Costs
  about 200 ms per cell, down from 690.
- **`ToF_ResetFilterAll()` after a pivot stopped being a guarantee.** It exists
  because the stored samples describe a heading the robot no longer holds.
  Single-shot guaranteed the next sample was triggered after it; free-running
  does not, and the sample in flight may have been captured mid-rotation. It now
  arms a per-sensor discard: the next sample is consumed, latch cleared, never
  decoded, so the first reading the filter sees provably started after the reset.
- **Stopping continuous mode moved onto the critical route.** A stop is only a
  request and the part is undefined if reconfigured during the window. Every
  `break` in `Navigator_Run()` already fell through to one cleanup point, so the
  stop lives there, and a failure is recorded in `tm_maze_tof_stop_fail` rather
  than swallowed.

`MAZE_TOF_CONTINUOUS` is the rollback: set it to 0 and everything behaves as it
did before. `TOF_MAX_SAMPLE_AGE_MS` is 120, two measurement periods plus a
timing budget, which is 1.2 cm of travel at cruise.

**Nothing else was retuned, on purpose.** `TOF_FILTER_EMA_ALPHA` stays at 0.2
because per-sample noise rejection does not change with rate, only the time
constant, from about 840 ms to 200 -- which is the point. `STRAIGHT_TOF_DIVIDER`
stays at 4, which gives a 40 ms sweep at a true 10 ms loop and already matches
`TOF_INTER_MEASUREMENT_MS`. `TOF_FILTER_JUMP_THRESHOLD_MM` stays at 30, and
`tof_filter_host_test.c` gained a case proving it: at 40 ms and 10 cm/s a
genuine approach is 4 mm per sample and must not trip the detector, while a wall
ending still must.

New suite `tof_cache_host_test.c` pins the age-gate rule, including the
unsigned-subtraction idiom against the tick counter's 32-bit wrap.

**The pass/fail test is the timing, not the feel.** The reader now histograms
the per-cycle period from the straight trace. It should print 10 ms and nothing
above 25. If it still shows 138s, continuous mode did not start -- check
`tm_maze_tof_start_fail` before concluding anything about the tuning.

### 2026-09-12 (latest) - One integrator, and where it is applied

The lateral loop was not converging: a run showed the robot visibly yawed left
with the left wall in view and no correction arriving. Two causes, both about
structure rather than gains.

- **The correction had a ceiling it could not pass.** The integral added in the
  previous change sat INSIDE the tilt sum, so the total lateral authority was
  bounded by `WALL_FOLLOW_MAX_TILT_DEG`. If the yaw estimate is off from the
  maze by more than that clamp, the robot cannot recover: to drive straight it
  must command the full offset, the clamp stops it short, and it keeps turning
  the same wrong way for as long as the wall lasts. The proportional term
  saturates first and the integral, sharing the same budget, then has nothing
  left to give.

  The integral now goes to the **heading target**, outside the tilt clamp, via
  `WallFollow_GetDriftDeg()`. The proportional term keeps its full tilt budget
  for position. `WALL_FOLLOW_KI_LIMIT_DEG` is 8 degrees and is deliberately
  LARGER than the tilt clamp -- `wall_follow_host_test.c` asserts that
  ordering, which is the inverse of the check it replaced.

  The linear-range rule still applies, but to the heading loop's ERROR, not to
  how far the setpoint has moved. A setpoint the robot tracks costs no error.
  What has to stay small is the integral's RATE, and the test checks that.

- **There were two integrators doing one job.** `WALL_FOLLOW_DRIFT_BLEED`
  integrated the TILT, and the tilt contained the integral -- two integrators
  in series on one error, with no clamp on the outer one and no way for them to
  agree on which owned the correction. The bleed is gone. One term now does
  both jobs and it is the drift corrector, integrating the lateral error. The
  reasoning that motivated the bleed is unchanged: a tilt the robot must hold
  forever is the gyro being wrong, because a centred robot needs no tilt to
  stay centred. It just arrives one integration earlier.

- **Turns inherit the bias too.** `turn_controller.c` adds the learned drift
  where the target is USED, the same way the straight move does; the
  accumulator stays nominal. Without it every turn landed the robot at a
  heading the loop already knew was wrong, and the next straight spent its
  first stretch turning out of it.

**Then the clamp itself went up, 6 -> 10, with its two companions.** The log
justified it: five consecutive cells asked for more tilt than the clamp could
give, two of them for more than twice it (13.5 and 12.8 degrees against a 6
degree clamp). The robot spent the stretch pinned at the limit, drifted into
the wall anyway, and wedged -- the next straight held ~180 of 200 command units
for a second and a half while making 2.9 cm/s against a profile asking 10.

Raising the clamp alone would have re-created the bang-bang failure the note at
`WALL_FOLLOW_MAX_TILT_DEG` describes, so all three moved together:

| constant | was | now | why |
|---|---|---|---|
| `WALL_FOLLOW_MAX_TILT_DEG` | 6 | 10 | covers the 27 mm worst case actually recorded |
| `STRAIGHT_YAW_LIMIT` | 60 | 80 | widens the inner loop's linear range to 80/8 = 10 first |
| `WALL_FOLLOW_TILT_SLEW_DPS` | 30 | 15 | see below |

The slew limit was the quiet one. The inner loop buys `STRAIGHT_YAW_KP /
TURN_FF_GAIN` = 4 deg/s of turn rate per degree of heading error, so following a
setpoint that ramps at R deg/s costs R/4 degrees of standing error. At 30 that
was 7.5 degrees -- the whole linear range, spent on the ramp before the tilt
asked for anything. A slew limit reads like a safety measure; it is also a
load, and it was saturating the loop it exists to protect.

`wall_follow_host_test.c` now asserts the cascade rule symbolically
(`WALL_FOLLOW_MAX_TILT_DEG <= STRAIGHT_YAW_LIMIT / STRAIGHT_YAW_KP`) and the
ramp cost separately, so these cannot drift apart again.

**Watch for corner swing.** The cost of 10 degrees is half a chassis length
times sin(10) rather than sin(6), roughly 3.5 mm more on a 100 mm body in a
124 mm corridor. If the robot starts clipping walls mid-corridor rather than at
junctions, this is the first thing to look at. The clamp is only reached at
20 mm of error; a well-centred robot never sees it.

Instrumentation, since the complaint was about a heading that no log showed:

- `StraightTrace_t` is 40 bytes and carries `yaw_deg`, `tilt_deg`, `drift_deg`
  and `err_mm`. `yaw_err` alone says the inner loop is happy; it cannot say
  whether the heading it is happy about is the right one. Capacity is 200, up
  from 120, which was running out four fifths of the way through every move.
- `MazeTrace_t` is 40 bytes and carries `drift_deg` per cell, because the term
  is supposed to CONVERGE over a run and that is not visible in one move.
- The reader prints the heading swept per move, the drift at every cell, and a
  verdict on whether it settled, is still climbing, or is pinned at the limit.
  Pinned means mechanical asymmetry, not a gain to trim.

### 2026-09-12 (later) - Front-wall alignment, and backing out of dead ends
- **The forward axis is now closed-loop.** The side walls always held the
  robot's lateral position; nothing held its longitudinal position except
  odometry. A pivot swaps the two axes, so an uncontrolled forward axis
  reappears one move later as a clearance problem -- which is exactly why
  back-to-back turns were far worse than corridors. Measured over one 24-cell
  run: worst off-centre 6.5 mm in the corridor stretch, 23.0 mm in the
  turn-dense stretch, with the front-wall gap at the ten walled stops
  scattered over 50-110 mm. Same 60 mm, seen from two directions.
- `runForwardFused()` now retargets so the move ENDS at `WALL_FRONT_ALIGN_MM`
  from a wall ahead, when one is in range. Applied at most once and only in
  the first quarter of the move: the retarget translates the profile, and a
  step that arrives after the robot has braked would need a few cm closed from
  rest, which this drivetrain cannot do.
- **New `runReverseFused()`.** `runForwardFused()` and it are now two thin
  wrappers over one `runFused()`; reverse needed no second copy of the loop
  because the profile, the encoders and `applyMinSpeed()` were all already
  signed. Two things did need flipping and are marked `!! DIRECTION !!`:
  the wall follower (tilting the nose left walks the robot left going
  forwards and RIGHT going backwards, so an unflipped cascade is positive
  feedback in reverse), and the stiction-floor gate (`ref_acc >= 0` meant
  "not braking" only going forwards; it is now `ref_acc * ref_vel >= 0`).
- **Dead ends back out instead of pivoting in place.** `NAV_ACT_AROUND` is now
  reverse one cell, then turn 180. Same two moves as before in the opposite
  order, ending in the same cell facing the same way for the same cost -- the
  host test asserts that equivalence. The gain is that the pivot happens after
  a full cell of lateral correction rather than the instant the robot arrives.
  Every pivot that succeeded in that run was within 8.5 mm of centre; the one
  that jammed was 24 mm out.
- **The reverse is the best-referenced move the robot makes.** Backing out, the
  wall it just faced stays in view the whole way, so the move is measured
  against the wall instead of counted in ticks -- and it corrects the error the
  robot ARRIVED with, which odometry cannot. Arrive 50 mm from the wall and it
  reverses 22.9 cm; arrive at 110 mm and it reverses 16.9 cm. Both finish in
  the same place.
- Added `MazeMap_Retreat()`, `sl_align_delta_cm` / `sl_align_applied`
  telemetry, and `WALL_FRONT_ALIGN_MM` / `_RANGE_MM` / `_MAX_CM` to
  `control_config.h`.
- `WALL_FOLLOW_KP_DEG_PER_MM` raised 0.25 -> 0.50 (one cell of travel now
  removes 81% of a lateral error rather than 57%), and
  `TURN_PROFILE_MAX_DPS` lowered 120 -> 90 after a trace showed the command
  saturated through every cruise and the robot delivering only 113 dps.


### 2026-09-12 - Reactive navigation, split out of the test harness
- **The scripted arena route is gone.** `TEST_MAZE_RUN` used to drive a fixed
  sequence (forward, forward, turn right, forward). The robot now stops at
  each cell centre, reads its three ToF sensors, and picks the next action
  from what it saw, so the same binary runs any arena. Right-hand wall
  following, not flood fill: it knows nothing about where a goal is.
- **New module `Maze/navigator.c/.h`.** The behaviour does not belong in the
  test harness -- everything else there is bring-up scaffolding that exercises
  one subsystem and is then never touched again, whereas this is the robot's
  actual job and is what the flood fill eventually replaces. `test_harness.c`
  dropped from 989 to 754 lines and now calls `Navigator_Run()`.
  `MazeTrace_t` and the `tm_maze_*` telemetry moved with it; the names did not
  change, so the SWD reader is unaffected.
- **Every action ends in one cell of forward motion.** The turn only chooses
  which way to leave. The first version treated "turn right" as a complete
  action and `tests/navigator_host_test.c` caught it: the robot reached the
  opening, turned into it, saw another open right, and pivoted back down the
  corridor it came from without entering the new cell. On hardware that would
  have looked like a turn-tuning problem.
- **Decisions read the SENSORS, never the map.** The map has the outer
  boundary pre-set and never clears a wall, so deciding from it would let one
  bad reflection close a corridor for the rest of the run.
- **Every pivot now resets the ToF filter and the wall follower.** Nothing did
  this before. The median and EMA stages carry several samples across a turn,
  and the jump detector only rescues large steps -- so turning from one wall
  to another at a similar distance slipped through as a slow ramp. It mattered
  little with a fixed route; it matters a lot when the next reading picks the
  next turn.
- **A failed move no longer writes walls into the map.** The pose is
  deliberately not advanced on failure, so the robot is between cells and the
  readings belong to no cell the map can name. They are still logged, because
  they are the evidence of what went wrong.
- Runs are bounded four ways, reported in `tm_maze_abort_reason`: cell budget,
  failed move, returning to the start cell, or a full trace buffer. A wall
  follower in open space circles forever and a bench arena has no outer
  boundary to stop it.
- `MazeTrace_t` gained the chosen action and the per-sensor vote tallies,
  packed into padding the compiler was already inserting, so it stays 36 bytes
  and the reader's stride is unchanged. A 3-2 vote on the FRONT sensor is now
  visible -- that is the reading that decides whether the robot drives into a
  wall.
- Added `tests/navigator_host_test.c` (9 checks) and a `tests/README.md`
  section.


### 2026-09-11 — ToF noise filtering + per-sensor offsets
- Bench measurement with the sensors working: a wall at a true 80 mm read
  83–90 mm. That is **two** problems — ~7 mm of spread (noise) on top of a
  ~+6 mm consistent over-read (bias) — and they are now handled separately,
  because filtering cannot fix bias.
- Added `Sensors/VL53L0X_Driver/tof_filter.c/.h`: median (3-sample) feeding an
  EMA, plus a step detector. Median kills lone outliers, EMA smooths dense
  jitter. A plain moving average was rejected — its lag into a wall-following
  controller invites oscillation, and worsens as you lengthen the window.
  Host-measured: **σ 2.27 mm → 0.94 mm**, mean preserved.
- **The jump detector tests the RAW sample, not the median.** The host test
  caught this as a real defect: a genuine wall transition arrives as one new
  value against a window of old ones, so `{86, 86, 250}` medians to `86` and
  the median stage suppressed the first sample of every real step exactly as
  if it were an outlier — leaving the robot blind to an opening for two more
  samples, which is the failure the detector exists to prevent. Cost of the
  fix is one sample of overshoot on a large lone outlier; documented and
  asserted in test 5.
- Added per-sensor `TOF_OFFSET_*_MM` to `control_config.h`, applied **before**
  filtering so the filter smooths an already-centred signal and the jump
  threshold compares corrected values. Currently all 0 — they need measuring
  against a ruler (see §9). Offset application clamps at 0 so a negative
  offset cannot underflow `uint16_t` into a huge distance.
- Added `TOF_FILTER_EMA_ALPHA` (0.2) and `TOF_FILTER_JUMP_THRESHOLD_MM` (30).
  30 mm sits between the ~7 mm noise floor and the ~100 mm+ real transitions.
- `ToF_Measurement_t` now carries `raw_mm` alongside `distance_mm`, so
  bring-up can tell a noisy sensor from a badly-tuned filter.
- Filter continuity is broken on genuine faults (bus error, blocking timeout,
  invalid range) but **deliberately not** on the non-blocking
  `TOF_ERROR_TIMEOUT` path — that fires whenever polling outpaces the sensor,
  and resetting there would clear the history on most calls and destroy the
  smoothing entirely.
- Added `tests/tof_filter_host_test.c` (8 cases) and expanded
  `tests/README.md`. Test 2 asserts a *limitation* on purpose — that the
  filter does not remove bias — so nobody tries to fix a constant error with
  filter constants.
- `test_harness.c/.h`: added `tm_tof_*_raw_mm` and `tm_tof_*_jumps` telemetry.
  Jump counts climbing while stationary means the threshold is below the noise
  floor.
- Channel mapping corrected to front=0, left=3, right=4 (was 0/1/2).

### 2026-09-10 — VL53L0X ToF ranging brought up
- Added ST's official VL53L0X API (STSW-IMG005 v1.0.4) under
  `Sensors/VL53L0X/`, verbatim. The five `Core/` sources are in the build;
  see §9 for the three Win32-only `Platform/` files that are deliberately
  **not**.
- **Re-ported `Platform/vl53l0x_platform.c`.** ST ships it as a Win32
  reference implementation (`#include <Windows.h>`, talks to a Nucleo over a
  COM port via `ranging_sensor_comms.dll`) which cannot build for this
  target. Replaced with a direct STM32 HAL I2C implementation:
  read/write byte/word/dword built on `HAL_I2C_Mem_Read` /
  `HAL_I2C_Master_Transmit`, MSB-first packing for the sensor's big-endian
  registers, and a 1 ms `VL53L0X_PollingDelay()`. This is the only ST file
  modified, so an API update means re-porting one file.
- Added `Sensors/TCA9548A/TCA9548A.c/.h` — mux channel select, with the
  active mask cached so a repeat select costs no I2C traffic. Every sensor
  read is wrapped in a select, so that mattered.
- Added `Sensors/VL53L0X_Driver/tof_sensors.c/.h` — the application-facing
  driver. Three sensors (front/left/right), single **and** continuous
  ranging, distances in **mm**. Every entry point selects the mux channel
  first. Continuous mode uses `CONTINUOUS_TIMED_RANGING` rather than
  back-to-back, and `ToF_ReadContinuous()` has a non-blocking form for use
  from a control loop.
- Failure handling mirrors the IMU's: a sensor that fails init is marked
  not-ready and skipped by later calls rather than retried, so one dead
  sensor cannot stall a loop with repeated I2C timeouts. The rest stay
  usable. **2 fast LED blinks at boot** = ToF init failure.
- Invalid readings return `TOF_DISTANCE_INVALID` (0xFFFF) rather than a stale
  distance, so ignoring a return code fails loudly. `TOF_ERROR_RANGE`
  (sensor fine, nothing in range) is kept distinct from `TOF_ERROR` (bus
  fault) — different problems, different fixes.
- `control_config.h`: added the ToF section — mux channel mapping, timing
  budget, inter-measurement period, VCSEL periods, signal/sigma limits.
  Note **VCSEL periods must be set before the timing budget** or the API
  silently recomputes the budget; the init order encodes this.
- `test_harness.c/.h`: added tests 11 (`TEST_TOF_SINGLE`) and 12
  (`TEST_TOF_CONTINUOUS`) plus `tm_tof_*` telemetry. `tm_tof_ready` is a
  bitmask latched at boot so the mask is readable whatever test is selected —
  all-zero means the mux never answered, which is the failure worth
  distinguishing first.
- `main.c`: `ToF_Init()` in `USER CODE BEGIN 2`, after the controllers.
- **Wall detection is deliberately not implemented.** This change reports
  distances and nothing more; interpreting them belongs with the maze logic.

### 2026-09-09 — Test harness pulled out of main.c
- `main.c`'s `USER CODE` blocks had absorbed the entire test harness (20
  `tm_*` telemetry globals, `LED_Blink`, `Telemetry_Capture`/`_CaptureYaw`,
  `Test_Pause`, all 11 `Test_*` routines, and the `ACTIVE_TEST` dispatch)
  alongside CubeMX's peripheral bring-up. Moved all of it, unchanged, into a
  new `Core/Inc/Tests/test_harness.h` / `Core/Src/Tests/test_harness.c`
  module — mirrors how `straightline_controller`/`turn_controller` are
  already split out. `main.c` now just includes `test_harness.h` and calls
  `TestHarness_RunCycle()` from its `while(1)` loop.
- `LED_Blink` is the one function `main()` itself still calls directly (for
  the IMU-up/absent startup indicator), so it's `void` (not `static`) and
  declared in `test_harness.h`.
- Pure move, no behavior change. Requires `Core/Src/Tests/test_harness.c` and
  `Core/Inc/Tests` to be registered in `CMakeLists.txt`
  (`target_sources`/`target_include_directories`) — already done.

### 2026-07-31 — IMU integration for precise turns
- Added `EKF.h`/`EKF.c`: 2-state (yaw, gyro bias) filter fusing gyro + encoder
  odometry. Joseph-form covariance, innovation gating, rate-adaptive R,
  continuous (unwrapped) yaw.
- Rewrote `turn_controller.c/.h` to close the loop on fused yaw in **degrees**
  instead of differential wheel arc in cm. Multi-rate: 1 kHz predict /
  100 Hz correct+actuate. Graceful encoder-only fallback when the IMU is absent.
- Added `Utils/dwt_timer.c/.h` — microsecond timebase for gyro integration
  (`HAL_GetTick()` at 1 ms is too coarse).
- `control_config.h`: added robot geometry, IMU and EKF sections; converted
  turn gains to the degrees convention (`Kp` 40.0 → 3.9 via the ×0.09774
  factor); replaced `TURN_TOLERANCE_ARC_CM` with `TURN_TOLERANCE_DEG` and added
  `TURN_SETTLE_RATE_DPS`.
- `ICM42688.c`: **fixed** burst read starting at the wrong register, which made
  `temperature` return timestamp bytes. Soft reset now precedes the WHO_AM_I
  check, with retries. Added `ICM42688_ReadGyroZ()` (2-byte fast path for the
  1 kHz loop) and `ICM42688_CheckID()`.
- `main.c`: IMU/EKF telemetry, tests 8–10, CS deassert, `DWT_Timer_Init()`.
- Added `tests/` with a host harness for the EKF. It found a real defect:
  fixed-R fusion tracked a 30% slip ramp to 100.4°, which motivated
  `EKF_R_SLIP_COEFF`.

### 2026-07-30 — Low-level control bring-up
- Filled in `CMakeLists.txt`: user sources were never in the build.
- `main.c` test harness with `ACTIVE_TEST` selector and live-watch telemetry.
- Added `control_config.h` as the single tuning surface.
- Fixed: `HAL_TIM_Encoder_Start`/`HAL_TIM_PWM_Start` were never called, so
  nothing moved.
- Fixed: negative PID output cast to `uint8_t` wrapped (−50 → 206), causing a
  full-speed lurch instead of a stop. Added `Motor_runSignedSpeed()`.
- Fixed: `static inline` with a later definition (undefined reference at -O0).
- Fixed: `while(1)` with no exit — added `CONTROL_MOVE_TIMEOUT_MS`.
- Fixed: completion on a single in-tolerance sample — added
  `CONTROL_SETTLE_CYCLES`.
- Fixed: `TUEN_CONTROLLER_H` guard typo, duplicate `PI` macro.
