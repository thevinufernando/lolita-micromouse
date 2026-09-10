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
13 test routines (implemented in `Core/Src/Tests/test_harness.c`). Set it,
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

- **Wall detection logic** — the ToF sensors now report distances (see
  §9), but nothing turns those into "there is a wall here" decisions.
  That judgement belongs with the maze logic, which does not exist yet.
- ToF in either motion controller — straight-line and turns remain
  encoder/IMU only; range data is not fed back into control
- Any maze-solving algorithm (flood fill, DFS, …)
- IMU in the straight-line controller
- Magnetometer — **not present in hardware**, so absolute heading is
  impossible. Yaw is always relative to the last reset.

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
