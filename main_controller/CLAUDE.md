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
| Range | VL53L0X ToF — **hardware present, not integrated in firmware yet** |
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

`tests/` contains a host harness for the EKF; see `tests/README.md`. Run it
after any change to `EKF.c` — it catches maths errors that on-target testing
cannot.

---

## 3. Layout

```
Core/Inc, Core/Src
├─ Control/LowLevel/    PID.c  EKF.c  straightline_controller.c  turn_controller.c
│                       control_config.h  ← ALL tuning lives here
├─ Encoders/            encoders.c
├─ Motors/              DRV8833.c
├─ Sensors/ICM-42688-P/ ICM42688.c
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
11 test routines (implemented in `Core/Src/Tests/test_harness.c`). Set it,
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

- VL53L0X / wall detection — driver files not present yet
- Any maze-solving algorithm (flood fill, DFS, …)
- IMU in the straight-line controller
- Magnetometer — **not present in hardware**, so absolute heading is
  impossible. Yaw is always relative to the last reset.

---

## Change log

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
