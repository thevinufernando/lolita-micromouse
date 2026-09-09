# Main Controller — Firmware

This folder contains the firmware for the micromouse's main controller board:
an **STM32F405RGTx** (Cortex-M4F) running low-level motion control for the
differential-drive robot. It's built on **STM32CubeMX + HAL**, developed with
the **STM32Cube VSCode extension**, and built via **CMake + Ninja**.

> For the electrical design this firmware runs on, see
> [`pcb_designs/`](../pcb_designs) (Main Controller PCB section).

For the full technical reference used in day-to-day development — hardware
pin/peripheral map, sign/unit conventions, control architecture, tuning
procedure, hardware quirks, and change log — see **[CLAUDE.md](CLAUDE.md)**.
This README covers the same ground at a higher level plus environment setup.

---

## Current status

The robot is in **low-level controls bring-up**. Straight-line motion and
pivot turns are working and tunable. Maze-solving is **not implemented** and
out of scope until the motion primitives are trusted. The VL53L0X ToF sensors
are present on the PCB but not yet integrated into firmware.

## Hardware summary

| Part | Detail |
|------|--------|
| MCU | STM32F405RGTx, Cortex-M4F, SYSCLK 96 MHz / **HCLK 48 MHz** |
| Motors | 2× N20 gear motor w/ quadrature encoder, differential drive |
| Motor driver | DRV8833 (IN/IN mode, 4 PWM channels) |
| IMU | ICM-42688-P (accel + gyro) over SPI1 — no magnetometer |
| Range | VL53L0X ToF ×5 — hardware present, not yet wired into firmware |
| Power | 3S LiPo, regulated by the companion Power PCB |

There is **no UART**. All runtime observation happens through the ST-Link
debugger's live-watch panel (`.vscode/launch.json` has the watch list
pre-populated), which is why the firmware exposes many `volatile` telemetry
globals.

## Firmware architecture

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

- **Straight-line motion** is encoder-only by design.
- **Turns** fuse gyro + encoder odometry through a 2-state EKF (`[yaw, gyro_bias]`),
  running a multi-rate loop (~1 kHz gyro prediction, 100 Hz encoder
  correction + PID + motor output).
- All tuning constants live in a single file:
  `Core/Inc/Control/LowLevel/control_config.h`.
- `Core/Inc/Tests/test_harness.h` selects one of several on-target test
  routines (`ACTIVE_TEST`), implemented in `Core/Src/Tests/test_harness.c` —
  motor/encoder checks, straight-line tuning, turn tuning, IMU bring-up, etc.
  `main.c` itself only brings up hardware and calls `TestHarness_RunCycle()`.
  See [CLAUDE.md §6](CLAUDE.md#6-tuning-and-bring-up) for the full list and
  recommended bring-up order.

## Folder layout

```
main_controller/
├── Core/
│   ├── Inc, Src/
│   │   ├── Control/LowLevel/   PID.c  EKF.c  straightline_controller.c  turn_controller.c
│   │   │                       control_config.h  ← all tuning lives here
│   │   ├── Encoders/           encoders.c
│   │   ├── Motors/             DRV8833.c
│   │   ├── Sensors/ICM-42688-P/ ICM42688.c
│   │   ├── Tests/               test_harness.c  ← ACTIVE_TEST + all Test_* routines
│   │   └── Utils/               dwt_timer.c
├── Drivers/                    CMSIS + STM32F4xx HAL
├── cmake/                      Toolchain files + CubeMX-generated CMakeLists
├── tests/                      Host-side EKF verification harness (see tests/README.md)
├── main_controller.ioc         STM32CubeMX project file (pin/clock/peripheral config)
├── CMakeLists.txt              User source list — edit this to add files
├── CMakePresets.json           Debug / Release presets
└── STM32F405XX_FLASH.ld, startup_stm32f405xx.s
```

CubeMX owns the generated regions of `main.c` and all of
`cmake/stm32cubemx/CMakeLists.txt`. Hand-written code must stay inside
`/* USER CODE BEGIN x */ … /* USER CODE END x */` blocks or it will be
overwritten on the next CubeMX regeneration.

---

## Setup: STM32CubeIDE / STM32Cube extension for VSCode

This project is developed and built using the **STM32Cube for VSCode**
extension (ST's official STM32CubeIDE tooling brought into VSCode), not
standalone STM32CubeIDE.

### 1. Prerequisites

- [Visual Studio Code](https://code.visualstudio.com/)
- The **STM32Cube** extension pack from the VSCode Marketplace (search
  `STMicroelectronics` / `STM32Cube`). Installing it pulls in the required
  toolchain bundles (ARM GCC, CMake, Ninja, OpenOCD/ST-Link tools) under
  `%LOCALAPPDATA%\STM32Cube\bundles` — no separate STM32CubeIDE install is
  required.
- An **ST-Link** debug probe connected via the board's SWD header
  (`SWDIO`/`SWCLK`/`SWO`/`NRST`).
- (Optional, only if you need to regenerate peripheral init code)
  [STM32CubeMX](https://www.st.com/en/development-tools/stm32cubemx.html), to
  edit `main_controller.ioc`.

### 2. Open the project

1. Launch VSCode and open the `main_controller/` folder directly (this
   folder, not the repo root) — the extension expects the `.ioc` file,
   `CMakeLists.txt`, and `.vscode/` config at the workspace root.
2. The STM32Cube extension should auto-detect the project via
   `main_controller.ioc` and `.mxproject`. If prompted to configure CMake
   Tools, let it use the settings already checked into `.vscode/settings.json`
   (`cmake.cmakePath: cube-cmake`, Ninja generator).

### 3. Build

The extension drives CMake + Ninja under the hood using the `Debug` /
`Release` presets in `CMakePresets.json`. Use the extension's build button /
CMake Tools status bar, or run it manually.

To build from a shell instead, the ARM toolchain isn't on `PATH` by default —
add the bundled tools first:

```sh
BUNDLES="$LOCALAPPDATA/STM32Cube/bundles"
export PATH="$BUNDLES/gnu-tools-for-stm32/14.3.1+st.2/bin:$BUNDLES/ninja/1.13.2+st.1/bin:$PATH"
cmake --preset Debug        # only needed after adding/removing source files
cmake --build --preset Debug
```

The build must stay **warning-free**.

### 4. Flash and debug

Use the extension's **Run/Debug** panel (or `.vscode/launch.json`), which
flashes over the connected ST-Link and starts a debug session with the
pre-populated **live-watch** expression list — this is the primary way to
observe robot state, since there is no UART output.

> **Do not modify the `preBuild` / `imagesAndSymbols` settings in
> `.vscode/launch.json`.** Only the `liveWatch` expressions array is safe to
> edit freely (e.g. to add a new telemetry variable while tuning).

### 5. Adding a new source file

Two steps, both required, regardless of which tool you use to build:

1. Add it to `CMakeLists.txt` (`target_sources`, and
   `target_include_directories` if it's a new folder).
2. Re-run `cmake --preset Debug` (or reconfigure via the extension) so Ninja
   picks it up.

Never add user sources to `cmake/stm32cubemx/CMakeLists.txt` — it's
regenerated by CubeMX and anything added there is lost.

### 6. Regenerating peripheral code (STM32CubeMX)

If you need to change pin assignments, clocks, or enable a new peripheral,
edit `main_controller.ioc` in STM32CubeMX and regenerate. This rewrites
`cmake/stm32cubemx/` and the generated regions of `main.c`/HAL init files —
hand-written code inside `USER CODE` blocks is preserved, everything else is
not.

---

## Host-side tests

`tests/` contains a standalone host harness that compiles and exercises the
EKF (`Core/Src/Control/LowLevel/EKF.c`) on the development machine against
simulated sensor data, independent of the STM32 build. Run it after any
change to `EKF.c`:

```sh
gcc -O1 -o ekf_test.exe tests/ekf_host_test.c Core/Src/Control/LowLevel/EKF.c \
    -I Core/Inc/Control/LowLevel -lm
./ekf_test.exe
```

See [`tests/README.md`](tests/README.md) for what each test case covers.

---

## Conventions at a glance

Full detail is in [CLAUDE.md](CLAUDE.md) — summarized here:

- **Positive yaw = anticlockwise = left turn**, consistently across encoders,
  gyro, EKF, and PID.
- Distance in **cm**, turn angle in **degrees**, EKF yaw in **continuous,
  unwrapped radians**, motor speed **0–255 signed**.
- There is no magnetometer, so absolute heading is impossible — yaw is always
  relative to the last reset.

## Related documentation

- [CLAUDE.md](CLAUDE.md) — full firmware development reference (peripheral
  map, EKF derivation notes, tuning workflow, hardware quirks, change log).
- [`tests/README.md`](tests/README.md) — EKF host test details.
- [`../pcb_designs/README.md`](../pcb_designs/README.md) — electrical design
  this firmware targets.
