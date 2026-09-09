# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this repo is

Lolita is a differential-drive micromouse robot: STM32F405RGT6 MCU, 6-axis IMU,
encoder-equipped DC motors, and five VL53L0X ToF sensors for wall detection.
This repo spans the whole project — firmware, two custom PCBs (Altium), and
mechanical CAD — not just software.

**Current status: low-level controls bring-up.** Straight-line motion and
pivot turns work in firmware. Maze-solving does not exist yet and is out of
scope until motion primitives are trusted. VL53L0X wall detection is on the
PCB but not yet wired into firmware.

## Repo layout

| Folder | Contents | Buildable/testable? |
|---|---|---|
| `main_controller/` | STM32 firmware (CubeMX + HAL, CMake/Ninja) | Yes — see below |
| `pcb_designs/` | Altium schematics/layouts for the Power and Main PCBs, bring-up test media | No (Altium-only, or PDFs) |
| `mechanical_designs/` | Chassis/mechanical CAD | Placeholder — README only, no files yet |
| `simulations/` | Simulation work | Placeholder — README only, no files yet |
| `tests/` (repo root) | Project-level test resources | Placeholder — README only, no files yet |
| `docs/` | Documentation images | Assets only |

Almost all actual code work happens in `main_controller/`. That folder has its
own detailed **`main_controller/CLAUDE.md`** — read it before touching
firmware. It covers the peripheral map, sign/unit conventions, the EKF-based
control architecture, tuning workflow, hardware quirks, and a maintained
change log. Do not duplicate that content here; this file only covers what's
true across the whole repo.

## Building and testing

Firmware only (`main_controller/`):

```sh
BUNDLES="$LOCALAPPDATA/STM32Cube/bundles"
export PATH="$BUNDLES/gnu-tools-for-stm32/14.3.1+st.2/bin:$BUNDLES/ninja/1.13.2+st.1/bin:$PATH"
cd main_controller
cmake --preset Debug        # only after adding/removing source files
cmake --build --preset Debug
```

Built via the STM32Cube VSCode extension normally; flashing/debugging goes
through an ST-Link (there is no UART — telemetry is read via live-watch, see
`main_controller/CLAUDE.md`). The build must stay warning-free.

Host-side EKF test (run after any change to `EKF.c`, no hardware needed):

```sh
cd main_controller
gcc -O1 -o ekf_test.exe tests/ekf_host_test.c Core/Src/Control/LowLevel/EKF.c \
    -I Core/Inc/Control/LowLevel -lm
./ekf_test.exe
```

Exit code 0 = pass. See `main_controller/tests/README.md` for what each case covers.

## Working across domains

- `pcb_designs/README.md` documents both boards (Power PCB: LiPo → +6V/+3.3V
  bucks; Main PCB: MCU, IMU, 5× VL53L0X behind a TCA9548A I2C mux, DRV8833
  motor drivers) in detail — read it before reasoning about hardware/firmware
  interactions (pin assignments, sensor wiring, power rails).
- Firmware pin/peripheral assumptions in `main_controller/CLAUDE.md` should
  match the Main PCB schematic in `pcb_designs/`; if they diverge, the PCB
  (as fabricated) is ground truth, not the `.ioc` file.
- `mechanical_designs/`, `simulations/`, and the root `tests/` folder are
  currently unused placeholders (README stub only) — don't assume conventions
  for them beyond what's asked.

## Sibling project: the maze-solving algorithm

This repo (`lolita-micromouse`) is the **hardware/firmware side** — PCBs,
mechanical design, and low-level motion control. It intentionally has no
maze-solving code yet.

The parent directory also contains `MicroMouseAlgorithm/` — a separate git
repo with its own GitHub remote — which is the **algorithm side**: a
flood-fill maze-solving algorithm in C, developed and tested against the
`mackorone/mms` desktop simulator rather than on real hardware.

The two are not currently integrated: `MicroMouseAlgorithm`'s `API.c`/`API.h`
talk to the `mms` simulator over stdin/stdout, not to this robot's firmware,
and `main_controller/` has no maze-solving logic. Porting the flood-fill
algorithm from `MicroMouseAlgorithm` onto this robot (replacing the simulator
API calls with real motion primitives from `main_controller/`) is the
expected eventual path once motion primitives here are trusted — but treat
that as future work, not something to assume is already wired up.
