#ifndef TEST_HARNESS_H
#define TEST_HARNESS_H

#include "main.h"

/*
 * ============================================================================
 *                          ON-TARGET TEST HARNESS
 * ============================================================================
 * All bring-up test routines and their live-watch telemetry, pulled out of
 * main.c so that file can stay focused on peripheral/hardware bring-up (the
 * part CubeMX and the STM32Cube extension care about). Nothing here touches
 * CubeMX-generated code, so it is safe to edit freely and regenerate the
 * .ioc-driven parts of the project without affecting it.
 *
 * Usage: set ACTIVE_TEST below, rebuild, flash. main() calls
 * TestHarness_RunCycle() once per loop iteration; which routine actually runs
 * is resolved entirely in test_harness.c.
 * ============================================================================
 */

#define TEST_MOTORS_OPEN_LOOP 0  /* No PID. Checks wiring and polarity.   */
#define TEST_ENCODERS_ONLY 1     /* No motion. Push the robot by hand.    */
#define TEST_STRAIGHT_FORWARD 2  /* Repeated forward runs.                */
#define TEST_STRAIGHT_FWD_BACK 3 /* Forward then back to the start.       */
#define TEST_TURN_LEFT_90 4      /* Repeated 90 deg left pivots.          */
#define TEST_TURN_RIGHT_90 5     /* Repeated 90 deg right pivots.         */
#define TEST_TURN_360 6          /* Full rotation. Best turn accuracy check. */
#define TEST_SQUARE 7            /* Straight + turn combined.             */
#define TEST_IMU_RAW 8           /* No motion. Raw IMU + gyro sign check. */
#define TEST_YAW_ESTIMATE 9      /* No motion. Rotate by hand, watch EKF. */
#define TEST_GYRO_BIAS 10        /* No motion. Bias + drift measurement.  */
#define TEST_TOF_SINGLE 11       /* No motion. Single-shot ToF ranging.   */
#define TEST_TOF_CONTINUOUS 12   /* No motion. Continuous ToF ranging.    */
#define TEST_TOF_MODE_CYCLE 13   /* No motion. Mode switching + stop path. */
#define TEST_MAZE_RUN 14         /* MOVES. Scripted arena run + wall map.  */

/* ---- SELECT THE TEST TO RUN HERE ---- */
#define ACTIVE_TEST TEST_MAZE_RUN

/* ---- Test parameters ---- */
#define TEST_DISTANCE_CM 30.0f    /* Straightline test distance          */
#define TEST_ANGLE_DEG 90.0f      /* Turn test angle                   */
#define TEST_SQUARE_SIDE_CM 50.0f /* Square test side length           */
#define TEST_OPEN_LOOP_SPEED 120  /* Open loop test speed (0-255)      */

/* Pause between individual moves, in ms. Lets the chassis settle so each
 * move starts from rest and the encoder reading is unambiguous. */
#define TEST_MOVE_PAUSE_MS 800U

/* Pause between full test cycles, in ms. Long enough to observe the result
 * before the next move starts. */
#define TEST_CYCLE_PAUSE_MS 2500U

/* Number of test cycles to execute before stopping. Each cycle records into
 * tm_history; on reaching the limit the robot halts with motors braked.
 *
 * !! 0 MEANS RUN FOREVER, AND THAT IS DANGEROUS FOR THE CUMULATIVE TESTS !!
 * TEST_STRAIGHT_FORWARD, TEST_TURN_LEFT_90 and TEST_TURN_RIGHT_90 do not
 * return to where they started, so every cycle moves the robot further from
 * its starting point. Five forward cycles at TEST_DISTANCE_CM = 30 is 150 cm
 * of travel, which is more maze than you have. Use 0 only for the stationary
 * tests (IMU, gyro bias, ToF), or for TEST_STRAIGHT_FWD_BACK and TEST_SQUARE,
 * which end roughly where they began.
 *
 * Set to 1 for a single move, then reposition and power-cycle to repeat. */
#define TEST_CYCLE_LIMIT 5U

/* How long TEST_YAW_ESTIMATE observes the filter per cycle, in ms. */
#define TEST_YAW_OBSERVE_MS 10000U

/* How long TEST_GYRO_BIAS lets yaw drift before reporting, in ms. */
#define TEST_BIAS_DRIFT_MS 10000U

/* Continuous polls per cycle of TEST_TOF_MODE_CYCLE. Only needs to be enough
 * for the sensor to be genuinely free-running when the stop is issued. */
#define TEST_TOF_CONT_POLLS 5U

/* ---- TEST_MAZE_RUN ----
 * The arena: (0,0)N -> (0,1)N -> (0,2)N -> turn right -> (1,2)E.
 * Three forward moves and one right turn, reading walls at every cell centre
 * INCLUDING the start, because the map has to know about the cell it begins
 * in as well as the ones it drives to. */
#define TEST_MAZE_CELL_CM 18.0f   /* centre-to-centre cell pitch */

/* Per-cell record of what the run actually did. */
#define MAZE_TRACE_CAPACITY 12U

typedef struct {
  uint32_t timestamp_ms;
  int16_t x;             /* pose when the walls were read */
  int16_t y;
  uint8_t dir;           /* Direction enum                */
  uint8_t front;         /* walls seen, robot-relative    */
  uint8_t left;
  uint8_t right;
  uint16_t front_mm;     /* distances behind those calls  */
  uint16_t left_mm;
  uint16_t right_mm;
  float yaw_deg;         /* fused heading at the cell     */
  float heading_target;  /* what it should have been      */
  float move_error_cm;   /* distance error of the move in */
  uint8_t move_ok;       /* 1 = the move that got here completed */
  uint8_t wall_side;     /* WallFollowSide_t in use       */
} MazeTrace_t;

_Static_assert(sizeof(MazeTrace_t) == 36,
               "MazeTrace_t stride changed: update the SWD telemetry reader");

extern volatile MazeTrace_t tm_maze_trace[MAZE_TRACE_CAPACITY];
extern volatile uint32_t    tm_maze_trace_count;
extern volatile uint8_t     tm_maze_complete;

/* ---------------------------------------------------------------------------
 * Telemetry for the debugger live-watch panel.
 * Add these names to the "liveWatch" expressions list in .vscode/launch.json
 * to observe them while the robot runs.
 * ------------------------------------------------------------------------ */

/* ---- Flight Recorder History Buffer ---- */
#define TM_HISTORY_CAPACITY 50U

/* NOTE: changing the layout of this struct changes the stride the offline
 * SWD telemetry reader walks the buffer with. Update both together. */
typedef struct {
  uint32_t timestamp_ms; /* HAL_GetTick() when move completed */
  float target;          /* target distance (cm) or turn angle (deg) */
  float actual;          /* achieved distance (cm) or turn angle (deg) */
  float error;           /* target - actual */

  /* State AT THE MOMENT THE MOVE ENDED. These exist to make a failed move
   * self-diagnosing: on a timeout the controller brakes and returns without
   * touching either value, so they preserve whatever it was doing when the
   * clock ran out. Rate near zero with a large command means the robot was
   * STALLED and could not break static friction. A large rate means it was
   * HUNTING, swinging through the target too fast to satisfy the settle
   * test. The two failures want opposite fixes, and without this the only
   * way to tell them apart was to catch a failure as the last move of a run
   * and read the live registers before they were overwritten. */
  float rate_dps;  /* fused rotation rate, deg/s (turns only) */
  float basespeed; /* last commanded speed, motor units */

  int32_t drift_cnt; /* left - right ticks (skew for straight runs) */
  int32_t left_cnt;  /* left encoder count */
  int32_t right_cnt; /* right encoder count */
  uint8_t ok;        /* 1 = success, 0 = timeout */
} MoveRecord_t;

/* The offline reader walks tm_history by raw byte stride over SWD, so a
 * layout change here silently turns every decoded field into garbage rather
 * than failing. Fail the build instead. */
_Static_assert(sizeof(MoveRecord_t) == 40,
               "MoveRecord_t stride changed: update the SWD telemetry reader");

extern volatile MoveRecord_t tm_history[TM_HISTORY_CAPACITY];
extern volatile uint32_t tm_history_count;

/* Result of the most recent move */
extern volatile float tm_final_left_cm;     /* left wheel travel, cm       */
extern volatile float tm_final_right_cm;    /* right wheel travel, cm      */
extern volatile float tm_final_avg_cm;      /* average travel, cm          */
extern volatile float tm_final_error_cm;    /* target - achieved, cm       */
extern volatile int32_t tm_final_left_cnt;  /* left encoder ticks          */
extern volatile int32_t tm_final_right_cnt; /* right encoder ticks         */
extern volatile int32_t tm_drift_cnt;       /* left - right ticks (skew)   */

/* Progress counters */
extern volatile uint32_t tm_cycle_count;   /* completed test cycles       */
extern volatile uint32_t tm_move_count;    /* completed moves             */
extern volatile uint32_t tm_timeout_count; /* moves that hit the timeout  */
extern volatile uint8_t tm_last_ok;        /* 1 = success, 0 = timeout    */

/* ---- IMU / EKF telemetry ---- */
extern volatile uint8_t tm_imu_ok;   /* 1 = IMU up, 0 = enc only    */
extern volatile float tm_gyro_z_dps; /* raw gyro Z, deg/s           */
extern volatile float tm_accel_x_g;
extern volatile float tm_accel_y_g;
extern volatile float tm_accel_z_g;
extern volatile float tm_imu_temp_c;

extern volatile float tm_yaw_deg;        /* fused yaw after the move    */
extern volatile float tm_yaw_error_deg;  /* target - fused, deg         */
extern volatile float tm_enc_yaw_deg;    /* encoder-only yaw, deg       */
extern volatile float tm_fusion_gap_deg; /* fused - encoder, deg        */
extern volatile float tm_gyro_bias_dps;  /* EKF bias estimate, deg/s    */
extern volatile float tm_yaw_sigma_deg;  /* EKF yaw 1-sigma, deg        */
extern volatile float tm_bias_drift_deg; /* yaw drift while stationary  */
extern volatile uint32_t tm_ekf_rejects; /* gated-out encoder updates   */

/* Which phase of a test produced a ToF record. Only TEST_TOF_MODE_CYCLE uses
 * more than one; the other two tag every record with their own constant so the
 * column always means the same thing. */
#define TOF_PHASE_SINGLE 0U     /* TEST_TOF_SINGLE                          */
#define TOF_PHASE_CONTINUOUS 1U /* TEST_TOF_CONTINUOUS                      */
#define TOF_PHASE_CYCLE_PRE 2U  /* mode cycle: single-shot BEFORE the start */
#define TOF_PHASE_CYCLE_CONT 3U /* mode cycle: free-running                 */
#define TOF_PHASE_CYCLE_POST 4U /* mode cycle: single-shot AFTER the stop   */

/* ---- ToF telemetry ----
 * Distances are in MILLIMETRES (the ST API's native unit), not the cm used
 * by the motion controllers. TOF_DISTANCE_INVALID (0xFFFF = 65535) means the
 * reading is not usable -- check the matching status/valid field to find out
 * why before assuming the sensor is broken. */
extern volatile uint8_t tm_tof_ready; /* bit0 front, bit1 left, bit2 right */
extern volatile uint16_t tm_tof_front_mm; /* filtered (what code should use) */
extern volatile uint16_t tm_tof_left_mm;
extern volatile uint16_t tm_tof_right_mm;

/* Unfiltered, offset-corrected. Watch these ALONGSIDE the filtered values to
 * see the filter working: raw should visibly jitter while the filtered value
 * sits still. If both jitter equally the filter is not engaging; if the
 * filtered value lags a moving target badly, lower TOF_FILTER_EMA_ALPHA's
 * effect by raising it toward 1.0. */
extern volatile uint16_t tm_tof_front_raw_mm;
extern volatile uint16_t tm_tof_left_raw_mm;
extern volatile uint16_t tm_tof_right_raw_mm;

/* Step-jumps the filter has snapped to, per sensor. With the robot stationary
 * these must NOT climb -- if they do, TOF_FILTER_JUMP_THRESHOLD_MM is below
 * the noise floor and the smoothing is being defeated. */
extern volatile uint32_t tm_tof_front_jumps;
extern volatile uint32_t tm_tof_left_jumps;
extern volatile uint32_t tm_tof_right_jumps;
extern volatile uint8_t tm_tof_front_status; /* raw ST RangeStatus, 0 = good */
extern volatile uint8_t tm_tof_left_status;
extern volatile uint8_t tm_tof_right_status;
extern volatile uint32_t tm_tof_sample_count; /* successful full sweeps      */
extern volatile uint32_t tm_tof_error_count;  /* sweeps with any bad reading */

/* ---- ToF flight recorder ----
 *
 * The tm_tof_* scalars above hold only the MOST RECENT sweep, which is fine
 * for watching a live value but useless for characterising a sensor: noise,
 * bias and filter behaviour are all properties of a SERIES, and reading one
 * scalar over SWD gives one aliased sample per second at best.
 *
 * This buffer records every sweep so a whole run can be pulled off the target
 * afterwards and analysed as a population, exactly as tm_history does for
 * moves. Filling it halts the test with the motors braked so the samples are
 * still there when you go to read them.
 *
 * Both filtered and raw are stored per sensor. Comparing their spreads is the
 * only direct way to see whether the filter is earning its place. */
#define TOF_HISTORY_CAPACITY 200U

typedef struct {
  uint32_t timestamp_ms; /* HAL_GetTick() at capture                  */
  uint16_t front_mm;     /* filtered, TOF_DISTANCE_INVALID if unusable */
  uint16_t left_mm;
  uint16_t right_mm;
  uint16_t front_raw_mm; /* offset-corrected, unfiltered               */
  uint16_t left_raw_mm;
  uint16_t right_raw_mm;
  uint8_t front_status; /* raw ST RangeStatus, 0 = good               */
  uint8_t left_status;
  uint8_t right_status;
  uint8_t ok;    /* 1 = all three sensors gave a valid reading */
  uint8_t phase; /* TOF_PHASE_*, so records can be grouped     */
} ToFRecord_t;

/* Same stride contract as MoveRecord_t: the offline reader walks this by raw
 * byte offset, so a layout change must fail the build, not the analysis. */
_Static_assert(sizeof(ToFRecord_t) == 24,
               "ToFRecord_t stride changed: update the SWD telemetry reader");

extern volatile ToFRecord_t tm_tof_history[TOF_HISTORY_CAPACITY];
extern volatile uint32_t tm_tof_history_count;

/* TEST_TOF_MODE_CYCLE counters. A stop that reports failure, or a single-shot
 * read after a stop that does not succeed, is the defect this test exists to
 * catch. Both must stay at 0. */
extern volatile uint32_t tm_tof_mode_cycles; /* completed start/stop cycles */
extern volatile uint32_t
    tm_tof_stop_fail_count; /* ToF_StopContinuousAll != OK */
extern volatile uint32_t
    tm_tof_post_stop_fail; /* single-shot after stop failed */

/* Blink the on-board LED n times to signal progress without a serial port.
 * Shared by the test routines and main()'s own startup indicator. */
void LED_Blink(uint8_t times, uint32_t on_ms, uint32_t off_ms);

/* Latch which ToF sensors came up into tm_tof_ready. Called by main() once
 * after ToF_Init(), so the mask is readable regardless of ACTIVE_TEST. */
void TestHarness_CaptureToFReady(void);

/* Run one iteration of the selected test, including its cycle pause.
 * Called from main()'s while(1) loop. */
void TestHarness_RunCycle(void);

#endif /* TEST_HARNESS_H */
