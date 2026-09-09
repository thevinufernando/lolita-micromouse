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

/* ---- SELECT THE TEST TO RUN HERE ---- */
#define ACTIVE_TEST TEST_TURN_LEFT_90

/* ---- Test parameters ---- */
#define TEST_DISTANCE_CM 15.0f    /* Straightline test distance (15 cm) */
#define TEST_ANGLE_DEG 90.0f      /* Turn test angle                   */
#define TEST_SQUARE_SIDE_CM 18.0f /* Square test side length           */
#define TEST_OPEN_LOOP_SPEED 120  /* Open loop test speed (0-255)      */

/* Pause between individual moves, in ms. Lets the chassis settle so each
 * move starts from rest and the encoder reading is unambiguous. */
#define TEST_MOVE_PAUSE_MS 800U

/* Pause between full test cycles, in ms. Long enough to observe the result
 * before the next move starts. */
#define TEST_CYCLE_PAUSE_MS 2500U

/* Number of test cycles to execute before stopping.
 * 5 = Runs 5 moves in sequence, records each into tm_history, then halts with
 * motors braked. 0 = Continuous mode. */
#define TEST_CYCLE_LIMIT 5U

/* How long TEST_YAW_ESTIMATE observes the filter per cycle, in ms. */
#define TEST_YAW_OBSERVE_MS 10000U

/* How long TEST_GYRO_BIAS lets yaw drift before reporting, in ms. */
#define TEST_BIAS_DRIFT_MS 10000U

/* ---------------------------------------------------------------------------
 * Telemetry for the debugger live-watch panel.
 * Add these names to the "liveWatch" expressions list in .vscode/launch.json
 * to observe them while the robot runs.
 * ------------------------------------------------------------------------ */

/* ---- Flight Recorder History Buffer ---- */
#define TM_HISTORY_CAPACITY 50U

typedef struct {
  uint32_t timestamp_ms; /* HAL_GetTick() when move completed */
  float target;          /* target distance (cm) or turn angle (deg) */
  float actual;          /* achieved distance (cm) or turn angle (deg) */
  float error;           /* target - actual */
  int32_t drift_cnt;     /* left - right ticks (skew for straight runs) */
  int32_t left_cnt;      /* left encoder count */
  int32_t right_cnt;     /* right encoder count */
  uint8_t ok;            /* 1 = success, 0 = timeout */
} MoveRecord_t;

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

/* Blink the on-board LED n times to signal progress without a serial port.
 * Shared by the test routines and main()'s own startup indicator. */
void LED_Blink(uint8_t times, uint32_t on_ms, uint32_t off_ms);

/* Run one iteration of the selected test, including its cycle pause.
 * Called from main()'s while(1) loop. */
void TestHarness_RunCycle(void);

#endif /* TEST_HARNESS_H */
