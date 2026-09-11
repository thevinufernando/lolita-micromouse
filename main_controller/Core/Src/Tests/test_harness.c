#include "test_harness.h"

#include "encoders.h"
#include "DRV8833.h"
#include "straightline_controller.h"
#include "turn_controller.h"
#include "ICM42688.h"
#include "tof_sensors.h"

/* Result of the most recent move */
volatile float  tm_final_left_cm    = 0.0f;
volatile float  tm_final_right_cm   = 0.0f;
volatile float  tm_final_avg_cm     = 0.0f;
volatile float  tm_final_error_cm   = 0.0f;
volatile int32_t tm_final_left_cnt  = 0;
volatile int32_t tm_final_right_cnt = 0;
volatile int32_t tm_drift_cnt       = 0;

/* Progress counters */
volatile uint32_t tm_cycle_count    = 0;
volatile uint32_t tm_move_count     = 0;
volatile uint32_t tm_timeout_count  = 0;
volatile uint8_t  tm_last_ok        = 1;

/* Flight recorder history buffer */
volatile MoveRecord_t tm_history[TM_HISTORY_CAPACITY];
volatile uint32_t     tm_history_count = 0;

/* ---- IMU / EKF telemetry ---- */
volatile uint8_t  tm_imu_ok         = 0;
volatile float    tm_gyro_z_dps     = 0.0f;
volatile float    tm_accel_x_g      = 0.0f;
volatile float    tm_accel_y_g      = 0.0f;
volatile float    tm_accel_z_g      = 0.0f;
volatile float    tm_imu_temp_c     = 0.0f;

volatile float    tm_yaw_deg        = 0.0f;
volatile float    tm_yaw_error_deg  = 0.0f;
volatile float    tm_enc_yaw_deg    = 0.0f;
volatile float    tm_fusion_gap_deg = 0.0f;
volatile float    tm_gyro_bias_dps  = 0.0f;
volatile float    tm_yaw_sigma_deg  = 0.0f;
volatile float    tm_bias_drift_deg = 0.0f;
volatile uint32_t tm_ekf_rejects    = 0;

/* ---- ToF telemetry ---- */
volatile uint8_t  tm_tof_ready         = 0;
volatile uint16_t tm_tof_front_mm      = TOF_DISTANCE_INVALID;
volatile uint16_t tm_tof_left_mm       = TOF_DISTANCE_INVALID;
volatile uint16_t tm_tof_right_mm      = TOF_DISTANCE_INVALID;
volatile uint16_t tm_tof_front_raw_mm  = TOF_DISTANCE_INVALID;
volatile uint16_t tm_tof_left_raw_mm   = TOF_DISTANCE_INVALID;
volatile uint16_t tm_tof_right_raw_mm  = TOF_DISTANCE_INVALID;
volatile uint32_t tm_tof_front_jumps   = 0;
volatile uint32_t tm_tof_left_jumps    = 0;
volatile uint32_t tm_tof_right_jumps   = 0;
volatile uint8_t  tm_tof_front_status  = 255;
volatile uint8_t  tm_tof_left_status   = 255;
volatile uint8_t  tm_tof_right_status  = 255;
volatile uint32_t tm_tof_sample_count  = 0;
volatile uint32_t tm_tof_error_count   = 0;

/* Flight recorder for ToF sweeps. See test_harness.h for why the scalars
 * above are not enough to characterise a sensor. */
volatile ToFRecord_t tm_tof_history[TOF_HISTORY_CAPACITY];
volatile uint32_t    tm_tof_history_count = 0;

/* Only the test selected by ACTIVE_TEST is called, so the others would each
 * raise -Wunused-function. Mark them so real warnings stay visible. */
#define TEST_FN __attribute__((unused)) static

/* Blink the on-board LED n times to signal progress without a serial port.
 * Not static: main.c also calls this directly for its startup indicator. */
void LED_Blink(uint8_t times, uint32_t on_ms, uint32_t off_ms)
{
  for (uint8_t i = 0; i < times; i++)
  {
    HAL_GPIO_WritePin(MCU_LED_GPIO_Port, MCU_LED_Pin, GPIO_PIN_SET);
    HAL_Delay(on_ms);
    HAL_GPIO_WritePin(MCU_LED_GPIO_Port, MCU_LED_Pin, GPIO_PIN_RESET);
    HAL_Delay(off_ms);
  }
}

/* Record a completed move into the history buffer.
 * `rate` and `cmd` capture what the controller was doing as the move ended;
 * see MoveRecord_t for why they matter on a failure. */
static void History_Record(float target, float actual, float error,
                           float rate, float cmd,
                           int32_t drift, int32_t l_cnt, int32_t r_cnt, uint8_t ok)
{
  if (tm_history_count < TM_HISTORY_CAPACITY)
  {
    tm_history[tm_history_count].timestamp_ms = HAL_GetTick();
    tm_history[tm_history_count].target       = target;
    tm_history[tm_history_count].actual       = actual;
    tm_history[tm_history_count].error        = error;
    tm_history[tm_history_count].rate_dps     = rate;
    tm_history[tm_history_count].basespeed    = cmd;
    tm_history[tm_history_count].drift_cnt    = drift;
    tm_history[tm_history_count].left_cnt     = l_cnt;
    tm_history[tm_history_count].right_cnt    = r_cnt;
    tm_history[tm_history_count].ok           = ok;
    tm_history_count++;
  }
}

/* Capture the encoder state after a move into the live-watch telemetry. */
static void Telemetry_Capture(float target_cm, uint8_t ok)
{
  Encoders_Update();

  tm_final_left_cm    = Encoder_getLeftDistance();
  tm_final_right_cm   = Encoder_getRightDistance();
  tm_final_avg_cm     = Encoder_getAverageDistance();
  tm_final_error_cm   = target_cm - tm_final_avg_cm;
  tm_final_left_cnt   = Encoder_getLeftCount();
  tm_final_right_cnt  = Encoder_getRightCount();
  tm_drift_cnt        = Encoder_getLeftCount() - Encoder_getRightCount();

  tm_last_ok = ok;
  tm_move_count++;

  if (!ok) tm_timeout_count++;

  /* No rate signal in the straight-line path: it is encoder-only by design. */
  History_Record(target_cm, tm_final_avg_cm, tm_final_error_cm,
                 0.0f, basespeed,
                 tm_drift_cnt, tm_final_left_cnt, tm_final_right_cnt, ok);
}

/* Capture the yaw estimator state and wheel travel after a turn. `target_deg` is
 * the signed commanded angle, so the error is directly readable. */
static void Telemetry_CaptureYaw(float target_deg, uint8_t ok)
{
  Encoders_Update();

  tm_final_left_cm    = Encoder_getLeftDistance();
  tm_final_right_cm   = Encoder_getRightDistance();
  tm_final_avg_cm     = Encoder_getAverageDistance();
  tm_final_error_cm   = 0.0f;
  tm_final_left_cnt   = Encoder_getLeftCount();
  tm_final_right_cnt  = Encoder_getRightCount();
  tm_drift_cnt        = Encoder_getLeftCount() - Encoder_getRightCount();

  tm_last_ok = ok;
  tm_move_count++;

  if (!ok) tm_timeout_count++;

  tm_yaw_deg        = TurnController_GetYawDeg();
  tm_yaw_error_deg  = target_deg - tm_yaw_deg;
  tm_enc_yaw_deg    = turn_encoder_yaw_deg;
  tm_fusion_gap_deg = tm_yaw_deg - turn_encoder_yaw_deg;
  tm_gyro_bias_dps  = TurnController_GetGyroBiasDps();
  tm_ekf_rejects    = turn_reject_count;

  /* Both survive a timeout untouched: the controller brakes and returns
   * without writing either, so they hold the last commanded state. */
  History_Record(target_deg, tm_yaw_deg, tm_yaw_error_deg,
                 turn_gyro_rate_dps, turn_basespeed,
                 tm_drift_cnt, tm_final_left_cnt, tm_final_right_cnt, ok);
}

/* Pause between moves, holding the motors braked. */
static void Test_Pause(uint32_t ms)
{
  Motor_Brake();
  HAL_Delay(ms);
}

/* ---------------------------------------------------------------------------
 * TEST 0: Open loop motor check. No PID involved.
 * Verifies motor wiring, direction polarity and that both wheels spin.
 * Watch tm_final_left_cnt / tm_final_right_cnt: both should grow POSITIVE
 * when driving forward. If one is negative, flip encoder_polarity for that
 * wheel in Core/Src/Encoders/encoders.c.
 * ------------------------------------------------------------------------ */
TEST_FN void Test_MotorsOpenLoop(void)
{
  Encoders_Reset();

  /* Forward */
  LED_Blink(1, 100, 100);
  MotorForward_runSpeed(TEST_OPEN_LOOP_SPEED, TEST_OPEN_LOOP_SPEED);
  HAL_Delay(1000);
  Motor_Brake();
  Telemetry_Capture(0.0f, 1);
  HAL_Delay(TEST_MOVE_PAUSE_MS);

  /* Backward */
  LED_Blink(2, 100, 100);
  Encoders_Reset();
  MotorBackward_runSpeed(TEST_OPEN_LOOP_SPEED, TEST_OPEN_LOOP_SPEED);
  HAL_Delay(1000);
  Motor_Brake();
  Telemetry_Capture(0.0f, 1);
  HAL_Delay(TEST_MOVE_PAUSE_MS);

  /* Pivot left */
  LED_Blink(3, 100, 100);
  Encoders_Reset();
  MotorLeftTurn_runSpeed(TEST_OPEN_LOOP_SPEED);
  HAL_Delay(700);
  Motor_Brake();
  Telemetry_Capture(0.0f, 1);
  HAL_Delay(TEST_MOVE_PAUSE_MS);

  /* Pivot right */
  LED_Blink(4, 100, 100);
  Encoders_Reset();
  MotorRightTurn_runSpeed(TEST_OPEN_LOOP_SPEED);
  HAL_Delay(700);
  Motor_Brake();
  Telemetry_Capture(0.0f, 1);
}

/* ---------------------------------------------------------------------------
 * TEST 1: Encoder check. Motors stay off; push the robot by hand.
 * Both counts should increase when the wheels roll forward.
 * Also use this to verify WHEEL_DIAMETER_MM and the gear ratios: roll the
 * robot exactly 20 cm by hand and confirm tm_final_avg_cm reads ~20.
 * ------------------------------------------------------------------------ */
TEST_FN void Test_EncodersOnly(void)
{
  Motor_Brake();
  Encoders_Update();

  tm_final_left_cm   = Encoder_getLeftDistance();
  tm_final_right_cm  = Encoder_getRightDistance();
  tm_final_avg_cm    = Encoder_getAverageDistance();
  tm_final_left_cnt  = Encoder_getLeftCount();
  tm_final_right_cnt = Encoder_getRightCount();
  tm_drift_cnt       = Encoder_getLeftCount() - Encoder_getRightCount();

  /* Heartbeat so it is obvious the firmware is alive */
  HAL_GPIO_TogglePin(MCU_LED_GPIO_Port, MCU_LED_Pin);
  HAL_Delay(100);
}

/* ---------------------------------------------------------------------------
 * TEST 2: Straightline forward.
 * Tune STRAIGHT_DIST_* for distance accuracy (watch tm_final_error_cm)
 * and STRAIGHT_HEADING_* for straightness (watch tm_drift_cnt, want ~0).
 * ------------------------------------------------------------------------ */
TEST_FN void Test_StraightForward(void)
{
  LED_Blink(1, 150, 150);

  uint8_t ok = runForwardDistance(TEST_DISTANCE_CM);
  Telemetry_Capture(TEST_DISTANCE_CM, ok);
}

/* ---------------------------------------------------------------------------
 * TEST 3: Forward then backward. The robot should return to its start point.
 * Any leftover offset points at asymmetric friction or distance calibration.
 * ------------------------------------------------------------------------ */
TEST_FN void Test_StraightForwardBackward(void)
{
  LED_Blink(1, 150, 150);
  uint8_t ok = runForwardDistance(TEST_DISTANCE_CM);
  Telemetry_Capture(TEST_DISTANCE_CM, ok);

  Test_Pause(TEST_MOVE_PAUSE_MS);

  LED_Blink(2, 150, 150);
  ok = runBackwardDistance(TEST_DISTANCE_CM);
  Telemetry_Capture(-TEST_DISTANCE_CM, ok);
}

/* ---------------------------------------------------------------------------
 * TEST 4/5: Pivot turns. Tune TURN_* here.
 * Mark the floor and measure the real angle against TEST_ANGLE_DEG. If every
 * turn is off by the same ratio, correct ROBOT_WHEEL_BASE_CM rather than the
 * gains: a turn that overshoots means the configured wheel base is too small.
 * ------------------------------------------------------------------------ */
TEST_FN void Test_TurnLeft(void)
{
  LED_Blink(1, 150, 150);

  uint8_t ok = turnLeftAngle(TEST_ANGLE_DEG);
  Telemetry_CaptureYaw(+TEST_ANGLE_DEG, ok);
}

TEST_FN void Test_TurnRight(void)
{
  LED_Blink(2, 150, 150);

  uint8_t ok = turnRightAngle(TEST_ANGLE_DEG);
  Telemetry_CaptureYaw(-TEST_ANGLE_DEG, ok);
}

/* ---------------------------------------------------------------------------
 * TEST 6: Four 90 deg turns in the same direction = one full rotation.
 * Errors accumulate, so this magnifies a small per-turn bias fourfold and is
 * the most sensitive check of turn calibration.
 * ------------------------------------------------------------------------ */
TEST_FN void Test_Turn360(void)
{
  for (uint8_t i = 0; i < 4; i++)
  {
    LED_Blink(1, 100, 100);

    uint8_t ok = turnLeftAngle(90.0f);
    Telemetry_CaptureYaw(+90.0f, ok);

    Test_Pause(TEST_MOVE_PAUSE_MS);
  }
}

/* ---------------------------------------------------------------------------
 * TEST 7: Drive a square. Combines both controllers; the robot should end up
 * back where it started, facing its original heading.
 * ------------------------------------------------------------------------ */
TEST_FN void Test_Square(void)
{
  for (uint8_t i = 0; i < 4; i++)
  {
    LED_Blink(1, 100, 100);

    uint8_t ok = runForwardDistance(TEST_SQUARE_SIDE_CM);
    Telemetry_Capture(TEST_SQUARE_SIDE_CM, ok);

    Test_Pause(TEST_MOVE_PAUSE_MS);

    LED_Blink(2, 100, 100);

    ok = turnRightAngle(90.0f);
    Telemetry_CaptureYaw(-90.0f, ok);

    Test_Pause(TEST_MOVE_PAUSE_MS);
  }
}

/* ---------------------------------------------------------------------------
 * TEST 8: Raw IMU readout. Motors stay off.
 *
 * RUN THIS FIRST after wiring the IMU. Two things to confirm:
 *
 *  1. tm_imu_ok must be 1. If it is 0 the WHO_AM_I check failed: check the
 *     SPI wiring, the IMU_NCS pin, and that CS idles HIGH.
 *
 *  2. GYRO SIGN. Rotate the robot ANTICLOCKWISE (to its left) by hand and
 *     watch tm_gyro_z_dps. It must read POSITIVE. If it reads negative, set
 *     IMU_GYRO_Z_SIGN to -1.0f in control_config.h and rebuild. Every turn
 *     depends on this being right, so do not skip it.
 *
 * With the robot level and still, tm_accel_z_g should read about 1.0 and
 * tm_gyro_z_dps should sit near zero (a bias of a few tenths is normal).
 * ------------------------------------------------------------------------ */
TEST_FN void Test_ImuRaw(void)
{
  Motor_Brake();

  ICM42688_t imu;

  if (ICM42688_ReadData(&imu) == IMU_OK)
  {
    /* Report the raw sensor value, NOT sign-corrected, so the sign check
     * above is meaningful. */
    tm_gyro_z_dps = imu.gz;
    tm_accel_x_g  = imu.ax;
    tm_accel_y_g  = imu.ay;
    tm_accel_z_g  = imu.az;
    tm_imu_temp_c = imu.temperature;
  }

  HAL_GPIO_TogglePin(MCU_LED_GPIO_Port, MCU_LED_Pin);
  HAL_Delay(50);
}

/* ---------------------------------------------------------------------------
 * TEST 9: Yaw estimator check. Motors stay off.
 *
 * Runs the full fusion loop for TEST_YAW_OBSERVE_MS while you rotate the
 * robot by hand, then holds the result so you can read it.
 *
 * Watch:
 *   tm_yaw_deg        fused yaw. Rotate the robot exactly 90 deg
 *                     anticlockwise by hand; this should read about +90.
 *   tm_enc_yaw_deg    encoder-only yaw over the same motion.
 *   tm_fusion_gap_deg how far the fusion moved away from raw odometry.
 *                     Large values mean the wheels slipped and the gyro
 *                     corrected for it, which is the whole point.
 *   tm_ekf_rejects    encoder updates rejected as slip.
 *
 * Return the robot to its starting heading and tm_yaw_deg should come back
 * to roughly zero.
 * ------------------------------------------------------------------------ */
TEST_FN void Test_YawEstimate(void)
{
  LED_Blink(1, 150, 150);

  TurnController_ObserveYaw(TEST_YAW_OBSERVE_MS);

  tm_yaw_deg        = TurnController_GetYawDeg();
  tm_enc_yaw_deg    = turn_encoder_yaw_deg;
  tm_fusion_gap_deg = tm_yaw_deg - turn_encoder_yaw_deg;
  tm_gyro_bias_dps  = TurnController_GetGyroBiasDps();
  tm_ekf_rejects    = turn_reject_count;
}

/* ---------------------------------------------------------------------------
 * TEST 10: Gyro bias and drift. Motors stay off, ROBOT MUST NOT MOVE.
 *
 * Recalibrates the bias, then lets the estimator run untouched for
 * TEST_BIAS_DRIFT_MS and reports how far yaw wandered.
 *
 * tm_bias_drift_deg is the headline number: total yaw drift over the window
 * while perfectly stationary. Under ~1 deg per 10 s is healthy. If it is
 * much worse, the bias calibration was taken while the robot was moving, or
 * EKF_Q_BIAS needs raising so the filter tracks bias more aggressively.
 * ------------------------------------------------------------------------ */
TEST_FN void Test_GyroBias(void)
{
  Motor_Brake();

  LED_Blink(2, 150, 150);

  /* Recalibrate from rest, then measure what leaks through. */
  TurnController_CalibrateGyroBias();
  TurnController_ResetYaw();

  TurnController_ObserveYaw(TEST_BIAS_DRIFT_MS);

  tm_bias_drift_deg = TurnController_GetYawDeg();
  tm_gyro_bias_dps  = TurnController_GetGyroBiasDps();
  tm_enc_yaw_deg    = turn_encoder_yaw_deg;
  tm_ekf_rejects    = turn_reject_count;
}

/* Publish one full sweep to the live-watch globals.
 * Shared by both ToF tests so single and continuous mode report identically
 * and their numbers can be compared directly. */
static void Telemetry_CaptureToF(const ToF_Measurement_t m[TOF_SENSOR_COUNT],
                                 int sweep_status)
{
  tm_tof_front_mm     = m[TOF_FRONT].distance_mm;
  tm_tof_left_mm      = m[TOF_LEFT].distance_mm;
  tm_tof_right_mm     = m[TOF_RIGHT].distance_mm;

  tm_tof_front_raw_mm = m[TOF_FRONT].raw_mm;
  tm_tof_left_raw_mm  = m[TOF_LEFT].raw_mm;
  tm_tof_right_raw_mm = m[TOF_RIGHT].raw_mm;

  tm_tof_front_jumps  = ToF_GetFilterJumpCount(TOF_FRONT);
  tm_tof_left_jumps   = ToF_GetFilterJumpCount(TOF_LEFT);
  tm_tof_right_jumps  = ToF_GetFilterJumpCount(TOF_RIGHT);

  tm_tof_front_status = m[TOF_FRONT].range_status;
  tm_tof_left_status  = m[TOF_LEFT].range_status;
  tm_tof_right_status = m[TOF_RIGHT].range_status;

  if (sweep_status == TOF_OK)
  {
    tm_tof_sample_count++;
  }
  else
  {
    tm_tof_error_count++;
  }

  /* Append to the flight recorder. Stops at capacity rather than wrapping:
   * a wrapped buffer read afterwards would silently mix the start and end of
   * the run, and for a stationary noise measurement the first samples are the
   * interesting ones (they include the filter priming). */
  if (tm_tof_history_count < TOF_HISTORY_CAPACITY)
  {
    volatile ToFRecord_t *r = &tm_tof_history[tm_tof_history_count];

    r->timestamp_ms  = HAL_GetTick();
    r->front_mm      = m[TOF_FRONT].distance_mm;
    r->left_mm       = m[TOF_LEFT].distance_mm;
    r->right_mm      = m[TOF_RIGHT].distance_mm;
    r->front_raw_mm  = m[TOF_FRONT].raw_mm;
    r->left_raw_mm   = m[TOF_LEFT].raw_mm;
    r->right_raw_mm  = m[TOF_RIGHT].raw_mm;
    r->front_status  = m[TOF_FRONT].range_status;
    r->left_status   = m[TOF_LEFT].range_status;
    r->right_status  = m[TOF_RIGHT].range_status;
    r->ok            = (sweep_status == TOF_OK) ? 1U : 0U;

    tm_tof_history_count++;
  }
}

/* Park the robot once the recorder is full so the samples survive until they
 * are read. Mirrors what TEST_CYCLE_LIMIT does for the motion tests: the ToF
 * tests loop forever by design, and without this the run would keep sampling
 * into a full buffer while the LED kept blinking as if it were still working.
 * Heartbeat blink (100 ms on, 900 ms off) means "done, go read it". */
static void ToF_HaltIfBufferFull(void)
{
  if (tm_tof_history_count < TOF_HISTORY_CAPACITY)
  {
    return;
  }

  Motor_Brake();

  while (1)
  {
    HAL_GPIO_WritePin(MCU_LED_GPIO_Port, MCU_LED_Pin, GPIO_PIN_SET);
    HAL_Delay(100);
    HAL_GPIO_WritePin(MCU_LED_GPIO_Port, MCU_LED_Pin, GPIO_PIN_RESET);
    HAL_Delay(900);
  }
}

/* Which sensors survived init, as a bitmask. Read this FIRST: an all-zero
 * mask means the mux itself never answered, which is a wiring/address
 * problem, not a sensor problem.
 *
 * Not static: main() calls it once after ToF_Init() so the mask is visible in
 * live-watch even when a non-ToF test is selected. */
void TestHarness_CaptureToFReady(void)
{
  tm_tof_ready = (uint8_t)((ToF_IsSensorReady(TOF_FRONT) ? 0x01U : 0x00U) |
                           (ToF_IsSensorReady(TOF_LEFT)  ? 0x02U : 0x00U) |
                           (ToF_IsSensorReady(TOF_RIGHT) ? 0x04U : 0x00U));
}

/* Single-shot ranging. Run this first after wiring the sensors: it is the
 * simplest path that exercises mux -> sensor -> distance, and each reading is
 * triggered by us so nothing is stale.
 *
 * Hold a hand or a wall at a known distance in front of each sensor and check
 * the matching tm_tof_*_mm against a ruler. A sensor reading a plausible
 * distance for the WRONG direction means the TOF_CHANNEL_* mapping in
 * control_config.h does not match the PCB. */
TEST_FN void Test_ToFSingle(void)
{
  Motor_Brake();

  ToF_Measurement_t m[TOF_SENSOR_COUNT];
  int status = ToF_ReadAll(m);

  Telemetry_CaptureToF(m, status);
  ToF_HaltIfBufferFull();

  HAL_GPIO_TogglePin(MCU_LED_GPIO_Port, MCU_LED_Pin);
  HAL_Delay(100);
}

/* Continuous ranging. Same readings, but the sensors free-run and each poll
 * returns the newest completed measurement without waiting.
 *
 * Started once on the first call rather than in ToF_Init(), so that selecting
 * this test is all it takes to switch modes. Expect tm_tof_error_count to
 * climb faster here than in the single-shot test: polling faster than
 * TOF_INTER_MEASUREMENT_MS legitimately returns "no new data yet". Rising
 * counts alongside a static tm_tof_sample_count is the real fault signal. */
TEST_FN void Test_ToFContinuous(void)
{
  static uint8_t started = 0;

  Motor_Brake();

  if (!started)
  {
    (void)ToF_StartContinuousAll();
    started = 1;
  }

  ToF_Measurement_t m[TOF_SENSOR_COUNT];
  int status = ToF_ReadAll(m);

  Telemetry_CaptureToF(m, status);
  ToF_HaltIfBufferFull();

  HAL_GPIO_TogglePin(MCU_LED_GPIO_Port, MCU_LED_Pin);
  HAL_Delay(20);
}

void TestHarness_RunCycle(void)
{
#if   (ACTIVE_TEST == TEST_MOTORS_OPEN_LOOP)
  Test_MotorsOpenLoop();

#elif (ACTIVE_TEST == TEST_ENCODERS_ONLY)
  Test_EncodersOnly();
  return;   /* poll continuously, no cycle pause */

#elif (ACTIVE_TEST == TEST_STRAIGHT_FORWARD)
  Test_StraightForward();

#elif (ACTIVE_TEST == TEST_STRAIGHT_FWD_BACK)
  Test_StraightForwardBackward();

#elif (ACTIVE_TEST == TEST_TURN_LEFT_90)
  Test_TurnLeft();

#elif (ACTIVE_TEST == TEST_TURN_RIGHT_90)
  Test_TurnRight();

#elif (ACTIVE_TEST == TEST_TURN_360)
  Test_Turn360();

#elif (ACTIVE_TEST == TEST_SQUARE)
  Test_Square();

#elif (ACTIVE_TEST == TEST_IMU_RAW)
  Test_ImuRaw();
  return;   /* poll continuously, no cycle pause */

#elif (ACTIVE_TEST == TEST_YAW_ESTIMATE)
  Test_YawEstimate();

#elif (ACTIVE_TEST == TEST_GYRO_BIAS)
  Test_GyroBias();

#elif (ACTIVE_TEST == TEST_TOF_SINGLE)
  Test_ToFSingle();
  return;   /* poll continuously, no cycle pause */

#elif (ACTIVE_TEST == TEST_TOF_CONTINUOUS)
  Test_ToFContinuous();
  return;   /* poll continuously, no cycle pause */

#else
  #error "ACTIVE_TEST is not set to a valid test id"
#endif

  tm_cycle_count++;

  /* If a cycle limit is set and reached, stop the robot indefinitely */
  if (TEST_CYCLE_LIMIT > 0 && tm_cycle_count >= TEST_CYCLE_LIMIT)
  {
    Motor_Brake();
    while (1)
    {
      /* Heartbeat blink (100ms on, 900ms off) indicates test is complete */
      HAL_GPIO_WritePin(MCU_LED_GPIO_Port, MCU_LED_Pin, GPIO_PIN_SET);
      HAL_Delay(100);
      HAL_GPIO_WritePin(MCU_LED_GPIO_Port, MCU_LED_Pin, GPIO_PIN_RESET);
      HAL_Delay(900);
    }
  }

  /* Idle between cycles with the motors braked, so you can reposition the
   * robot and read the telemetry before the next run starts. */
  Test_Pause(TEST_CYCLE_PAUSE_MS);
}
