#include "test_harness.h"

#include "encoders.h"
#include "DRV8833.h"
#include "straightline_controller.h"
#include "turn_controller.h"
#include "ICM42688.h"

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
}

/* Capture the yaw estimator state after a turn. `target_deg` is the signed
 * commanded angle, so the error is directly readable. */
static void Telemetry_CaptureYaw(float target_deg)
{
  tm_yaw_deg        = TurnController_GetYawDeg();
  tm_yaw_error_deg  = target_deg - tm_yaw_deg;
  tm_enc_yaw_deg    = turn_encoder_yaw_deg;
  tm_fusion_gap_deg = tm_yaw_deg - turn_encoder_yaw_deg;
  tm_gyro_bias_dps  = TurnController_GetGyroBiasDps();
  tm_ekf_rejects    = turn_reject_count;
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
  Telemetry_Capture(0.0f, ok);
  Telemetry_CaptureYaw(+TEST_ANGLE_DEG);
}

TEST_FN void Test_TurnRight(void)
{
  LED_Blink(2, 150, 150);

  uint8_t ok = turnRightAngle(TEST_ANGLE_DEG);
  Telemetry_Capture(0.0f, ok);
  Telemetry_CaptureYaw(-TEST_ANGLE_DEG);
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
    Telemetry_Capture(0.0f, ok);
    Telemetry_CaptureYaw(+90.0f);

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
    Telemetry_Capture(0.0f, ok);
    Telemetry_CaptureYaw(-90.0f);

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

#else
  #error "ACTIVE_TEST is not set to a valid test id"
#endif

  tm_cycle_count++;

  /* Idle between cycles with the motors braked, so you can reposition the
   * robot and read the telemetry before the next run starts. */
  Test_Pause(TEST_CYCLE_PAUSE_MS);
}
