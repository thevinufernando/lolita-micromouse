/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "encoders.h"
#include "DRV8833.h"
#include "PID.h"
#include "EKF.h"
#include "control_config.h"
#include "straightline_controller.h"
#include "turn_controller.h"
#include "ICM42688.h"
#include "dwt_timer.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ==========================================================================
 *                          TEST SELECTION
 * ==========================================================================
 * Pick exactly one test by setting ACTIVE_TEST below, rebuild and flash.
 * PID gains for each test live in Core/Inc/Control/LowLevel/control_config.h
 * ==========================================================================
 */

#define TEST_MOTORS_OPEN_LOOP   0   /* No PID. Checks wiring and polarity.   */
#define TEST_ENCODERS_ONLY      1   /* No motion. Push the robot by hand.    */
#define TEST_STRAIGHT_FORWARD   2   /* Repeated forward runs.                */
#define TEST_STRAIGHT_FWD_BACK  3   /* Forward then back to the start.       */
#define TEST_TURN_LEFT_90       4   /* Repeated 90 deg left pivots.          */
#define TEST_TURN_RIGHT_90      5   /* Repeated 90 deg right pivots.         */
#define TEST_TURN_360           6   /* Full rotation. Best turn accuracy check. */
#define TEST_SQUARE             7   /* Straight + turn combined.             */
#define TEST_IMU_RAW            8   /* No motion. Raw IMU + gyro sign check. */
#define TEST_YAW_ESTIMATE       9   /* No motion. Rotate by hand, watch EKF. */
#define TEST_GYRO_BIAS          10  /* No motion. Bias + drift measurement.  */

/* ---- SELECT THE TEST TO RUN HERE ---- */
#define ACTIVE_TEST             TEST_TURN_LEFT_90

/* ---- Test parameters ---- */
#define TEST_DISTANCE_CM        100.0f   /* Straightline test distance        */
#define TEST_ANGLE_DEG          90.0f   /* Turn test angle                   */
#define TEST_SQUARE_SIDE_CM     18.0f   /* Square test side length           */
#define TEST_OPEN_LOOP_SPEED    120    /* Open loop test speed (0-255)      */

/* Pause between individual moves, in ms. Lets the chassis settle so each
 * move starts from rest and the encoder reading is unambiguous. */
#define TEST_MOVE_PAUSE_MS      800U

/* Pause between full test cycles, in ms. Long enough to reposition the robot. */
#define TEST_CYCLE_PAUSE_MS     3000U

/* How long TEST_YAW_ESTIMATE observes the filter per cycle, in ms. */
#define TEST_YAW_OBSERVE_MS     10000U

/* How long TEST_GYRO_BIAS lets yaw drift before reporting, in ms. */
#define TEST_BIAS_DRIFT_MS      10000U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

/* ---------------------------------------------------------------------------
 * Telemetry for the debugger live-watch panel.
 * Add these names to the "liveWatch" expressions list in .vscode/launch.json
 * to observe them while the robot runs.
 * ------------------------------------------------------------------------ */

/* Result of the most recent move */
volatile float  tm_final_left_cm    = 0.0f;   /* left wheel travel, cm       */
volatile float  tm_final_right_cm   = 0.0f;   /* right wheel travel, cm      */
volatile float  tm_final_avg_cm     = 0.0f;   /* average travel, cm          */
volatile float  tm_final_error_cm   = 0.0f;   /* target - achieved, cm       */
volatile int32_t tm_final_left_cnt  = 0;      /* left encoder ticks          */
volatile int32_t tm_final_right_cnt = 0;      /* right encoder ticks         */
volatile int32_t tm_drift_cnt       = 0;      /* left - right ticks (skew)   */

/* Progress counters */
volatile uint32_t tm_cycle_count    = 0;      /* completed test cycles       */
volatile uint32_t tm_move_count     = 0;      /* completed moves             */
volatile uint32_t tm_timeout_count  = 0;      /* moves that hit the timeout  */
volatile uint8_t  tm_last_ok        = 1;      /* 1 = success, 0 = timeout    */

/* ---- IMU / EKF telemetry ---- */
volatile uint8_t  tm_imu_ok         = 0;      /* 1 = IMU up, 0 = enc only    */
volatile float    tm_gyro_z_dps     = 0.0f;   /* raw gyro Z, deg/s           */
volatile float    tm_accel_x_g      = 0.0f;
volatile float    tm_accel_y_g      = 0.0f;
volatile float    tm_accel_z_g      = 0.0f;
volatile float    tm_imu_temp_c     = 0.0f;

volatile float    tm_yaw_deg        = 0.0f;   /* fused yaw after the move    */
volatile float    tm_yaw_error_deg  = 0.0f;   /* target - fused, deg         */
volatile float    tm_enc_yaw_deg    = 0.0f;   /* encoder-only yaw, deg       */
volatile float    tm_fusion_gap_deg = 0.0f;   /* fused - encoder, deg        */
volatile float    tm_gyro_bias_dps  = 0.0f;   /* EKF bias estimate, deg/s    */
volatile float    tm_yaw_sigma_deg  = 0.0f;   /* EKF yaw 1-sigma, deg        */
volatile float    tm_bias_drift_deg = 0.0f;   /* yaw drift while stationary  */
volatile uint32_t tm_ekf_rejects    = 0;      /* gated-out encoder updates   */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Only the test selected by ACTIVE_TEST is called, so the others would each
 * raise -Wunused-function. Mark them so real warnings stay visible. */
#define TEST_FN __attribute__((unused)) static

/* Blink the on-board LED n times to signal progress without a serial port. */
static void LED_Blink(uint8_t times, uint32_t on_ms, uint32_t off_ms)
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

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  /* Start the quadrature encoder interfaces. CubeMX configures the timers but
   * does not start them, so without this the counters never move. */
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);   /* right encoder */
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);   /* left encoder  */

  /* Start the four motor PWM channels (2 per motor, IN/IN drive mode). */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);   /* RM_PWM_INA */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);   /* RM_PWM_INB */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);   /* LM_PWM_INA */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);   /* LM_PWM_INB */

  /* Make sure nothing is driven until a test asks for it. */
  Motor_Brake();

  /* Microsecond timebase for the EKF prediction step. Must come before any
   * controller init, since TurnController_Init() times its gyro calibration
   * with it. */
  DWT_Timer_Init();

  /* CubeMX drives IMU_NCS LOW in MX_GPIO_Init(), but SPI chip select must
   * IDLE HIGH or the first transaction is framed wrong. Deassert it before
   * talking to the IMU. */
  HAL_GPIO_WritePin(IMU_NCS_GPIO_Port, IMU_NCS_Pin, GPIO_PIN_SET);
  HAL_Delay(50);

  /* Bring up both controllers. Each pulls its gains from control_config.h.
   * Both call Encoders_Init() and MotorDriver_Enable() internally, which is
   * idempotent, so initialising both here is safe.
   *
   * TurnController_Init() also brings up the IMU and runs the stationary gyro
   * bias calibration, so THE ROBOT MUST BE STILL AND LEVEL AT POWER-ON. */
  StraightlineController_Init();
  tm_imu_ok = TurnController_Init();

  /* Startup indication.
   *   3 slow blinks  = IMU up, fusion active
   *   6 fast blinks  = IMU not found, running encoder-only */
  if (tm_imu_ok) {
    LED_Blink(3, 300, 300);
  }
  else {
    LED_Blink(6, 80, 80);
  }

  tm_gyro_bias_dps = TurnController_GetGyroBiasDps();

  /* Settling delay so the robot is not already moving when you take your
   * hand off it. */
  HAL_Delay(2000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

#if   (ACTIVE_TEST == TEST_MOTORS_OPEN_LOOP)
    Test_MotorsOpenLoop();

#elif (ACTIVE_TEST == TEST_ENCODERS_ONLY)
    Test_EncodersOnly();
    continue;   /* poll continuously, no cycle pause */

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
    continue;   /* poll continuously, no cycle pause */

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
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 4799;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MCU_LED_GPIO_Port, MCU_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(IMU_NCS_GPIO_Port, IMU_NCS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DRV_STBY_GPIO_Port, DRV_STBY_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : MCU_LED_Pin */
  GPIO_InitStruct.Pin = MCU_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MCU_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : IMU_NCS_Pin */
  GPIO_InitStruct.Pin = IMU_NCS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(IMU_NCS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DRV_STBY_Pin */
  GPIO_InitStruct.Pin = DRV_STBY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DRV_STBY_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
