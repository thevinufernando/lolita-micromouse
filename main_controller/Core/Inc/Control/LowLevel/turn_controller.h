#ifndef TURN_CONTROLLER_H
#define TURN_CONTROLLER_H

#include "PID.h"
#include "EKF.h"
#include "encoders.h"
#include "DRV8833.h"
#include "ICM42688.h"
#include "dwt_timer.h"
#include "control_config.h"
#include <math.h>
#include "main.h"

/*
 * ============================================================================
 *              PIVOT TURN CONTROLLER - IMU + ENCODER SENSOR FUSION
 * ============================================================================
 *
 * Closes the loop on FUSED YAW IN DEGREES produced by the EKF in EKF.c, which
 * combines the ICM-42688-P Z gyro with differential wheel odometry.
 *
 * This replaces the earlier encoder-only scheme that regulated differential
 * wheel arc in cm. Turn PID gains are in different units as a result; see the
 * conversion note in control_config.h.
 *
 * MULTI-RATE STRUCTURE
 * --------------------
 * The two sensors run at different rates, and the controller respects that:
 *
 *   ~1 kHz  EKF prediction     - poll the gyro, integrate rate into yaw
 *                                (matches the configured gyro ODR)
 *    100 Hz EKF correction     - read encoders, apply the odometry update
 *           + PID + motor out    (CONTROL_SAMPLE_TIME_S)
 *
 * Predicting at the gyro's own rate keeps integration error low; correcting
 * and actuating more slowly keeps the PID sample time constant, which its
 * derivative and integral terms depend on.
 *
 * GRACEFUL DEGRADATION
 * --------------------
 * If the IMU fails to initialise or stops responding, the controller falls
 * back to encoder-only operation automatically: the EKF simply receives no
 * prediction steps and its yaw is driven entirely by odometry updates. Turns
 * still work, just with the pre-IMU accuracy. Check TurnController_IsImuOk().
 * ============================================================================
 */

#ifndef PI
#define PI 3.14159265358979323846f
#endif

typedef enum {

    //Turning modes
    TURN_IDLE = 0,
    TURN_RUNNING,
    TURN_COMPLETED,
    TURN_TIMEOUT

} TurnState_t;

/* Outcome of the stationary gyro bias calibration.
 * Anything other than TURN_BIAS_OK means the EKF is running with an
 * unestimated (zero) gyro bias, which degrades turn accuracy silently. */
typedef enum {

    TURN_BIAS_NOT_RUN   = 0,  /* never attempted (e.g. no IMU detected)     */
    TURN_BIAS_OK        = 1,  /* stationary average accepted and applied    */
    TURN_BIAS_MOVING    = 2,  /* rejected: motion above IMU_GYRO_BIAS_MAX_DPS */
    TURN_BIAS_IMU_ERROR = 3   /* aborted: a gyro read failed mid-sweep      */

} TurnBiasCalStatus_t;

/* Debugging / live-watch. Volatile because their only purpose is to be read
 * from outside the firmware (live watch / raw SWD memory reads). */
extern volatile float turn_target_yaw_deg;
extern volatile float turn_fused_yaw_deg;
extern volatile float turn_encoder_yaw_deg;
extern volatile float turn_yaw_error_deg;   /* control error: target - fused */
extern volatile float turn_fusion_gap_deg;  /* fused - encoder (ObserveYaw)  */
extern volatile float turn_gyro_rate_dps;
extern volatile float turn_gyro_bias_dps;
extern volatile float turn_basespeed;
extern volatile uint32_t turn_predict_count;
extern volatile uint32_t turn_update_count;
extern volatile uint32_t turn_reject_count;

/* Failed gyro reads. Non-zero means fused yaw lost integration intervals and
 * is under-reading rotation -- indistinguishable from encoder over-read due
 * to wheel slip unless you check this counter. */
extern volatile uint32_t turn_imu_fail_count;

extern volatile TurnBiasCalStatus_t turn_bias_cal_status;

//Function prototypes

/* Initialise the controller: encoders, motor driver, IMU and EKF.
 * Also performs the stationary gyro bias calibration, so THE ROBOT MUST BE
 * STILL when this is called. Returns 1 if the IMU came up, 0 if the
 * controller fell back to encoder-only mode. */
uint8_t TurnController_Init(void);

/* Re-run the stationary gyro bias calibration. Robot must be still.
 * Returns 1 on success, 0 if the IMU is absent or the robot was moving. */
uint8_t TurnController_CalibrateGyroBias(void);

/* Was the IMU detected and is fusion active? */
uint8_t TurnController_IsImuOk(void);

/* Did the startup gyro bias calibration actually succeed? This is a SEPARATE
 * question from TurnController_IsImuOk(): the IMU can be up and responding
 * while the bias calibration was rejected, in which case turns still run but
 * with an unestimated bias. See turn_bias_cal_status for the reason. */
uint8_t TurnController_IsBiasCalibrated(void);

/* Blocking turns. Return 1 on success, 0 if the safety timeout fired. */
uint8_t turnLeftAngle(float angle_deg);
uint8_t turnRightAngle(float angle_deg);

/* Current fused yaw estimate in degrees, relative to the last turn's start. */
float TurnController_GetYawDeg(void);

/* Estimated gyro bias in deg/s. Should settle to a small, stable number. */
float TurnController_GetGyroBiasDps(void);

/* Run the fusion loop without commanding the motors, for `duration_ms`.
 * Used by the yaw-estimation test to observe the filter while the robot is
 * rotated by hand. */
void TurnController_ObserveYaw(uint32_t duration_ms);

/* Reset the yaw estimate to zero without touching the learned gyro bias. */
void TurnController_ResetYaw(void);

#endif /* TURN_CONTROLLER_H */
