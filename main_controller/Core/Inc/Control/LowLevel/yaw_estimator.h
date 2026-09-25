#ifndef YAW_ESTIMATOR_H
#define YAW_ESTIMATOR_H

#include "EKF.h"
#include "encoders.h"
#include "ICM42688.h"
#include "dwt_timer.h"
#include "control_config.h"
#include "main.h"
#include <stdint.h>

/*
 * ============================================================================
 *                 YAW ESTIMATOR - GYRO + ENCODER SENSOR FUSION
 * ============================================================================
 *
 * Owns the 2-state EKF ([yaw, gyro_bias]) that fuses the ICM-42688-P Z gyro
 * with differential wheel odometry, plus the stationary gyro bias calibration.
 *
 * This used to live inside turn_controller.c as file-static state. It was
 * pulled out because turns are no longer the only consumer: arc turns and the
 * straight-line controller both need fused yaw, and neither should have to go
 * through a turn controller to get it.
 *
 * WHAT THIS MODULE IS NOT
 * -----------------------
 * It does not drive motors, does not own a PID and does not know what a move
 * is. It answers one question: which way is the robot pointing, relative to
 * the last reset. Everything about how to act on that belongs to a controller.
 *
 * ---------------------------------------------------------------------------
 * THERE IS NO ABSOLUTE HEADING
 * ---------------------------------------------------------------------------
 * The IMU has NO MAGNETOMETER, so nothing here is referenced to the world.
 * Yaw is always relative to the last YawEstimator_Reset(), and it drifts with
 * nothing to bound it. The encoder update anchors the gyro BIAS, not the
 * heading itself.
 *
 * The practical consequence: a long sequence of moves accumulates heading
 * error that this module cannot see or correct. Bounding it needs an outside
 * reference, which on this robot means the ToF sensors reading the maze walls.
 *
 * ---------------------------------------------------------------------------
 * !! RESET PAIRING - THE ONE RULE THAT MATTERS !!
 * ---------------------------------------------------------------------------
 * The measurement is ABSOLUTE differential wheel travel accumulated since the
 * last Encoders_Reset(), not an increment. YawEstimator_Reset() zeroes the
 * filter's yaw but deliberately DOES NOT touch the encoders, because encoder
 * distance belongs to whichever controller is running the move.
 *
 * So the caller MUST reset both together, encoders first:
 *
 *     Encoders_Reset();
 *     YawEstimator_Reset();
 *
 * Reset one without the other and the measurement is offset from the state by
 * however much the wheels had already turned. The filter will quietly drag
 * yaw toward that offset instead of failing, which is the worst kind of bug.
 *
 * ---------------------------------------------------------------------------
 * MULTI-RATE USE
 * ---------------------------------------------------------------------------
 *   ~1 kHz  YawEstimator_Predict()  - polls the gyro, integrates rate
 *    100 Hz YawEstimator_Correct()  - reads encoders, applies the update
 *
 * Predict() is self-rate-limiting to IMU_PREDICT_PERIOD_US, so calling it
 * faster than the gyro's output rate is harmless -- it simply returns. Call it
 * as often as the control loop allows.
 *
 * GRACEFUL DEGRADATION
 * --------------------
 * If the IMU fails to initialise, Predict() becomes a no-op and yaw is driven
 * entirely by odometry. Everything still works, just with pre-IMU accuracy.
 * Check YawEstimator_IsImuOk().
 * ============================================================================
 */

/* Outcome of the stationary gyro bias calibration. Anything other than
 * YAW_BIAS_OK means the EKF is running with an unestimated (zero) gyro bias,
 * which degrades accuracy silently. */
typedef enum {

    YAW_BIAS_NOT_RUN   = 0,  /* never attempted (e.g. no IMU detected)       */
    YAW_BIAS_OK        = 1,  /* stationary average accepted and applied      */
    YAW_BIAS_MOVING    = 2,  /* rejected: motion above IMU_GYRO_BIAS_MAX_DPS */
    YAW_BIAS_IMU_ERROR = 3   /* aborted: a gyro read failed mid-sweep        */

} YawBiasCalStatus_t;

/* Debugging / live-watch. Volatile because their only purpose is to be read
 * from outside the firmware (live watch / raw SWD memory reads). */
extern volatile float yaw_fused_deg;      /* fused estimate, deg           */
extern volatile float yaw_encoder_deg;    /* odometry-only yaw, deg        */
extern volatile float yaw_fusion_gap_deg; /* fused - encoder, deg          */
extern volatile float yaw_gyro_rate_dps;  /* bias-corrected rate, deg/s    */
extern volatile float yaw_gyro_bias_dps;  /* EKF bias estimate, deg/s      */
extern volatile uint32_t yaw_predict_count;
extern volatile uint32_t yaw_update_count;
extern volatile uint32_t yaw_reject_count; /* gated-out encoder updates    */

/* Failed gyro reads. Non-zero means fused yaw lost integration intervals and
 * is under-reading rotation -- indistinguishable from encoder over-read due
 * to wheel slip unless you check this counter. */
extern volatile uint32_t yaw_imu_fail_count;

extern volatile YawBiasCalStatus_t yaw_bias_cal_status;

/* Sweeps the calibration needed (1 = clean first try), and the worst single
 * gyro sample on the last sweep -- i.e. what the IMU_GYRO_BIAS_MAX_DPS test
 * was actually judging. */
extern volatile uint32_t yaw_bias_cal_attempts;
extern volatile float    yaw_bias_cal_peak_dps;

/* Bring up the IMU and the filter, then run the stationary bias calibration.
 * THE ROBOT MUST BE STILL when this is called.
 * Returns 1 if the IMU came up, 0 if it fell back to encoder-only. */
uint8_t YawEstimator_Init(void);

/* Re-run the stationary gyro bias calibration. Robot must be still.
 * Returns 1 on success, 0 if the IMU is absent or the robot was moving. */
uint8_t YawEstimator_CalibrateGyroBias(void);

/* Was the IMU detected and is fusion active? */
uint8_t YawEstimator_IsImuOk(void);

/* Did the startup bias calibration actually succeed? A SEPARATE question from
 * IsImuOk(): the IMU can be up and responding while calibration was rejected,
 * in which case everything runs but with an unestimated bias. */
uint8_t YawEstimator_IsBiasCalibrated(void);

/* Fast path, ~1 kHz. Self-rate-limiting; safe to call more often. */
void YawEstimator_Predict(void);

/* Slow path, 100 Hz. Reads the encoders and applies the odometry update. */
void YawEstimator_Correct(void);

/* Copy filter diagnostics into the live-watch globals above. */
void YawEstimator_PublishTelemetry(void);

/* Re-base the odometry measurement onto the CURRENT yaw estimate, without
 * disturbing that estimate. Call it immediately after Encoders_Reset() any
 * time yaw must stay continuous across a move boundary -- which is every move
 * once heading is tracked across a whole run rather than per-move.
 *
 * Use this INSTEAD of YawEstimator_Reset() when continuity matters. Reset
 * throws the accumulated heading away; rebase keeps it. */
void YawEstimator_RebaseEncoders(void);

/* Zero the yaw estimate, keeping the learned gyro bias -- the bias belongs to
 * the sensor and stays valid across moves.
 * DOES NOT reset the encoders; see the reset pairing rule above. */
void YawEstimator_Reset(void);

/* Current fused yaw in degrees, relative to the last reset. */
float YawEstimator_GetYawDeg(void);

/* Estimated gyro bias in deg/s. Should settle to a small, stable number. */
float YawEstimator_GetGyroBiasDps(void);

#endif /* YAW_ESTIMATOR_H */
