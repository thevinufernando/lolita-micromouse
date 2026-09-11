#include "yaw_estimator.h"

#include <math.h>

#ifndef PI
#define PI 3.14159265358979323846f
#endif

#define DEG_TO_RAD_F (PI / 180.0f)
#define RAD_TO_DEG_F (180.0f / PI)

/* The filter itself. Static: every consumer goes through this module's API so
 * there is exactly one yaw estimate on the robot, and controllers cannot get
 * out of step by each keeping their own. */
static EKF_t yaw_ekf;

/* DWT cycle stamp of the last prediction. */
static uint32_t last_predict_cycles;

/* Set once at init; drives the encoder-only fallback. */
static uint8_t imu_ok = 0U;

/* Yaw at the moment the encoders were last zeroed.
 *
 * The odometry measurement is absolute travel since Encoders_Reset(), so on
 * its own it can only ever express "how far have I turned since the reset".
 * Continuous heading needs the filter's yaw to survive a move boundary while
 * the encoders do not, and this offset is what bridges the two: the encoders
 * restart from zero, and their contribution is re-based onto wherever yaw had
 * actually reached. Without it, resetting the encoders alone would make the
 * measurement disagree with the state by the whole accumulated heading. */
static float encoder_origin_rad = 0.0f;

volatile float yaw_fused_deg;
volatile float yaw_encoder_deg;
volatile float yaw_fusion_gap_deg;
volatile float yaw_gyro_rate_dps;
volatile float yaw_gyro_bias_dps;
volatile uint32_t yaw_predict_count;
volatile uint32_t yaw_update_count;
volatile uint32_t yaw_reject_count;
volatile uint32_t yaw_imu_fail_count;

volatile YawBiasCalStatus_t yaw_bias_cal_status = YAW_BIAS_NOT_RUN;
volatile uint32_t yaw_bias_cal_attempts;
volatile float    yaw_bias_cal_peak_dps;


/* ------------------------------------------------------------------------
 * Yaw from wheel odometry.
 *
 * The wheels' differential travel is (right - left), and the rotation is that
 * divided by the wheel base. Units cancel, giving radians. Positive =
 * anticlockwise = left turn, matching the gyro sign convention.
 *
 * This holds for ANY motion, not just a pivot: forward travel is common-mode
 * and cancels in the difference, so the same expression is valid during a
 * straight run or an arc.
 * ---------------------------------------------------------------------- */
static float encoderYawRad(void)
{
    float since_reset = (Encoder_getRightDistance() - Encoder_getLeftDistance())
                        / ROBOT_WHEEL_BASE_CM;

    return encoder_origin_rad + since_reset;
}


void YawEstimator_Predict(void)
{
    if (!imu_ok) return;

    /* Only consume a new sample once per gyro output period. Polling faster
     * would integrate the same reading twice and inflate the rotation. */
    if (DWT_ElapsedUs(last_predict_cycles) < IMU_PREDICT_PERIOD_US) {
        return;
    }

    float dt = (float)DWT_ElapsedUs(last_predict_cycles) * 1.0e-6f;
    last_predict_cycles = DWT_GetCycles();

    float gz_dps = 0.0f;

    if (ICM42688_ReadGyroZ(&gz_dps) != IMU_OK) {
        /* NOTE: last_predict_cycles was already advanced above, so this
         * interval's rotation is dropped rather than carried into the next
         * prediction. Counted here so the loss is at least visible; if this
         * ever reads non-zero, fix the ordering (read the gyro before closing
         * the interval) rather than just watching the counter grow. */
        yaw_imu_fail_count++;
        return;
    }

    /* Apply the mounting sign, then convert to rad/s for the filter. */
    float gz_rads = gz_dps * IMU_GYRO_Z_SIGN * DEG_TO_RAD_F;

    EKF_Predict(&yaw_ekf, gz_rads, dt);

    yaw_gyro_rate_dps = yaw_ekf.last_rate * RAD_TO_DEG_F;
}


void YawEstimator_Correct(void)
{
    Encoders_Update();

    float yaw_enc = encoderYawRad();

    yaw_encoder_deg = yaw_enc * RAD_TO_DEG_F;

    EKF_UpdateEncoderYaw(&yaw_ekf, yaw_enc);
}


void YawEstimator_PublishTelemetry(void)
{
    yaw_fused_deg      = EKF_GetYawDeg(&yaw_ekf);
    yaw_gyro_bias_dps  = EKF_GetGyroBiasDps(&yaw_ekf);
    yaw_predict_count  = yaw_ekf.predict_count;
    yaw_update_count   = yaw_ekf.update_count;
    yaw_reject_count   = yaw_ekf.reject_count;

    /* How far the filter has walked away from raw odometry. This is the
     * wheel-slip measurement: the gyro is carrying the estimate and the
     * encoders are being discounted, so the gap IS the slip. */
    yaw_fusion_gap_deg = yaw_fused_deg - yaw_encoder_deg;
}


uint8_t YawEstimator_CalibrateGyroBias(void)
{
    if (!imu_ok) {
        yaw_bias_cal_status = YAW_BIAS_IMU_ERROR;
        return 0U;
    }

    /* Retry a motion-rejected sweep rather than giving up on the first one.
     * A single leftover wobble (from setting the robot down, or from the
     * reset button being pressed on the chassis itself) is enough to trip the
     * worst-sample test, and silently falling back to an unestimated bias is
     * worse than spending a few more seconds at boot. */
    for (uint32_t attempt = 0U; attempt < IMU_GYRO_BIAS_MAX_ATTEMPTS; attempt++)
    {
        yaw_bias_cal_attempts = attempt + 1U;

        /* Let the chassis stop vibrating before sampling. */
        HAL_Delay(IMU_GYRO_BIAS_SETTLE_MS);

        float sum = 0.0f;
        float max_abs = 0.0f;

        for (uint32_t i = 0U; i < IMU_GYRO_BIAS_SAMPLES; i++)
        {
            float gz_dps = 0.0f;

            if (ICM42688_ReadGyroZ(&gz_dps) != IMU_OK) {
                /* A dead bus will not fix itself, so do not burn the
                 * remaining attempts on it. */
                yaw_imu_fail_count++;
                yaw_bias_cal_status = YAW_BIAS_IMU_ERROR;
                return 0U;
            }

            sum += gz_dps;

            float abs_dps = fabsf(gz_dps);
            if (abs_dps > max_abs) max_abs = abs_dps;

            /* Pace the sampling to the gyro output rate so we average distinct
             * samples rather than re-reading one value many times. */
            DWT_DelayUs(IMU_PREDICT_PERIOD_US);
        }

        /* If anything moved during calibration the average is meaningless. */
        if (max_abs > IMU_GYRO_BIAS_MAX_DPS) {
            yaw_bias_cal_peak_dps = max_abs;    /* what tripped it, for tuning */
            continue;                           /* settle longer and try again */
        }

        float mean_dps = sum / (float)IMU_GYRO_BIAS_SAMPLES;

        /* Store in the same sign convention the prediction step uses. */
        float bias_rads = mean_dps * IMU_GYRO_Z_SIGN * DEG_TO_RAD_F;

        /* A 1000-sample average is a confident estimate, so seed a small
         * variance and let the filter refine it from there. */
        EKF_SetGyroBias(&yaw_ekf, bias_rads, 1.0e-6f);

        yaw_gyro_bias_dps     = EKF_GetGyroBiasDps(&yaw_ekf);
        yaw_bias_cal_peak_dps = max_abs;
        yaw_bias_cal_status   = YAW_BIAS_OK;

        return 1U;
    }

    /* Every attempt saw motion. */
    yaw_bias_cal_status = YAW_BIAS_MOVING;

    return 0U;
}


uint8_t YawEstimator_Init(void)
{
    DWT_Timer_Init();

    EKF_Config_t ekf_cfg;
    EKF_GetDefaultConfig(&ekf_cfg);
    EKF_Init(&yaw_ekf, &ekf_cfg);

    /* Bring up the IMU. Failure is not fatal: fall back to encoder-only. */
    imu_ok = (ICM42688_Init() == IMU_OK) ? 1U : 0U;

    if (imu_ok) {
        /* A rejected calibration is NOT fatal but is also not harmless: the
         * EKF keeps running with gyro_bias = 0, which quietly degrades
         * everything downstream. The outcome lands in yaw_bias_cal_status so
         * the caller (and live-watch) can tell the difference between "IMU up
         * and calibrated" and "IMU up but flying blind on bias". */
        (void)YawEstimator_CalibrateGyroBias();
    }

    last_predict_cycles = DWT_GetCycles();

    YawEstimator_PublishTelemetry();

    return imu_ok;
}


uint8_t YawEstimator_IsImuOk(void)
{
    return imu_ok;
}


uint8_t YawEstimator_IsBiasCalibrated(void)
{
    return (yaw_bias_cal_status == YAW_BIAS_OK) ? 1U : 0U;
}


void YawEstimator_Reset(void)
{
    /* Deliberately does NOT reset the encoders -- see the pairing rule in
     * yaw_estimator.h. The caller resets them, first. */
    EKF_Reset(&yaw_ekf, 0.0f);

    /* Yaw is now zero, so the encoder measurement must start from zero too. */
    encoder_origin_rad = 0.0f;

    last_predict_cycles = DWT_GetCycles();

    YawEstimator_PublishTelemetry();
}


void YawEstimator_RebaseEncoders(void)
{
    /* Call IMMEDIATELY after Encoders_Reset() when yaw must stay continuous.
     * Pins the encoder measurement to wherever the estimate currently is, so
     * zeroing the wheels does not move the measurement out from under it. */
    encoder_origin_rad = EKF_GetYaw(&yaw_ekf);
}


float YawEstimator_GetYawDeg(void)
{
    return EKF_GetYawDeg(&yaw_ekf);
}


float YawEstimator_GetGyroBiasDps(void)
{
    return EKF_GetGyroBiasDps(&yaw_ekf);
}
