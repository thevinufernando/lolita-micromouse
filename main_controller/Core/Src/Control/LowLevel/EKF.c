#include "EKF.h"
#include "control_config.h"
#include <math.h>

#ifndef M_PI_F
#define M_PI_F 3.14159265358979323846f
#endif

#define TWO_PI_F (2.0f * M_PI_F)
#define RAD_TO_DEG_F (180.0f / M_PI_F)

/* Largest dt we will accept in one prediction step. A longer gap means the
 * loop stalled (debugger halt, blocking SPI retry); integrating across it
 * would inject a large bogus rotation, so we clamp instead. */
#define EKF_MAX_DT_S 0.100f


void EKF_GetDefaultConfig(EKF_Config_t *cfg)
{
    if (cfg == 0) return;

    cfg->Q_yaw           = EKF_Q_YAW;
    cfg->Q_bias          = EKF_Q_BIAS;
    cfg->R_encoder       = EKF_R_ENCODER_YAW;
    cfg->R_slip_coeff    = EKF_R_SLIP_COEFF;
    cfg->P0_yaw          = EKF_P0_YAW;
    cfg->P0_bias         = EKF_P0_BIAS;
    cfg->innovation_gate = EKF_INNOVATION_GATE;
}


void EKF_Init(EKF_t *ekf, const EKF_Config_t *cfg)
{
    if (ekf == 0) return;

    if (cfg != 0) {
        ekf->cfg = *cfg;
    }
    else {
        EKF_GetDefaultConfig(&ekf->cfg);
    }

    ekf->yaw       = 0.0f;
    ekf->gyro_bias = 0.0f;

    /* Diagonal initial covariance: yaw and bias start uncorrelated. */
    ekf->P[0][0] = ekf->cfg.P0_yaw;
    ekf->P[0][1] = 0.0f;
    ekf->P[1][0] = 0.0f;
    ekf->P[1][1] = ekf->cfg.P0_bias;

    ekf->last_innovation     = 0.0f;
    ekf->last_innovation_var = 0.0f;
    ekf->last_R              = ekf->cfg.R_encoder;
    ekf->last_rate           = 0.0f;
    ekf->last_dt             = 0.0f;

    ekf->predict_count = 0U;
    ekf->update_count  = 0U;
    ekf->reject_count  = 0U;

    ekf->initialized = 1U;
}


void EKF_Reset(EKF_t *ekf, float yaw0_rad)
{
    if (ekf == 0) return;

    ekf->yaw = yaw0_rad;

    /* Reset yaw uncertainty and the yaw/bias cross-correlation, but KEEP the
     * bias estimate and its variance: the bias belongs to the sensor and is
     * still valid, and rediscovering it from scratch every turn would waste
     * the observability we paid for during the previous move. */
    ekf->P[0][0] = ekf->cfg.P0_yaw;
    ekf->P[0][1] = 0.0f;
    ekf->P[1][0] = 0.0f;
    /* P[1][1] deliberately untouched */

    ekf->last_innovation     = 0.0f;
    ekf->last_innovation_var = 0.0f;
}


void EKF_SetGyroBias(EKF_t *ekf, float bias_rads, float bias_variance)
{
    if (ekf == 0) return;

    ekf->gyro_bias = bias_rads;

    if (bias_variance > 0.0f) {
        ekf->P[1][1] = bias_variance;
    }

    /* A directly measured bias is uncorrelated with the current yaw. */
    ekf->P[0][1] = 0.0f;
    ekf->P[1][0] = 0.0f;
}


void EKF_Predict(EKF_t *ekf, float gyro_z_rads, float dt)
{
    if (ekf == 0 || !ekf->initialized) return;

    /* Reject non-finite or non-positive intervals outright. */
    if (!(dt > 0.0f)) return;

    if (dt > EKF_MAX_DT_S) {
        dt = EKF_MAX_DT_S;
    }

    /* ---------------- state propagation ---------------- */

    float rate = gyro_z_rads - ekf->gyro_bias;

    ekf->yaw += rate * dt;
    /* bias is modelled as constant over one step */

    ekf->last_rate = rate;
    ekf->last_dt   = dt;

    /* ---------------- covariance propagation ----------------
     *
     * P = F P F^T + Q,   with F = [ 1  -dt ]
     *                             [ 0   1  ]
     *
     * Expanded for the 2x2 case so no general matrix code is needed:
     *
     *   P00' = P00 - dt*(P01 + P10) + dt^2 * P11
     *   P01' = P01 - dt*P11
     *   P10' = P10 - dt*P11
     *   P11' = P11
     */

    float P00 = ekf->P[0][0];
    float P01 = ekf->P[0][1];
    float P10 = ekf->P[1][0];
    float P11 = ekf->P[1][1];

    float new_P00 = P00 - dt * (P01 + P10) + dt * dt * P11;
    float new_P01 = P01 - dt * P11;
    float new_P10 = P10 - dt * P11;
    float new_P11 = P11;

    /* Add process noise, scaled by the interval. */
    new_P00 += ekf->cfg.Q_yaw  * dt;
    new_P11 += ekf->cfg.Q_bias * dt;

    ekf->P[0][0] = new_P00;
    ekf->P[0][1] = new_P01;
    ekf->P[1][0] = new_P10;
    ekf->P[1][1] = new_P11;

    ekf->predict_count++;
}


uint8_t EKF_UpdateEncoderYaw(EKF_t *ekf, float yaw_meas_rad)
{
    if (ekf == 0 || !ekf->initialized) return 0U;

    /* ---------------- innovation ----------------
     * H = [1 0], so h(x) is simply the yaw state. Both the state and the
     * measurement are continuous, so no angle wrapping is needed here. */

    float innovation = yaw_meas_rad - ekf->yaw;

    /* ---------------- rate-adaptive measurement noise ----------------
     * Slip grows with how hard the robot is spinning, so inflate R with the
     * square of the current rotation rate. At rest this reduces to the plain
     * R_encoder and the wheels anchor the gyro bias; mid-turn R becomes large
     * and the gyro is left to carry the estimate through the slip. */
    float rate = ekf->last_rate;
    float R = ekf->cfg.R_encoder + ekf->cfg.R_slip_coeff * rate * rate;

    ekf->last_R = R;

    /* S = H P H^T + R */
    float S = ekf->P[0][0] + R;

    ekf->last_innovation     = innovation;
    ekf->last_innovation_var = S;

    if (!(S > 0.0f)) {
        /* Degenerate covariance; skip rather than divide by zero. */
        return 0U;
    }

    /* ---------------- innovation gating ----------------
     * Wheel slip during a pivot turn shows up as a large innovation. Applying
     * it would drag the estimate toward the slipped odometry, which is exactly
     * the error the IMU is here to reject. */
    if (ekf->cfg.innovation_gate > 0.0f) {

        float gate = ekf->cfg.innovation_gate * sqrtf(S);

        if (fabsf(innovation) > gate) {
            ekf->reject_count++;
            return 0U;
        }
    }

    /* ---------------- Kalman gain ----------------
     * K = P H^T S^-1, and H^T selects the first column of P. */

    float K0 = ekf->P[0][0] / S;
    float K1 = ekf->P[1][0] / S;

    /* ---------------- state correction ---------------- */

    ekf->yaw       += K0 * innovation;
    ekf->gyro_bias += K1 * innovation;

    /* ---------------- covariance correction, Joseph form ----------------
     *
     * P = (I - K H) P (I - K H)^T + K R K^T
     *
     * With H = [1 0]:  (I - K H) = [ 1-K0   0 ]
     *                              [  -K1   1 ]
     *
     * The Joseph form costs a few more multiplies than (I-K H)P but stays
     * symmetric and positive-definite under float rounding, which matters
     * here because the filter runs thousands of updates per turn.
     */

    float a = 1.0f - K0;
    float b = -K1;

    float P00 = ekf->P[0][0];
    float P01 = ekf->P[0][1];
    float P10 = ekf->P[1][0];
    float P11 = ekf->P[1][1];

    /* Same R that produced K, or the Joseph form is inconsistent. */

    float new_P00 = a * a * P00
                  + R * K0 * K0;

    float new_P01 = a * b * P00 + a * P01
                  + R * K0 * K1;

    float new_P10 = a * b * P00 + a * P10
                  + R * K1 * K0;

    float new_P11 = b * b * P00 + b * P10 + b * P01 + P11
                  + R * K1 * K1;

    /* Re-symmetrise. In exact arithmetic these are already equal; forcing it
     * stops rounding asymmetry from compounding over long runs. */
    float off_diagonal = 0.5f * (new_P01 + new_P10);

    ekf->P[0][0] = new_P00;
    ekf->P[0][1] = off_diagonal;
    ekf->P[1][0] = off_diagonal;
    ekf->P[1][1] = new_P11;

    ekf->update_count++;

    return 1U;
}


void EKF_SetEncoderNoise(EKF_t *ekf, float R)
{
    if (ekf == 0) return;

    if (R > 0.0f) {
        ekf->cfg.R_encoder = R;
    }
}


float EKF_GetYaw(const EKF_t *ekf)
{
    return (ekf != 0) ? ekf->yaw : 0.0f;
}

float EKF_GetYawDeg(const EKF_t *ekf)
{
    return (ekf != 0) ? (ekf->yaw * RAD_TO_DEG_F) : 0.0f;
}

float EKF_GetGyroBias(const EKF_t *ekf)
{
    return (ekf != 0) ? ekf->gyro_bias : 0.0f;
}

float EKF_GetGyroBiasDps(const EKF_t *ekf)
{
    return (ekf != 0) ? (ekf->gyro_bias * RAD_TO_DEG_F) : 0.0f;
}

float EKF_GetYawVariance(const EKF_t *ekf)
{
    return (ekf != 0) ? ekf->P[0][0] : 0.0f;
}

float EKF_GetYawStdDevDeg(const EKF_t *ekf)
{
    if (ekf == 0) return 0.0f;

    float variance = ekf->P[0][0];

    if (variance < 0.0f) variance = 0.0f;

    return sqrtf(variance) * RAD_TO_DEG_F;
}


float EKF_NormalizeAngle(float angle_rad)
{
    /* fmodf keeps this O(1) regardless of how far the input has wound up,
     * unlike a while-loop which would stall on a large continuous yaw. */
    angle_rad = fmodf(angle_rad + M_PI_F, TWO_PI_F);

    if (angle_rad < 0.0f) {
        angle_rad += TWO_PI_F;
    }

    return angle_rad - M_PI_F;
}
