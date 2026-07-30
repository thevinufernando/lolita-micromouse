#ifndef EKF_H
#define EKF_H

#include <stdint.h>

/*
 * ============================================================================
 *      EXTENDED KALMAN FILTER - YAW ESTIMATION FROM GYRO + ENCODER ODOMETRY
 * ============================================================================
 *
 * PURPOSE
 * -------
 * Fuses the ICM-42688-P Z-axis gyro with differential wheel odometry to
 * produce a yaw estimate that is better than either sensor alone:
 *
 *   - The gyro is accurate over short intervals but its output contains a
 *     slowly drifting bias. Integrating it open-loop makes yaw drift without
 *     bound.
 *   - Encoder odometry has no drift in the absence of slip, but wheels DO slip
 *     during pivot turns, and slip corrupts the yaw estimate permanently.
 *
 * The filter integrates the gyro for short-term accuracy while continuously
 * estimating the gyro bias, and uses the encoders as a slow absolute anchor
 * that keeps the bias observable.
 *
 * There is no magnetometer on this robot, so yaw is a RELATIVE quantity only:
 * it is measured from wherever the filter was last reset, not from magnetic
 * north. Each turn resets the filter to zero, so accumulated drift from
 * previous turns does not carry into the next one.
 *
 * ----------------------------------------------------------------------------
 * STATE VECTOR
 * ----------------------------------------------------------------------------
 *      x = [ yaw        ]   radians, CONTINUOUS (never wrapped, see below)
 *          [ gyro_bias  ]   rad/s, the additive offset on the gyro reading
 *
 * ----------------------------------------------------------------------------
 * PROCESS MODEL (prediction, driven by the gyro as a control input)
 * ----------------------------------------------------------------------------
 *      yaw_k       = yaw_{k-1} + (gyro_z - bias_{k-1}) * dt
 *      bias_k      = bias_{k-1}                      (random walk)
 *
 *      F = d f / d x = [ 1   -dt ]
 *                      [ 0    1  ]
 *
 *      Q = [ Q_yaw*dt      0       ]
 *          [    0      Q_bias*dt   ]
 *
 * ----------------------------------------------------------------------------
 * MEASUREMENT MODEL (correction, from wheel odometry)
 * ----------------------------------------------------------------------------
 *      z = yaw_encoder = (right_distance - left_distance) / wheel_base
 *
 *      h(x) = yaw          =>   H = [ 1   0 ]
 *      R    = encoder yaw variance, rad^2
 *
 * ----------------------------------------------------------------------------
 * A NOTE ON "EXTENDED"
 * ----------------------------------------------------------------------------
 * For this particular state vector the process and measurement models are
 * both linear, so the Jacobians F and H are exact rather than approximations
 * and the filter is mathematically a plain Kalman filter. It is implemented
 * here in EKF form - explicit Jacobian evaluation each step, predict/update
 * separated - so that adding a nonlinear state later (wheel scale factor,
 * slip ratio, or a full [x, y, yaw] pose) requires only changing the model
 * functions, not restructuring the filter.
 *
 * ----------------------------------------------------------------------------
 * WHY YAW IS NOT WRAPPED TO [-pi, pi]
 * ----------------------------------------------------------------------------
 * A micromouse routinely turns 180 degrees and this project's test suite does
 * a full 360. If the state were wrapped, the innovation (z - yaw) would have
 * to be wrapped too, and any inconsistency between the two produces a filter
 * that silently locks up at the wrap boundary. Keeping yaw continuous makes
 * turns of any magnitude unambiguous and removes an entire class of bugs.
 * The cost is that yaw grows without bound over a long run; at float32
 * precision this stays well under 0.001 rad of resolution error even after
 * hundreds of revolutions. Use EKF_NormalizeAngle() at the point of display
 * if a wrapped value is wanted.
 *
 * ----------------------------------------------------------------------------
 * NUMERICAL CHOICES
 * ----------------------------------------------------------------------------
 *  - The covariance update uses the Joseph form, which preserves symmetry and
 *    positive-definiteness far better than the shorter (I-KH)P form when
 *    gains are small and updates are frequent.
 *  - P is explicitly re-symmetrised each update to stop asymmetry accumulating
 *    from float rounding.
 *  - Encoder updates are gated on normalised innovation. A wheel that slips
 *    during a turn produces a large, wrong measurement; rejecting it lets the
 *    gyro carry the estimate through the slip instead of being dragged off.
 * ============================================================================
 */

/* Tuning and initial-uncertainty parameters. */
typedef struct {

    float Q_yaw;        /* yaw process noise density, rad^2/s               */
    float Q_bias;       /* gyro bias random-walk density, (rad/s)^2/s       */
    float R_encoder;    /* encoder yaw variance at rest, rad^2              */

    /* Rate-dependent inflation of the encoder measurement noise:
     *
     *      R_effective = R_encoder + R_slip_coeff * yaw_rate^2
     *
     * Wheel slip is not random noise - it is a systematic error that grows
     * with how hard the robot is spinning. A fixed R therefore mis-models it:
     * innovation gating only catches sudden outliers, so a steady slip ramp
     * looks like a legitimate signal and drags the estimate off.
     *
     * Inflating R with rate^2 encodes the physics directly: at rest the
     * encoders are the trustworthy sensor and pin down the gyro bias, while
     * mid-turn the gyro is trusted and the wheels are all but ignored.
     * Set to 0 for a classical fixed-R filter. */
    float R_slip_coeff; /* rad^2 per (rad/s)^2                              */

    float P0_yaw;       /* initial yaw variance, rad^2                      */
    float P0_bias;      /* initial bias variance, (rad/s)^2                 */

    /* Reject an encoder update whose innovation exceeds this many standard
     * deviations. Set to 0 to disable gating entirely. */
    float innovation_gate;

} EKF_Config_t;

typedef struct {

    /* --- state --- */
    float yaw;              /* rad, continuous                             */
    float gyro_bias;        /* rad/s                                       */

    /* --- covariance, row major --- */
    float P[2][2];

    EKF_Config_t cfg;

    /* --- diagnostics, useful in the debugger live-watch --- */
    float last_innovation;      /* z - h(x), rad                           */
    float last_innovation_var;  /* S, rad^2                                */
    float last_R;               /* effective R actually used, rad^2        */
    float last_rate;            /* bias-corrected rate integrated, rad/s   */
    float last_dt;              /* seconds                                 */

    uint32_t predict_count;
    uint32_t update_count;
    uint32_t reject_count;      /* gated-out encoder updates               */

    uint8_t initialized;

} EKF_t;

/* Fill `cfg` with the defaults from control_config.h. */
void  EKF_GetDefaultConfig(EKF_Config_t *cfg);

/* Initialise the filter. Zeroes the state and seeds P from cfg. */
void  EKF_Init(EKF_t *ekf, const EKF_Config_t *cfg);

/* Restart estimation at a known yaw. The bias estimate and its variance are
 * PRESERVED, because bias is a property of the sensor and stays valid across
 * turns; only yaw and its uncertainty are reset. */
void  EKF_Reset(EKF_t *ekf, float yaw0_rad);

/* Overwrite the bias estimate, e.g. from a stationary calibration sweep. */
void  EKF_SetGyroBias(EKF_t *ekf, float bias_rads, float bias_variance);

/* Prediction step. `gyro_z_rads` is the RAW gyro reading in rad/s, already
 * sign-corrected for mounting orientation. `dt` is in seconds. */
void  EKF_Predict(EKF_t *ekf, float gyro_z_rads, float dt);

/* Correction step from odometry-derived yaw, in radians (continuous, same
 * convention as the state). Returns 1 if applied, 0 if gated out. */
uint8_t EKF_UpdateEncoderYaw(EKF_t *ekf, float yaw_meas_rad);

/* Adjust trust in the encoders at runtime. Larger R = trust them less. */
void  EKF_SetEncoderNoise(EKF_t *ekf, float R);

/* --- accessors --- */
float EKF_GetYaw(const EKF_t *ekf);         /* rad, continuous             */
float EKF_GetYawDeg(const EKF_t *ekf);      /* deg, continuous             */
float EKF_GetGyroBias(const EKF_t *ekf);    /* rad/s                       */
float EKF_GetGyroBiasDps(const EKF_t *ekf); /* deg/s                       */
float EKF_GetYawVariance(const EKF_t *ekf); /* rad^2                       */
float EKF_GetYawStdDevDeg(const EKF_t *ekf);/* deg, 1 sigma                */

/* Wrap an angle into [-pi, pi]. Only for display; the state stays continuous. */
float EKF_NormalizeAngle(float angle_rad);

#endif /* EKF_H */
