/* Host-side verification of the yaw EKF.
 * Simulates a pivot turn with a biased, noisy gyro and slipping encoders,
 * then checks that the filter recovers true yaw and true bias. */
#include "EKF.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#define PI_F 3.14159265358979323846f
#define D2R (PI_F/180.0f)
#define R2D (180.0f/PI_F)

static unsigned long rng = 12345;
static float urand(void){ rng = rng*1103515245UL + 12345UL; return (float)((rng>>16)&0x7FFF)/32767.0f; }
static float nrand(void){ /* crude gaussian via sum of uniforms */
    float s=0; for(int i=0;i<12;i++) s+=urand(); return s-6.0f; }

static int failures = 0;
static void check(const char *name, float got, float expect, float tol){
    float err = fabsf(got-expect);
    int ok = (err <= tol);
    if(!ok) failures++;
    printf("  [%s] %-34s got=%9.4f expect=%9.4f tol=%.4f\n",
           ok?"PASS":"FAIL", name, got, expect, tol);
}

/* Verify P stays symmetric and positive definite (2x2: P00>0, det>0). */
static void check_covariance(const EKF_t *e, const char *stage){
    float P00=e->P[0][0], P01=e->P[0][1], P10=e->P[1][0], P11=e->P[1][1];
    float det = P00*P11 - P01*P10;
    int sym = fabsf(P01-P10) < 1e-9f;
    int pd  = (P00 > 0.0f) && (P11 > 0.0f) && (det > 0.0f);
    if(!sym || !pd) failures++;
    printf("  [%s] covariance %-24s P00=%.3e P11=%.3e det=%.3e sym=%d\n",
           (sym&&pd)?"PASS":"FAIL", stage, P00, P11, det, sym);
}

int main(void)
{
    EKF_Config_t cfg; EKF_GetDefaultConfig(&cfg);

    /* ---------------- TEST 1: static bias estimation ---------------- */
    printf("\nTEST 1: stationary, gyro has a constant bias\n");
    {
        EKF_t e; EKF_Init(&e,&cfg);
        const float true_bias_dps = 1.5f;           /* deg/s */
        const float true_bias = true_bias_dps*D2R;
        float dt=0.001f;
        /* encoders say "not rotating"; gyro says it is. Filter must blame bias. */
        for(int k=0;k<30000;k++){                    /* 30 s */
            EKF_Predict(&e, true_bias + 0.0005f*nrand(), dt);
            if(k%10==0) EKF_UpdateEncoderYaw(&e, 0.0f);
        }
        check("bias estimate (deg/s)", EKF_GetGyroBiasDps(&e), true_bias_dps, 0.15f);
        check("yaw stays near zero (deg)", EKF_GetYawDeg(&e), 0.0f, 2.0f);
        check_covariance(&e,"after static run");
    }

    /* ---------------- TEST 2: 90 deg turn, clean sensors ---------------- */
    printf("\nTEST 2: 90 deg anticlockwise turn, consistent sensors\n");
    {
        EKF_t e; EKF_Init(&e,&cfg);
        float dt=0.001f, true_yaw=0.0f;
        const float rate = 90.0f*D2R;               /* 90 deg/s -> 1 s turn */
        for(int k=0;k<1000;k++){
            true_yaw += rate*dt;
            EKF_Predict(&e, rate + 0.001f*nrand(), dt);
            if(k%10==0) EKF_UpdateEncoderYaw(&e, true_yaw + 0.002f*nrand());
        }
        check("fused yaw (deg)", EKF_GetYawDeg(&e), 90.0f, 1.0f);
        check("true yaw (deg)", true_yaw*R2D, 90.0f, 0.01f);
        check_covariance(&e,"after clean turn");
    }

    /* ---------------- TEST 3: wheel slip rejection ---------------- */
    printf("\nTEST 3: 90 deg turn, encoders over-report by 30%% (slip)\n");
    {
        EKF_t e; EKF_Init(&e,&cfg);
        float dt=0.001f, true_yaw=0.0f;
        const float rate = 90.0f*D2R;
        for(int k=0;k<1000;k++){
            true_yaw += rate*dt;
            EKF_Predict(&e, rate, dt);
            /* wheels spin 30% further than the robot actually turned */
            if(k%10==0) EKF_UpdateEncoderYaw(&e, true_yaw*1.30f);
        }
        printf("    encoder-only would report %.2f deg\n", true_yaw*1.30f*R2D);
        check("fused yaw resists slip (deg)", EKF_GetYawDeg(&e), 90.0f, 5.0f);
        printf("    gated-out updates: %u of %u\n", e.reject_count, e.update_count+e.reject_count);
        check_covariance(&e,"after slip turn");
    }

    /* ---------------- TEST 4: continuous yaw past 360 ---------------- */
    printf("\nTEST 4: 720 deg rotation stays continuous (no wrap)\n");
    {
        EKF_t e; EKF_Init(&e,&cfg);
        float dt=0.001f, true_yaw=0.0f;
        const float rate = 360.0f*D2R;
        for(int k=0;k<2000;k++){
            true_yaw += rate*dt;
            EKF_Predict(&e, rate, dt);
            if(k%10==0) EKF_UpdateEncoderYaw(&e, true_yaw);
        }
        check("fused yaw (deg)", EKF_GetYawDeg(&e), 720.0f, 2.0f);
        check("normalized to [-180,180]", EKF_NormalizeAngle(EKF_GetYaw(&e))*R2D, 0.0f, 2.0f);
    }

    /* ---------------- TEST 5: reset preserves bias ---------------- */
    printf("\nTEST 5: EKF_Reset zeroes yaw but keeps the learned bias\n");
    {
        EKF_t e; EKF_Init(&e,&cfg);
        EKF_SetGyroBias(&e, 2.0f*D2R, 1e-6f);
        float before = EKF_GetGyroBiasDps(&e);
        EKF_Predict(&e, 5.0f*D2R, 0.5f);
        EKF_Reset(&e, 0.0f);
        check("yaw reset to zero", EKF_GetYawDeg(&e), 0.0f, 1e-4f);
        check("bias preserved (deg/s)", EKF_GetGyroBiasDps(&e), before, 1e-4f);
    }

    /* ---------------- TEST 6: degenerate inputs ---------------- */
    printf("\nTEST 6: hostile inputs do not corrupt the filter\n");
    {
        EKF_t e; EKF_Init(&e,&cfg);
        EKF_Predict(&e, 1.0f, 0.0f);      /* zero dt   */
        EKF_Predict(&e, 1.0f, -1.0f);     /* negative  */
        EKF_Predict(&e, 1.0f, 1000.0f);   /* huge, must clamp */
        check("yaw finite after abuse", isfinite(EKF_GetYaw(&e))?0.0f:1.0f, 0.0f, 0.0f);
        check("yaw clamped by EKF_MAX_DT", fabsf(EKF_GetYawDeg(&e)) < 10.0f ?0.0f:1.0f, 0.0f, 0.0f);
        EKF_Predict(NULL, 1.0f, 0.001f);          /* NULL safety */
        EKF_UpdateEncoderYaw(NULL, 0.0f);
        check_covariance(&e,"after abuse");
    }

    printf("\n===== %s (%d failures) =====\n", failures?"FAILURES":"ALL CHECKS PASSED", failures);
    return failures?1:0;
}
