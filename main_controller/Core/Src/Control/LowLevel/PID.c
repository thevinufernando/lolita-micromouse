#include "PID.h"

void PIDController_Init(PIDController *pid) {

	/* Clear controller variables */
	pid->integrator = 0.0f;
	pid->prevError  = 0.0f;

	pid->differentiator  = 0.0f;
	pid->prevMeasurement = 0.0f;

	pid->out = 0.0f;

}

float PIDController_Update(PIDController *pid, float setpoint, float measurement) {

	/*
	* Error signal
	*/
    float error = setpoint - measurement;


	/*
	* Proportional
	*/
    float proportional = pid->Kp * error;


	/*
	* Integral
	*/
    pid->integrator = pid->integrator + 0.5f * pid->Ki * pid->T * (error + pid->prevError);

	/* Anti-wind-up via integrator clamping */
    if (pid->integrator > pid->limMaxInt) {

        pid->integrator = pid->limMaxInt;

    } else if (pid->integrator < pid->limMinInt) {

        pid->integrator = pid->limMinInt;

    }


	/*
	* Derivative (band-limited differentiator)
	*
	* Tustin discretisation of  Kd*s / (1 + tau*s)  acting on -y:
	*
	*     D[k] = ( -2*Kd*(y[k] - y[k-1]) + (2*tau - T)*D[k-1] ) / (2*tau + T)
	*
	* The leading minus applies ONLY to the measurement difference (derivative
	* on measurement, so d(error)/dt = -d(y)/dt for a constant setpoint). It
	* must NOT be distributed over the recursive term as well.
	*
	* It used to be, which negated the filter pole and broke the term two ways:
	*   - DC gain became Kd/(2*tau) instead of Kd/T, i.e. HALF the requested Kd
	*     whenever tau == T.
	*   - The pole sat on the negative real axis, so the response alternated
	*     sign every sample. Gain then ROSE from DC to Nyquist instead of
	*     falling, amplifying the sample-to-sample encoder noise this filter
	*     exists to suppress. The error scaled with tau, so raising tau to
	*     filter harder made it worse rather than better.
	*
	* Ramp check (Kd = 1, tau = T = 0.01, measurement rising 1 unit/sample):
	* the true derivative term is -100, which this form settles to. The old
	* form settled to -50.
	*/

    pid->differentiator = (-2.0f * pid->Kd * (measurement - pid->prevMeasurement)
                        + (2.0f * pid->tau - pid->T) * pid->differentiator)
                        / (2.0f * pid->tau + pid->T);


	/*
	* Compute output and apply limits
	*/
    pid->out = proportional + pid->integrator + pid->differentiator;

    if (pid->out > pid->limMax) {

        pid->out = pid->limMax;

    } else if (pid->out < pid->limMin) {

        pid->out = pid->limMin;

    }

	/* Store error and measurement for later use */
    pid->prevError       = error;
    pid->prevMeasurement = measurement;

	/* Return controller output */
    return pid->out;

}