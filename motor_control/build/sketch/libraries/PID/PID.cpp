#line 1 "C:\\Users\\rukes\\Documents\\UCB\\FA23\\CS149\\Drone\\motor_control\\libraries\\PID\\PID.cpp"
#include "PID.h"
#include "arduino.h"

void PIDController_Init(PIDController *pid)
{

	/* Clear controller variables */
	pid->integrator = 0.0f;
	pid->prevError = 0.0f;

	pid->differentiator = 0.0f;
	pid->prevMeasurement = 0.0f;

	pid->out = 0.0f;
	pid->T = micros() / 1000000;
}

float PIDController_Update(PIDController *pid, float setpoint, float measurement)
{

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
	float delta_time = pid->T;
	pid->T = (micros() / 1000000);
	delta_time = pid->T - delta_time;
	pid->integrator = pid->integrator + 0.5f * pid->Ki * delta_time * (error + pid->prevError);

	/* Anti-wind-up via integrator clamping */
	if (pid->integrator > pid->limMaxInt)
	{

		pid->integrator = pid->limMaxInt;
	}
	else if (pid->integrator < pid->limMinInt)
	{

		pid->integrator = pid->limMinInt;
	}

	/*
	 * Derivative (band-limited differentiator)
	 */

	pid->differentiator = -(2.0f * pid->Kd * (measurement - pid->prevMeasurement) /* Note: derivative on measurement, therefore minus sign in front of equation! */
							+ (2.0f * pid->tau - delta_time) * pid->differentiator) /
						  (2.0f * pid->tau + delta_time);

	/*
	 * Compute output and apply limits
	 */
	pid->out = proportional + pid->integrator + pid->differentiator;

	if (pid->out > pid->limMax)
	{

		pid->out = pid->limMax;
	}
	else if (pid->out < pid->limMin)
	{

		pid->out = pid->limMin;
	}

	/* Store error and measurement for later use */
	pid->prevError = error;
	pid->prevMeasurement = measurement;

	/* Return controller output */
	return pid->out;
}
