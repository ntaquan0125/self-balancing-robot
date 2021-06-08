/*
 * PID.c
 *
 */

#include "PID.h"

void PID_init(PID_t *pid, float dt, float out_max, float saturation)
{
	pid->dt = dt;
	pid->out_max = out_max;
	pid->I_saturation = saturation;
	PID_reset(pid);
}

void PID_set_params(PID_t *pid, float Kp, float Ki, float Kd)
{
	pid->Kp = Kp;
	pid->Ki = Ki;
	pid->Kd = Kd;
}

float PID_update(PID_t *pid, float SP, float PV)
{
	pid->error = SP - PV;
	pid->P_part = pid->Kp * pid->error;

	// Trapezoidal approximation
	pid->I_part += 0.5 * pid->Ki * (pid->error + pid->last_err) * pid->dt;

	// Clamping integral
	if (pid->I_part > pid->I_saturation)
	{
		pid->I_part = pid->I_saturation;
	}
	else if (pid->I_part < -pid->I_saturation)
	{
		pid->I_part = -pid->I_saturation;
	}

	pid->D_part = pid->Kd * (pid->error - pid->last_err) / pid->dt;

	// Update state
	pid->last_err = pid->error;

	// Calculate output
	pid->out = pid->P_part + pid->I_part + pid->D_part;
	if (pid->out > pid->out_max)
	{
		pid->out = pid->out_max;
	}
	else if (pid->out < -pid->out_max)
	{
		pid->out = -pid->out_max;
	}

	return pid->out;
}

void PID_reset(PID_t *pid)
{
	pid->Kp = 0;
	pid->Ki = 0;
	pid->Kd = 0;
}
