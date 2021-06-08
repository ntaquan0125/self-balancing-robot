/*
 * PID.h
 *
 */

#ifndef INC_PID_H_
#define INC_PID_H_

typedef struct {
	float Kp;
	float Ki;
	float Kd;

	float P_part;
	float I_part;
	float D_part;
	float I_saturation;

	float error;
	float last_err;

	float out;
	float out_max;

	float dt;
} PID_t;

// Function prototypes
void PID_init(PID_t *pid, float dt, float out_max, float saturation);
void PID_set_params(PID_t *pid, float Kp, float Ki, float Kd);
float PID_update(PID_t *pid, float SP, float PV);
void PID_reset(PID_t *pid);

#endif /* INC_PID_H_ */
