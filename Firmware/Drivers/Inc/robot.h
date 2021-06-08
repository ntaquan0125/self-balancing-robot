/*
 * robot.h
 *
 */

#ifndef INC_ROBOT_H_
#define INC_ROBOT_H_

#include "LED.h"
#include "MPU6050.h"
#include "PID.h"
#include "QEI.h"
#include "motor.h"
#include "robot.h"
#include "serial.h"
#include "thread.h"
#include "user_config.h"

typedef struct {
    bool is_saved;
    MPU6050_t imu;
    int32_t position[2];
    float velocity[2];
    PID_t pid_tilt;
    PID_t pid_vel;
} params_t;

typedef enum {
    IMU_RPY,
    MOTOR_POS,
    MOTOR_VEL,
    PID_TILT,
    PID_VEL,
    SAVE_LOAD_PARAMS
} msg_id_t;

void robot_init(void);
void robot_test(void);
void save_params(params_t *params);
bool load_params(params_t *params);
bool message_check(char *msg);
bool message_decode(char *msg, params_t *params);
uint8_t message_pack(char *msg, msg_id_t msg_id, params_t params);

#endif /* INC_ROBOT_H_ */
