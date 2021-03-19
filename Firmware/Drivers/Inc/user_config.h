/*
 * user_config.h
 *
 */

#ifndef INC_USER_CONFIG_H_
#define INC_USER_CONFIG_H_

#define MOTOR_L_TIM PWM0_BASE
#define MOTOR_R_TIM PWM0_BASE

#define KALMAN_FILTER

#define ENC_PERIOD 100
#define IMU_PERIOD 100
#define TILT_PERIOD 100
#define TILT_REPORT_PERIOD 100
#define VEL_PERIOD 100
#define VEL_REPORT_PERIOD 100

#define UART_BAUD 115200
#define START_FLAG  0x01
#define END_FLAG    0x0A

#define SAVE_ADDR 0x00030000

#endif /* INC_USER_CONFIG_H_ */
