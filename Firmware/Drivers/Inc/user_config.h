/*
 * user_config.h
 *
 */

#ifndef INC_USER_CONFIG_H_
#define INC_USER_CONFIG_H_

#define MOTOR_L_TIM PWM0_BASE
#define MOTOR_R_TIM PWM1_BASE
#define DRV8412
#define OMEGA_L 0
#define OMEGA_R 0

//#define KALMAN_FILTER

#define ENC_PERIOD          50
#define IMU_PERIOD          20
#define TILT_PERIOD         20
#define TILT_REPORT_PERIOD  200
#define VEL_PERIOD          50
#define VEL_REPORT_PERIOD   200

#define UART_BAUD 115200
#define START_FLAG  0x01
#define END_FLAG    0x0A

#define SAVE_ADDR 0x00030000

#endif /* INC_USER_CONFIG_H_ */
