/*
 * motor.h
 *
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include <stdint.h>

typedef enum {
    MOTOR_L,
    MOTOR_R
} motor_t;

// Function prototypes
void motor_init(void);
void motor_set_speed(motor_t motor, int32_t speed);

#endif /* INC_MOTOR_H_ */
