/*
 * QEI.h
 *
 */

#ifndef INC_QEI_H_
#define INC_QEI_H_

#include "motor.h"

// Function prototypes
void QEI_init(void);
int32_t QEI_get_count(motor_t motor);
float QEI_get_velocity(motor_t motor);

#endif /* INC_QEI_H_ */
