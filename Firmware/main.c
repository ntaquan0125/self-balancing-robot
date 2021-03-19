

/**
 * main.c
 */

#include <stdbool.h>
#include <stdint.h>

#include "driverlib/sysctl.h"

#include "motor.h"
#include "user_config.h"

int32_t count;
float vel;

int main(void)
{
    SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    robot_init();
    robot_test();
    while(1)
    {
//        motor_l(500);
//        count = QEI_get_count(MOTOR_L);
//        vel = QEI_get_velocity(MOTOR_L);
    }
	return 0;
}
