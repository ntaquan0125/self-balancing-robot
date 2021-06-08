

/**
 * main.c
 */

#include <stdbool.h>
#include <stdint.h>

#include "driverlib/sysctl.h"

#include "robot.h"
#include "user_config.h"

int main(void)
{
    SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    robot_init();
    robot_test();
    while(1)
    {

    }
	return 0;
}
