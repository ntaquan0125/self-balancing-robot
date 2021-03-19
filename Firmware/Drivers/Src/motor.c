/*
 * motor.c
 *
 */

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "driverlib/sysctl.h"

#include "inc/hw_ints.h"
#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"

#include "user_config.h"
#include "motor.h"

static uint32_t Period;

void motor_init(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    SysCtlPWMClockSet(SYSCTL_PWMDIV_1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    GPIOPinConfigure(GPIO_PD0_M0PWM6);
    GPIOPinConfigure(GPIO_PD1_M0PWM7);
    PWMGenConfigure(PWM0_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);

    Period = SysCtlClockGet() / 40000;
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, Period);
    PWMGenEnable(PWM0_BASE, PWM_GEN_3);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_2 | GPIO_PIN_3);
    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_1 | GPIO_PIN_2);
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, ~GPIO_PIN_2);
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, ~GPIO_PIN_2);
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, ~GPIO_PIN_3);
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, ~GPIO_PIN_1);
    SysCtlDelay(100);
}

void motor_l(int32_t speed)
{
    if (speed > 0)
    {
        GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, 0xFF);
        GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, 0x00);
        PWMPulseWidthSet(MOTOR_L_TIM, PWM_OUT_6, speed);
        PWMOutputState(MOTOR_L_TIM, PWM_OUT_6_BIT, true);
    }
    else
    {
        GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, 0x00);
        GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, 0xFF);
        PWMPulseWidthSet(MOTOR_L_TIM, PWM_OUT_6, abs(speed));
        PWMOutputState(MOTOR_L_TIM, PWM_OUT_6_BIT, true);
    }
}

void motor_r(int32_t speed)
{
    if (speed > 0)
    {
        GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, 0x00);
        GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, 0xFF);
        PWMPulseWidthSet(MOTOR_R_TIM, PWM_OUT_7, speed);
        PWMOutputState(MOTOR_R_TIM, PWM_OUT_7_BIT, true);
    }
    else
    {
        GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, 0xFF);
        GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, 0x00);
        PWMPulseWidthSet(MOTOR_R_TIM, PWM_OUT_7, abs(speed));
        PWMOutputState(MOTOR_R_TIM, PWM_OUT_7_BIT, true);
    }
}

void motor_set_speed(motor_t motor, int32_t speed)
{
    switch (motor)
    {
        case MOTOR_L:
            motor_l(speed);
            break;
        case MOTOR_R:
            motor_r(speed);
            break;
    }
}
