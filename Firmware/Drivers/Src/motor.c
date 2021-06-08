/*
 * motor.c
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

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

static uint32_t period;

// Private functions
static void motor_l(int32_t speed);
static void motor_r(int32_t speed);

void motor_init(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    SysCtlPWMClockSet(SYSCTL_PWMDIV_1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    GPIOPinConfigure(GPIO_PD0_M0PWM6);
    GPIOPinConfigure(GPIO_PD1_M0PWM7);
    PWMGenConfigure(PWM0_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);

    period = SysCtlClockGet() / 40000;
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, period);
    PWMGenEnable(PWM0_BASE, PWM_GEN_3);

#ifdef DRV8412
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2 | GPIO_PIN_3);
    SysCtlPWMClockSet(SYSCTL_PWMDIV_1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
    GPIOPinConfigure(GPIO_PF2_M1PWM6);
    GPIOPinConfigure(GPIO_PF3_M1PWM7);
    PWMGenConfigure(PWM1_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, period);
    PWMGenEnable(PWM1_BASE, PWM_GEN_3);
#else
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_2 | GPIO_PIN_3);
    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_1 | GPIO_PIN_2);
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, ~GPIO_PIN_2);
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, ~GPIO_PIN_1);
#endif

    motor_set_speed(MOTOR_L, 0);
    motor_set_speed(MOTOR_R, 0);

    SysCtlDelay(100);
}

static void motor_l(int32_t speed)
{
    if (speed >= 0)
    {
#ifdef DRV8412
        PWMPulseWidthSet(MOTOR_L_TIM, PWM_OUT_6, speed);
        PWMPulseWidthSet(MOTOR_L_TIM, PWM_OUT_7, 1);
        PWMOutputState(MOTOR_L_TIM, PWM_OUT_6_BIT | PWM_OUT_7_BIT, true);
#else
        GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, ~GPIO_PIN_2);
        PWMPulseWidthSet(MOTOR_L_TIM, PWM_OUT_6, speed);
        PWMOutputState(MOTOR_L_TIM, PWM_OUT_6_BIT, true);
#endif
    }
    else
    {
#ifdef DRV8412
        PWMPulseWidthSet(MOTOR_L_TIM, PWM_OUT_6, 1);
        PWMPulseWidthSet(MOTOR_L_TIM, PWM_OUT_7, -speed);
        PWMOutputState(MOTOR_L_TIM, PWM_OUT_6_BIT | PWM_OUT_7_BIT, true);
#else
        GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, GPIO_PIN_2);
        PWMPulseWidthSet(MOTOR_L_TIM, PWM_OUT_6, 1000 - abs(speed));
        PWMOutputState(MOTOR_L_TIM, PWM_OUT_6_BIT, true);
#endif
    }
}

static void motor_r(int32_t speed)
{
    if (speed >= 0)
    {
#ifdef DRV8412
        PWMPulseWidthSet(MOTOR_R_TIM, PWM_OUT_6, speed);
        PWMPulseWidthSet(MOTOR_R_TIM, PWM_OUT_7, 1);
        PWMOutputState(MOTOR_R_TIM, PWM_OUT_6_BIT | PWM_OUT_7_BIT, true);
#else
        GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, ~GPIO_PIN_1);
        PWMPulseWidthSet(MOTOR_R_TIM, PWM_OUT_7, speed);
        PWMOutputState(MOTOR_R_TIM, PWM_OUT_7_BIT, true);
#endif
    }
    else
    {
#ifdef DRV8412
        PWMPulseWidthSet(MOTOR_R_TIM, PWM_OUT_6, 1);
        PWMPulseWidthSet(MOTOR_R_TIM, PWM_OUT_7, -speed);
        PWMOutputState(MOTOR_R_TIM, PWM_OUT_6_BIT | PWM_OUT_7_BIT, true);
#else
        GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, GPIO_PIN_1);
        PWMPulseWidthSet(MOTOR_R_TIM, PWM_OUT_7, 1000 - abs(speed));
        PWMOutputState(MOTOR_R_TIM, PWM_OUT_7_BIT, true);
#endif
    }
}

void motor_set_speed(motor_t motor, int32_t speed)
{
    switch (motor)
    {
        case MOTOR_L:
            motor_l(speed + OMEGA_L);
            break;
        case MOTOR_R:
            motor_r(speed + OMEGA_R);
            break;
    }
}
