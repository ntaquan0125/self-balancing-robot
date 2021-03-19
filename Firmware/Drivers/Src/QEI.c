/*
 * QEI.c
 *
 */

#include <stdbool.h>
#include <stdint.h>

#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/qei.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"

#include "inc/hw_ints.h"
#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/tm4c123gh6pm.h"

#include "QEI.h"

// Private variables
static int32_t count[2];
static float velocity[2];

// Private functions
static void QEI0_ISR(void);
static void QEI1_ISR(void);

void QEI_init(void)
{
    // Enable the QEI0 peripheral
    SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTD_BASE + GPIO_O_CR) = 0x80;
    HWREG(GPIO_PORTD_BASE + GPIO_O_AFSEL) &= ~0x80;

    QEIConfigure(QEI0_BASE, QEI_CONFIG_CAPTURE_A_B | QEI_CONFIG_NO_RESET | QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_SWAP, 0xffffffff);
    GPIOPinTypeQEI(GPIO_PORTD_BASE, GPIO_PIN_3 | GPIO_PIN_6 | GPIO_PIN_7);

    GPIOPinConfigure(GPIO_PD6_PHA0);
    GPIOPinConfigure(GPIO_PD7_PHB0);
    GPIOPinConfigure(GPIO_PD3_IDX0);

    QEIVelocityConfigure(QEI0_BASE, QEI_VELDIV_1, SysCtlClockGet()/50);
    QEIVelocityEnable(QEI0_BASE);
    QEIIntEnable(QEI0_BASE, QEI_INTINDEX | QEI_INTTIMER);
    QEIIntRegister(QEI0_BASE, &QEI0_ISR);

    // Enable the quadrature encoder.
    QEIEnable(QEI0_BASE);

    QEIPositionSet(QEI0_BASE, 0);

    // Do the same with QE1
    SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);

    QEIConfigure(QEI1_BASE, QEI_CONFIG_CAPTURE_A_B | QEI_CONFIG_NO_RESET | QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_SWAP, 0xffffffff);
    GPIOPinTypeQEI(GPIO_PORTC_BASE, GPIO_PIN_4 |GPIO_PIN_5 | GPIO_PIN_6);

    GPIOPinConfigure(GPIO_PC5_PHA1);
    GPIOPinConfigure(GPIO_PC6_PHB1);
    GPIOPinConfigure(GPIO_PC4_IDX1);

    QEIVelocityConfigure(QEI1_BASE, QEI_VELDIV_1, SysCtlClockGet()/50);
    QEIVelocityEnable(QEI1_BASE);
    QEIIntEnable(QEI1_BASE, QEI_INTINDEX | QEI_INTTIMER);
    QEIIntRegister(QEI1_BASE, &QEI1_ISR);

    QEIEnable(QEI1_BASE);

    QEIPositionSet(QEI1_BASE, 0);

    SysCtlDelay(100);
}

static void QEI0_ISR(void)
{
    if (QEIIntStatus(QEI0_BASE, true) == QEI_INTINDEX)
    {
        count[0] = QEIPositionGet(QEI0_BASE);
    }
    else if (QEIIntStatus(QEI0_BASE, true) == QEI_INTTIMER)
    {
        velocity[0] = (float)QEIVelocityGet(QEI0_BASE) * QEIDirectionGet(QEI0_BASE);
    }
    QEIIntClear(QEI0_BASE, QEIIntStatus(QEI0_BASE, true));
}

static void QEI1_ISR(void)
{
    if (QEIIntStatus(QEI1_BASE, true) == QEI_INTINDEX)
    {
        count[1] = QEIPositionGet(QEI1_BASE);
    }
    else if (QEIIntStatus(QEI1_BASE, true) == QEI_INTTIMER)
    {
        velocity[1] = (float)QEIVelocityGet(QEI1_BASE) * QEIDirectionGet(QEI1_BASE);
    }
    QEIIntClear(QEI1_BASE, QEIIntStatus(QEI1_BASE, true));
}

int32_t QEI_get_count(motor_t motor)
{
    switch (motor)
    {
        case MOTOR_L:
            return count[0];
        case MOTOR_R:
            return count[1];
    }
    return 0;
}

float QEI_get_velocity(motor_t motor)
{
    switch (motor)
    {
        case MOTOR_L:
            return velocity[0];
        case MOTOR_R:
            return velocity[1];
    }
    return 0;
}
