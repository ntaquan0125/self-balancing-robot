/*
 * LED.c
 *
 */

#include <stdbool.h>
#include <stdint.h>

#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"

#include "LED.h"

void led_init(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
}

void set_led(led_t led, led_state_t state)
{
    switch (led)
    {
    case LED_RED:
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, (state) ? GPIO_PIN_1 : 0);
        break;

    case LED_BLUE:
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, (state) ? GPIO_PIN_2 : 0);
        break;

    case LED_GREEN:
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, (state) ? GPIO_PIN_3 : 0);
        break;
    }
}

void toggle_led(led_t led)
{
    switch (led)
    {
    case LED_RED:
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, ~GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_1));
        break;

    case LED_BLUE:
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, ~GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_2));
        break;

    case LED_GREEN:
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, ~GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_3));
        break;
    }
}
