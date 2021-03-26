/*
 * serial.c
 *
 */

#include <stdbool.h>
#include <stdint.h>

#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "inc/hw_ints.h"
#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"

#include "serial.h"

extern params_t robot_params;

// Private variables
static char Rx_buf[20];

// Private functions
static void UART_ISR(void);

void uart_init(uint32_t baudrate)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    GPIOPinConfigure(GPIO_PB0_U1RX);
    GPIOPinConfigure(GPIO_PB1_U1TX);
    GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(), baudrate, UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE);

    UARTIntEnable(UART1_BASE, UART_INT_RX | UART_INT_RT);
    UARTIntRegister(UART1_BASE, &UART_ISR);
    IntEnable(INT_UART1);

    SysCtlDelay(100);
}

static void UART_ISR(void)
{
    UARTIntClear(UART1_BASE, UARTIntStatus(UART1_BASE, true));
    uart_receive(Rx_buf);
    message_decode(Rx_buf, &robot_params);
}

void uart_receive(char *buf)
{
    static uint16_t Rx_index = 0;
    while (UARTCharsAvail(UART1_BASE))
    {
        char c = UARTCharGet(UART1_BASE);
        *(buf + Rx_index) = c;
        Rx_index++;
        if (c == END_FLAG)
        {
            Rx_index = 0;
        }
    }
}

void uart_send(char *buf, uint16_t len)
{
    uint16_t i;
    for (i = 0; i < len; i++)
    {
        UARTCharPut(UART1_BASE, buf[i]);
    }
}
