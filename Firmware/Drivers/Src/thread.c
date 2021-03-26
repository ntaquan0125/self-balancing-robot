/*
 * thread.c
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "inc/hw_types.h"

#include "user_config.h"
#include "thread.h"

// Private variables
static void TIM0_ISR(void);

void thread_init(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    uint32_t period = (SysCtlClockGet() / 1000);
    TimerLoadSet(TIMER0_BASE, TIMER_A, period - 1);

    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    TimerIntRegister(TIMER0_BASE, TIMER_A, TIM0_ISR);
    IntEnable(INT_TIMER0A);
    TimerEnable(TIMER0_BASE, TIMER_A);
}

static void TIM0_ISR(void)
{
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    run_thread();
}

thread_id_t register_thread(thread_func_t thread_func, uint16_t period, void* context, thread_type_t type)
{
    uint8_t i;
	for (i = 0; i < MAX_THREAD; i++)
	{
		if (!threads[i].thread_func)
		{
			threads[i].thread_func = thread_func;
			threads[i].period = period;
			threads[i].counter = period;
			threads[i].type = type;
			threads[i].context = context;
			return i;
		}
	}

	// If no thread available
	return -1;
}

bool unregister_thread(thread_id_t id)
{
	if (id >= 0 && threads[id].thread_func)
	{
		threads[id].thread_func == NULL;
		return true;
	}
	return false;
}

void run_thread(void)
{
    uint8_t i;
	for (i = 0; i < MAX_THREAD; i++)
	{
		if (threads[i].thread_func)
		{
			threads[i].counter--;
			if (threads[i].counter == 0)
			{
				threads[i].thread_func(threads[i].context);
				if (threads[i].type == THREAD_REPEAT)
				{
					threads[i].counter = threads[i].period;
				}
				else
				{
					threads[i].thread_func = NULL;
				}
			}
		}
	}
}
