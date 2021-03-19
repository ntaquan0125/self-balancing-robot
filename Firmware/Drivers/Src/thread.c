/*
 * thread.c
 *
 */

#include <stdlib.h>

#include "user_config.h"
#include "thread.h"

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
