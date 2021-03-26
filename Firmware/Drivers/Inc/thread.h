/*
 * thread.h
 *
 */

#ifndef INC_THREAD_H_
#define INC_THREAD_H_

#include <stdbool.h>
#include <stdint.h>

#define MAX_THREAD 10

typedef void (*thread_func_t)(void *);

typedef int8_t thread_id_t;

typedef enum {
	THREAD_REPEAT,
	THREAD_SINGLESHOT
} thread_type_t;

typedef struct {
	thread_func_t thread_func;
	uint16_t period;
	uint16_t counter;
	thread_type_t type;
	void *context;
} thread_t;

thread_t threads[MAX_THREAD];

// Function prototypes
void thread_init(void);
thread_id_t register_thread(thread_func_t thread_func, uint16_t period, void* context, thread_type_t type);
bool unregister_thread(thread_id_t id);
void run_thread(void);

#endif /* INC_THREAD_H_ */
