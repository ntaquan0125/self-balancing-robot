/*
 * circular_buffer.h
 *
 */

#ifndef INC_CIRCULAR_BUFFER_H_
#define INC_CIRCULAR_BUFFER_H_

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

typedef struct {
	uint8_t * buffer;
	size_t head;
	size_t tail;
	size_t size;
	bool full;
} circular_buf_t;

void circular_buf_init(circular_buf_t *cbuf, uint8_t* buffer, size_t size);
void circular_buf_reset(circular_buf_t *cbuf);
bool circular_buf_full(circular_buf_t *cbuf);
bool circular_buf_empty(circular_buf_t *cbuf);
size_t circular_buf_size(circular_buf_t *cbuf);
bool circular_buf_put(circular_buf_t *cbuf, uint8_t data);
bool circular_buf_get(circular_buf_t *cbuf, uint8_t *data);

#endif /* INC_CIRCULAR_BUFFER_H_ */
