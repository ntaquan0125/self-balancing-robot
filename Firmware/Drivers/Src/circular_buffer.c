/*
 * circular_buffer.c
 *
 */

#include "circular_buffer.h"

void circular_buf_init(circular_buf_t *cbuf, uint8_t* buffer, size_t size)
{
	cbuf->buffer = buffer;
	cbuf->size = size;
	circular_buf_reset(cbuf);
}

void circular_buf_reset(circular_buf_t *cbuf)
{
	cbuf->head = 0;
	cbuf->tail = 0;
	cbuf->full = false;
}

bool circular_buf_full(circular_buf_t *cbuf)
{
    return cbuf->full;
}

bool circular_buf_empty(circular_buf_t *cbuf)
{
    return (!cbuf->full && (cbuf->head == cbuf->tail));
}

size_t circular_buf_size(circular_buf_t *cbuf)
{
	size_t size = cbuf->size;
	if(!cbuf->full)
	{
		if(cbuf->head >= cbuf->tail)
		{
			size = (cbuf->head - cbuf->tail);
		}
		else
		{
			size = (cbuf->size + cbuf->head - cbuf->tail);
		}
	}
	return size;
}

bool circular_buf_put(circular_buf_t *cbuf, uint8_t data)
{
	if(!circular_buf_full(cbuf))
	{
		cbuf->buffer[cbuf->head] = data;
		cbuf->head = (cbuf->head + 1) % cbuf->size;
		cbuf->full = (cbuf->head == cbuf->tail);
		return true;
	}
	return false;
}

bool circular_buf_get(circular_buf_t *cbuf, uint8_t *data)
{
    if(!circular_buf_empty(cbuf))
    {
        *data = cbuf->buffer[cbuf->tail];
    	cbuf->tail = (cbuf->tail + 1) % cbuf->size;
    	cbuf->full = false;
        return true;
    }
    return false;
}
