/**
  ******************************************************************************
  * @file    cbuffer.h
  * @author  Juraj Lonc (juraj.lonc@gmail.com)
  * @brief   Header file of circular buffer.
  ******************************************************************************
  */

#ifndef INC_CBUFFER_H_
#define INC_CBUFFER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stddef.h"
#include "stdint.h"
#include "stdbool.h"
#include <string.h>

typedef struct tag_CircularBuffer {
	uint8_t *buffer;
	size_t size; //of the buffer

	size_t head;
	size_t tail;

} CircularBuffer;

void cbuffer_init(CircularBuffer *cbuf, uint8_t *buffer, size_t size);
void cbuffer_reset(CircularBuffer *cbuf);
bool cbuffer_is_empty(CircularBuffer *cbuf);
bool cbuffer_is_full(CircularBuffer *cbuf);
size_t cbuffer_get_size(CircularBuffer *cbuf);
size_t cbuffer_get_free_size(CircularBuffer *cbuf);
void cbuffer_remove(CircularBuffer *cbuf, size_t len);

// 0 = ok
// -1 = error
int8_t cbuffer_put(CircularBuffer *cbuf, uint8_t *data, size_t len);

// 0 = ok
// -1 = error
int8_t cbuffer_get(CircularBuffer *cbuf, uint8_t *data, size_t len);

// 0 = ok
// -1 = error
int8_t cbuffer_put_object(CircularBuffer *cbuf, uint8_t *data, size_t len);

// return size of object
size_t cbuffer_get_object(CircularBuffer *cbuf, uint8_t *data);

#ifdef __cplusplus
}
#endif

#endif /* INC_CBUFFER_H_ */
