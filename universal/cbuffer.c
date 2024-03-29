/**
  ******************************************************************************
  * @file    cbuffer.c
  * @author  Juraj Lonc (juraj.lonc@gmail.com)
  * @brief   Circular buffer.
  ******************************************************************************
  */

#include "cbuffer.h"

void cbuffer_init(CircularBuffer *cbuf, uint8_t *buffer, size_t size) {
	cbuf->buffer=buffer;
	cbuf->size=size;
	cbuffer_reset(cbuf);
}

void cbuffer_reset(CircularBuffer *cbuf) {
	cbuf->head=0;
	cbuf->tail=0;
}

bool cbuffer_is_empty(CircularBuffer *cbuf) {
	return cbuf->head == cbuf->tail;
}

bool cbuffer_is_full(CircularBuffer *cbuf) {
	return cbuffer_get_free_size(cbuf)==0;
}

size_t cbuffer_get_size(CircularBuffer *cbuf) {
	return cbuf->size - cbuffer_get_free_size(cbuf)-1;
}

size_t cbuffer_get_free_size(CircularBuffer *cbuf) {
	// always keep 1 empty byte as a flag of full buffer. to distinguish from empty buffer.
	if (cbuf->head >= cbuf->tail) {
		return (cbuf->size + cbuf->tail - cbuf->head )-1;
	} else {
		return (cbuf->tail - cbuf->head)-1;
	}
}

int8_t cbuffer_put(CircularBuffer *cbuf, uint8_t *data, size_t len) {
	if (cbuffer_get_free_size(cbuf)< len) {
		return -1;
	}
	size_t l=cbuf->size - cbuf->head;
	if (l>=len) {
		memcpy(&cbuf->buffer[cbuf->head], data, len);
		cbuf->head+=len;
		if (cbuf->head >=cbuf->size) cbuf->head-=cbuf->size;
	} else {
		memcpy(&cbuf->buffer[cbuf->head], data, l);
		memcpy(cbuf->buffer, &data[l], len-l);
		cbuf->head=len-l;
	}
	return 0;
}

int8_t cbuffer_get(CircularBuffer *cbuf, uint8_t *data, size_t len) {
	if (cbuffer_get_size(cbuf)< len) {
		return -1;
	}
	size_t l=cbuf->size - cbuf->tail;
	if (l>=len) {
		memcpy(data, &cbuf->buffer[cbuf->tail], len);
		cbuf->tail+=len;
		if (cbuf->tail >=cbuf->size) cbuf->tail-=cbuf->size;
	} else {
		memcpy(data, &cbuf->buffer[cbuf->tail], l);
		memcpy(&data[l], cbuf->buffer, len-l);
		cbuf->tail=len-l;
	}
	return 0;
}

void cbuffer_remove(CircularBuffer *cbuf, size_t len) {
	if (cbuffer_get_size(cbuf)< len) {
		cbuffer_reset(cbuf);
		return;
	}
	size_t l=cbuf->size - cbuf->tail;
	if (l>=len) {
		cbuf->tail+=len;
		if (cbuf->tail >=cbuf->size) cbuf->tail-=cbuf->size;
	} else {
		cbuf->tail=len-l;
	}
}

int8_t cbuffer_put_object(CircularBuffer *cbuf, uint8_t *data, size_t len) {
	if (cbuffer_get_free_size(cbuf)< len+1) {
		return -1;
	}

	cbuffer_put(cbuf, (uint8_t *)&len, 1);
	cbuffer_put(cbuf, data, len);

	return 0;

}
size_t cbuffer_get_object(CircularBuffer *cbuf, uint8_t *data) {
	if (cbuffer_get_size(cbuf)< 2) {
		return 0;
	}

	size_t len;
	cbuffer_get(cbuf, (uint8_t *)&len, 1);
	cbuffer_get(cbuf, data, len);

	return len;
}
