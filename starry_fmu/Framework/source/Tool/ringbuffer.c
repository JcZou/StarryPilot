/*
 * File      : ringbuffer.c
 *
 * Change Logs:
 * Date           Author       Notes
 * 2017-02-04     zoujiachi   	the first version
 */

#include <stdlib.h>
#include "ringbuffer.h"
#include "console.h"
#include "global.h"

static char* TAG = "Ringbuffer";

ringbuffer* ringbuffer_create(uint16_t size)
{
	ringbuffer* rb = (ringbuffer*)rt_malloc(sizeof(ringbuffer));
	if(rb == NULL){
		Console.e(TAG, "ringbuffer_create fail\r\n");
		return NULL;
	}
	
	rb->buff = (uint8_t*)rt_malloc(size);
	if(rb->buff == NULL){
		Console.e(TAG, "ringbuffer_create fail\r\n");
		return NULL;
	}
	
	rb->size = size;
	rb->head = 0;
	rb->tail = 0;
	rb->static_flag = 0;
	
	return rb;
}

ringbuffer* ringbuffer_static_create(uint8_t* buffer, uint16_t size)
{
	ringbuffer* rb = (ringbuffer*)rt_malloc(sizeof(ringbuffer));
	if(rb == NULL){
		Console.e(TAG, "ringbuffer_static_create fail\r\n");
		return NULL;
	}
	
	rb->buff = buffer;
	
	rb->size = size;
	rb->head = 0;
	rb->tail = 0;
	rb->static_flag = 1;
	
	return rb;
}

void ringbuffer_delete(ringbuffer* rb)
{
	if(!rb->static_flag)
		rt_free(rb->buff);
	rt_free(rb);
}

uint16_t ringbuffer_getlen(ringbuffer* rb)
{
	uint16_t len;
	
	OS_ENTER_CRITICAL;
	if(rb->head >= rb->tail)
		len = rb->head - rb->tail;
	else
		len = rb->head + (rb->size - rb->tail);
	OS_EXIT_CRITICAL;
	
	return len;
}

uint8_t ringbuffer_putc(ringbuffer* rb, uint8_t c)
{
	OS_ENTER_CRITICAL;
	if( (rb->head+1)%rb->size == rb->tail ){
		OS_EXIT_CRITICAL;
		return 0;
	}
	
	rb->buff[rb->head] = c;
	rb->head = (rb->head+1)%rb->size;
	OS_EXIT_CRITICAL;
	
	return 1;
}

uint8_t ringbuffer_getc(ringbuffer* rb)
{
	uint8_t c;

	OS_ENTER_CRITICAL;
	c = rb->buff[rb->tail];
	rb->tail = (rb->tail+1)%rb->size;
	OS_EXIT_CRITICAL;
	
	return c;
}

uint16_t ringbuffer_get(ringbuffer* rb, uint8_t* buffer, uint16_t len)
{
	if( ringbuffer_getlen(rb) < len )
		return 0;
	
	OS_ENTER_CRITICAL;
	for(uint16_t i = 0 ; i < len ; i++){
		buffer[i] = rb->buff[rb->tail];
		rb->tail = (rb->tail+1)%rb->size;	
	}
	OS_EXIT_CRITICAL;

	return len;
}

void ringbuffer_flush(ringbuffer* rb)
{
	OS_ENTER_CRITICAL;
	rb->head = rb->tail = 0;
	OS_EXIT_CRITICAL;
}

