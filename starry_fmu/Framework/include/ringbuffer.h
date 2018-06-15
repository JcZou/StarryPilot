/*
 * File      : ringbuffer.c
 *
 * Change Logs:
 * Date           Author       Notes
 * 2017-02-04     zoujiachi   	the first version
 */

#ifndef __RINGBUFFER_H__
#define __RINGBUFFER_H__

#include <stdint.h>
 
typedef struct
{
	uint8_t	static_flag;
	unsigned head;
	unsigned tail;
	unsigned size;
	uint8_t* buff;
}ringbuffer;

ringbuffer* ringbuffer_create(uint16_t size);
ringbuffer* ringbuffer_static_create(uint8_t* buffer, uint16_t size);
uint16_t ringbuffer_getlen(ringbuffer* rb);
uint8_t ringbuffer_putc(ringbuffer* rb, uint8_t c);
uint16_t ringbuffer_get(ringbuffer* rb, uint8_t* buffer, uint16_t len);
uint8_t ringbuffer_getc(ringbuffer* rb);
void ringbuffer_flush(ringbuffer* rb);
void ringbuffer_delete(ringbuffer* rb);

#endif
