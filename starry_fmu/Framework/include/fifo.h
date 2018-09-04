/*
 * File      : fifo.h
 *
 *
 * Change Logs:
 * Date           Author       	Notes
 * 2018-07-18     zoujiachi   	the first version
 */
 
#ifndef __FIFO_H__
#define __FIFO_H__

#include <stdint.h>

typedef struct
{
	uint32_t size;
	uint32_t head;
	uint32_t cnt;
	float *data;
}FIFO;

uint8_t fifo_create(FIFO *fifo, uint16_t size);
void fifo_flush(FIFO *fifo);
void fifo_push(FIFO *fifo, float val);
float fifo_pop(FIFO *fifo);
float fifo_read_back(FIFO *fifo, uint16_t offset);

#endif
