/*
 * File      : fifo.c
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-07-18     zoujiachi   	the first version
 */
 
#include "fifo.h"
#include "global.h"
#include "console.h"

uint8_t fifo_create(FIFO *fifo, uint16_t size)
{
	fifo->data = (float*)OS_MALLOC(size*sizeof(float));
	if(fifo->data == NULL){
		Console.print("fifo create fail\n");
		return 1;
	}
	fifo->size = size;
	fifo->head = 0;
	fifo->cnt = 0;
	for(int i = 0 ; i < size ; i++){
		fifo->data[i] = 0.0f;
	}
	
	return 0;
}

void fifo_flush(FIFO *fifo)
{
	if(fifo == NULL)
		return;
	fifo->head = 0;
	fifo->cnt = 0;
	for(int i = 0 ; i < fifo->size ; i++){
		fifo->data[i] = 0.0f;
	}
}

void fifo_push(FIFO *fifo, float val)
{
	fifo->head = (fifo->head+1) % fifo->size;
	fifo->data[fifo->head] = val;
	if(fifo->cnt < fifo->size)
		fifo->cnt++;
}

float fifo_pop(FIFO *fifo)
{
	uint16_t tail = (fifo->head+1) % fifo->size;
	return fifo->data[tail];
}

float fifo_read_back(FIFO *fifo, uint16_t offset)
{
	offset = offset % fifo->size;
	uint16_t index = (fifo->head >= offset) ? (fifo->head-offset) : (fifo->head+fifo->size-offset);
	return fifo->data[index];
}