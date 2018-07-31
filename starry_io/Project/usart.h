/**
  ******************************************************************************
  * @file    usart.h 
  * @author  J Zou
  * @version V1.0
  * @date    13-Feb-2017
  * @brief   Usart Function
  ******************************************************************************
  */  

#ifndef  _USART_H_
#define  _USART_H_

#include "stm32f10x.h"

#define RING_BUFFER_SIZE	256

typedef struct
{
	uint16_t head;
	uint16_t tail;
	uint8_t buff[RING_BUFFER_SIZE];
}RING_BUFFER_Def;

uint8_t usart_init(void);
uint8_t read_ch(uint8_t* ch);
uint8_t send_ch(uint8_t ch);
uint16_t send(uint8_t* data, uint16_t len);
void console_putc(uint8_t ch);

#endif
