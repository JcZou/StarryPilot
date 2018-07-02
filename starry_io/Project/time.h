/**
  ******************************************************************************
  * @file    time.h 
  * @author  J Zou
  * @version V1.0
  * @date    16-Feb-2017
  * @brief   Time Function
  ******************************************************************************
  */  

#ifndef  _TIME_H_
#define  _TIME_H_

#include "stm32f10x.h"

void time_init(void);
uint64_t time_nowUs(void);
uint32_t time_nowMs(void);
void time_waitUs(uint32_t delay);
void time_waitMs(uint32_t delay);

#endif
