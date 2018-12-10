/*
 * File      : systime.h
 *
 *
 * Change Logs:
 * Date           Author       	Notes
 * 2016-6-6    		zoujiachi   	the first version
 */
 
#ifndef __SYSTIME_H__
#define __SYSTIME_H__

#include "stm32f4xx.h"

typedef struct 
{
    volatile uint32_t msPeriod;		//整周期的时间 , ms
    uint32_t ticksPerUs;  			//每us的tick数 168M/1e6=168
    uint32_t ticksPerMs;  			//每ms的tick数 168M/1e3=168000
    uint32_t msPerPeriod; 			//每周期的ms数
}SYSTIME_Def;

void device_systime_init(void);
uint64_t time_nowUs(void);
uint32_t time_nowMs(void);
void time_waitUs(uint32_t delay);
void time_waitMs(uint32_t delay);

extern SYSTIME_Def _systime_t;

#endif