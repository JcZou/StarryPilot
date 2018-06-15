/**
* File      : delay.c
*
*
* Change Logs:
* Date           Author       	Notes
* 2016-6-6    		zoujiachi   	the first version
*/

#include <rtthread.h>
#include "delay.h"

DELAY_TIME_Def _delay_t;

// 获取当前时间，us。
uint64_t time_nowUs(void)
{
    return _delay_t.msPeriod * (uint64_t)1000 + (SysTick->LOAD - SysTick->VAL) / _delay_t.ticksPerUs;
}

// 获取当前时间，ms。
uint32_t time_nowMs(void)
{
    return _delay_t.msPeriod + (SysTick->LOAD - SysTick->VAL) / _delay_t.ticksPerMs;
}

// 延时delay us，delay>=4时才准确。
void time_waitUs(uint32_t delay)
{
    uint64_t target = time_nowUs() + delay;
    while(time_nowUs() < target)
		;
}

// 延时delay ms。
void time_waitMs(uint32_t delay)
{
    time_waitUs(delay * 1000);
}

void device_delay_init(void)
{
	RCC_ClocksTypeDef  rcc_clocks;

    RCC_GetClocksFreq(&rcc_clocks);
	
    _delay_t.msPeriod = 0;  							
    _delay_t.ticksPerUs = rcc_clocks.HCLK_Frequency/8 / 1e6;    	
    _delay_t.ticksPerMs = rcc_clocks.HCLK_Frequency/8 / 1e3;    	
    _delay_t.msPerPeriod = 1000/RT_TICK_PER_SECOND;  	
}
