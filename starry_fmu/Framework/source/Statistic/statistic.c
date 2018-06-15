/*
 * File      : statistic.c
 *
 *
 * Change Logs:
 * Date           Author       	Notes
 * 2017-09-28     zoujiachi   	the first version
 */

#include <rtthread.h>
#include <rthw.h>
#include "statistic.h"
#include "console.h"

/* calculate CPU usage each 100ms */
#define OS_STATISTIC_INTERVAL		500

static uint64_t _os_idle_ctr = 0;
static uint64_t _os_ctr_max;
static float _cpu_usage = 0;

static struct rt_timer timer_sta;

void _thread_idle_hook_func(void)
{
	rt_enter_critical();
	_os_idle_ctr++;
	rt_exit_critical();
}

static void timer_sta_entry(void* parameter)
{
	/* calculate cpu usage */
	_cpu_usage = 100.0f * (1.0f - ((float)_os_idle_ctr)/_os_ctr_max);
	_os_idle_ctr = 0;
}

float get_cpu_usage(void)
{
	float usage;
	
	rt_enter_critical();
	usage = _cpu_usage;
	rt_exit_critical();
	
	return usage;
}

void statistic_init(void)
{
	/* we increment idle counter in idle thread */
	rt_thread_idle_sethook(_thread_idle_hook_func);
	
	rt_enter_critical();
	_os_idle_ctr = 0;
	rt_exit_critical();
	/* we only have idle thread ready now, delay OS_STATISTIC_INTERVAL to count the maximal value of counter */
	rt_thread_delay(OS_STATISTIC_INTERVAL);
	rt_enter_critical();
	_os_ctr_max = _os_idle_ctr;
	rt_exit_critical();
	
	/* register a timer event to calculate CPU usage */
	rt_timer_init(&timer_sta, "timer_sta",
					timer_sta_entry,
					RT_NULL,
					OS_STATISTIC_INTERVAL,
					RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
	rt_timer_start(&timer_sta);
}
