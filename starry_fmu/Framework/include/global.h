/*
 * File      : global.h
 *
 *
 * Change Logs:
 * Date           Author       	Notes
 * 2016-10-02     zoujiachi   	the first version
 */
 
#ifndef __GLOBAL_H__
#define __GLOBAL_H__

#include <stdio.h>
#include <stdlib.h>
/* FPU Library */
#include <arm_math.h>
#include <rtthread.h>
#include <rtdevice.h>
#include <rthw.h>

#define FASTLOOP_THREAD_PRIORITY		3
#define COPTER_THREAD_PRIORITY			4
#define STARRYIO_THREAD_PRIORITY		9
#define FINSH_THREAD_PRIORITY			10
#define LOGGER_THREAD_PRIORITY			11
#define MAVLINK_RX_THREAD_PRIORITY		11
#define MAVLINK_THREAD_PRIORITY			12
#define LED_THREAD_PRIORITY				13
#define CALI_THREAD_PRIORITY			13

#define Rad2Deg(x)			((x)*57.2957795f)
#define Deg2Rad(x)			((x)*0.0174533f)
#define GRAVITY_MSS 		9.81f

//extern const float PI;
#ifndef PI
	#define PI					3.14159265358979f
#endif

#define TIME_GAP(t1,t2)		(((t2)>=(t1))?((t2)-(t1)):(0xFFFFFFFF-(t1)+(t2)))
#define IN_RANGE(v,l,r)		( (v)>(l) && (v)<(r) )

#define OS_ENTER_CRITICAL		rt_enter_critical()
#define OS_EXIT_CRITICAL		rt_exit_critical()
#define OS_MALLOC(size)			rt_malloc(size)
#define OS_FREE(ptr)			rt_free(ptr)

/* HIL simulation */
//#define HIL_SIMULATION

/* global configuration */
//#define AHRS_USE_EKF

typedef int bool;
#define true	1
#define false	0

#endif
