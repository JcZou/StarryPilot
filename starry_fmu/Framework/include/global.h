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

#define COPTER_THREAD_PRIORITY			3

#define SENSOR_THREAD_PRIORITY			5
#define ATTITUDE_THREAD_PRIORITY		6
#define POS_THREAD_PRIORITY				7
#define CONTROL_THREAD_PRIORITY			8
#define STARRYIO_THREAD_PRIORITY		9
#define FINSH_THREAD_PRIORITY			10
#define LOGGER_THREAD_PRIORITY			11
#define MAVLINK_THREAD_PRIORITY			12
#define LED_THREAD_PRIORITY				13

#define Rad2Deg(x)			((x)*57.2957795f)
#define Deg2Rad(x)			((x)*0.0174533f)
//#define Rad2Deg(x)			((x)*180.0f/PI)
//#define Deg2Rad(x)			((x)*PI/180.0f)
#define GRAVITY_MSS 		9.80665f

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

/* global configuration */
#define AHRS_USE_DEFAULT
//#define AHRS_USE_MAHONY
//#define AHRS_USE_MARG

typedef enum
{
	false = 0,
	true = 1,
}bool;

#endif
