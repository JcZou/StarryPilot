/*
 * File      : filter.h
 *
 *
 * Change Logs:
 * Date           Author       	Notes
 * 2016-07-01     zoujiachi   	the first version
 */
 
#ifndef __FILTER_H__
#define __FILTER_H__

#include <rtthread.h>
#include "butter.h"
#include "fir.h"
//#include <rtdevice.h>

rt_err_t filter_init(void);
float lpf_get_alpha(float cutoff_freq, float dt);
void accfilter_input(const float val[3]);
void gyrfilter_input(const float val[3]);
void magfilter_input(const float val[3]);
const float * accfilter_current(void);
const float * gyrfilter_current(void);
const float * magfilter_current(void);
void accfilter_read(float* mag);
void gyrfilter_read(float* mag);
void magfilter_read(float* mag);
	
#endif
