/*
 * File      : filter.c
 *
 * Change Logs:
 * Date           Author       Notes
 * 2016-07-01     zoujiachi    first version.
 * 2017-09-06	  zoujiachi	   add butterworth filter and fir filter
 */

#include <math.h>
#include "filter.h"
#include "global.h"
#include "sensor_manager.h"
#include "butter.h"

static float g_gyr[3];
static float g_mag[3];
static float g_acc[3];

static Butter2 _butter_acc[3];
static Butter2 _butter_gyr[3];
static Butter2 _butter_mag[3];
static Butter3* _butter3_gyr[3];
static Butter3* _butter3_acc[3];
static Butter3* _butter3_mag[3];


float lpf_get_alpha(float cutoff_freq, float dt)
{
	float rc = 1.0f/(2.0f*PI*cutoff_freq);
	return dt / (dt + rc);
}

void accfilter_init(void)
{
    for(int i=0;i<3;i++)
		g_acc[i] = 0.0f;

#ifdef HIL_SIMULATION
//	/* 30Hz cut-off frequency, 250Hz sampling frequency */
//	float B[4] = {0.0286, 0.0859, 0.0859, 0.0286};
//	float A[4] = {1.0, -1.5189, 0.96, -0.2120};
	butter2_set_cutoff_frequency(&_butter_acc[0], 250, 30);
	butter2_set_cutoff_frequency(&_butter_acc[1], 250, 30);
	butter2_set_cutoff_frequency(&_butter_acc[2], 250, 30);
#else
//	/* 30Hz cut-off frequency, 1000Hz sampling frequency */
//	float B[4] = {0.0007, 0.0021, 0.0021, 0.0007};
//	float A[4] = {1.0, -2.6236, 2.3147, -0.6855};
	butter2_set_cutoff_frequency(&_butter_acc[0], 1000, 30);
	butter2_set_cutoff_frequency(&_butter_acc[1], 1000, 30);
	butter2_set_cutoff_frequency(&_butter_acc[2], 1000, 30);
#endif
//	_butter3_acc[0] = butter3_filter_create(B, A);
//	_butter3_acc[1] = butter3_filter_create(B, A);
//	_butter3_acc[2] = butter3_filter_create(B, A);
	
	/* set initial data */
	butter2_reset(&_butter_acc[0], 0);
	butter2_reset(&_butter_acc[1], 0);
	butter2_reset(&_butter_acc[2], -9.8f);
}

void accfilter_input(const float val[3])
{
    for(uint8_t i=0;i<3;i++)
    {
#ifdef HIL_SIMULATION
		// do not filter for HIL simulation
//		g_acc[i] = val[i];
		g_acc[i] = butter2_filter_process(&_butter_acc[i], val[i]);
#else
		//g_acc[i] = butter3_filter_process(val[i], _butter3_acc[i]);
		g_acc[i] = butter2_filter_process(&_butter_acc[i], val[i]);
		//g_acc[i] = val[i];
#endif
		//g_acc[i] = butter2_filter_process(&_butter_acc[i], val[i]);
		//g_acc[i] = butter3_filter_process(val[i], _butter3_acc[i]);
		//g_acc[i] = val[i];
	}
}

const float * accfilter_current(void)
{
    return g_acc;
}

void accfilter_read(float* acc)
{
	//OS_ENTER_CRITICAL;
	acc[0] = g_acc[0];
	acc[1] = g_acc[1];
	acc[2] = g_acc[2];
	//OS_EXIT_CRITICAL;
}

void gyrfilter_init(void)
{
    for(uint8_t i=0;i<3;i++)
        g_gyr[i] = 0;

#ifdef HIL_SIMULATION
//	/* 30Hz cut-off frequency, 250Hz sampling frequency */
//	float B[4] = {0.0286, 0.0859, 0.0859, 0.0286};
//	float A[4] = {1.0, -1.5189, 0.96, -0.2120};
	butter2_set_cutoff_frequency(&_butter_gyr[0], 250, 30);
	butter2_set_cutoff_frequency(&_butter_gyr[1], 250, 30);
	butter2_set_cutoff_frequency(&_butter_gyr[2], 250, 30);
#else
//	/* 30Hz cut-off frequency, 1000Hz sampling frequency */
//	float B[4] = {0.0007, 0.0021, 0.0021, 0.0007};
//	float A[4] = {1.0, -2.6236, 2.3147, -0.6855};
	butter2_set_cutoff_frequency(&_butter_gyr[0], 1000, 30);
	butter2_set_cutoff_frequency(&_butter_gyr[1], 1000, 30);
	butter2_set_cutoff_frequency(&_butter_gyr[2], 1000, 30);
#endif
//	_butter3_gyr[0] = butter3_filter_create(B, A);
//	_butter3_gyr[1] = butter3_filter_create(B, A);
//	_butter3_gyr[2] = butter3_filter_create(B, A);
	
	/* set initial data */
	butter2_reset(&_butter_gyr[0], 0);
	butter2_reset(&_butter_gyr[1], 0);
	butter2_reset(&_butter_gyr[2], 0);
}

void gyrfilter_input(const float val[3])
{
    for(int i=0;i<3;i++)
	{
#ifdef HIL_SIMULATION
		// do not filter for HIL simulation
		//g_gyr[i] = val[i];
		g_gyr[i] = butter2_filter_process(&_butter_gyr[i], val[i]);
#else
		//g_gyr[i] = butter3_filter_process(val[i], _butter3_gyr[i]);
		g_gyr[i] = butter2_filter_process(&_butter_gyr[i], val[i]);
		//g_gyr[i] = val[i];
#endif
		//g_gyr[i] = butter2_filter_process(&_butter_gyr[i], val[i]);
		//g_gyr[i] = butter3_filter_process(val[i], _butter3_gyr[i]);
		//g_gyr[i] = val[i];
	}
}

const float* gyrfilter_current(void)
{
    return g_gyr;
}

void gyrfilter_read(float* gyr)
{
	gyr[0] = g_gyr[0];
	gyr[1] = g_gyr[1];
	gyr[2] = g_gyr[2];
}

void magfilter_init(void)
{
    for(int i=0;i<3;i++)
        g_mag[i] = 0;
	
//	/* 10Hz cut-off frequency, 100Hz sampling frequency */
//	float B[4] = {0.0181, 0.0543, 0.0543, 0.0181};
//	float A[4] = {1.0000, -1.7600, 1.1829, -0.2781};
//	_butter3_mag[0] = butter3_filter_create(B, A);
//	_butter3_mag[1] = butter3_filter_create(B, A);
//	_butter3_mag[2] = butter3_filter_create(B, A);

	butter2_set_cutoff_frequency(&_butter_mag[0], 100, 30);
	butter2_set_cutoff_frequency(&_butter_mag[1], 100, 30);
	butter2_set_cutoff_frequency(&_butter_mag[2], 100, 30);
	
	/* set initial data */
	butter2_reset(&_butter_mag[0], 0);
	butter2_reset(&_butter_mag[1], 0.7071);
	butter2_reset(&_butter_mag[2], 0.7071);
}

void magfilter_input(const float val[3])
{
    for(int i=0;i<3;i++){
#ifdef HIL_SIMULATION
		// do not filter for HIL simulation
		g_mag[i] = val[i];
		//g_mag[i] = butter2_filter_process(&_butter_mag[i], val[i]);
#else
		g_mag[i] = butter2_filter_process(&_butter_mag[i], val[i]);
		//g_mag[i] = val[i];
#endif
        //g_mag[i] = val[i];
		//g_mag[i] = butter2_filter_process(&_butter_mag[i], val[i]);
		//g_mag[i] = butter3_filter_process(val[i], _butter3_mag[i]);
	}
}

const float * magfilter_current(void)
{
    return g_mag;
}

void magfilter_read(float* mag)
{
	mag[0] = g_mag[0];
	mag[1] = g_mag[1];
	mag[2] = g_mag[2];
}

rt_err_t filter_init(void)
{
    accfilter_init();   
    gyrfilter_init();  
    magfilter_init();
	
	return RT_EOK;
}
