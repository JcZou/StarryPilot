/*
 * File      : attitude.h
 *
 *
 * Change Logs:
 * Date           Author       	Notes
 * 2016-07-01     zoujiachi   	the first version
 */
 
#ifndef __ATTITUDE_H__
#define __ATTITUDE_H__

#include <rtthread.h>
#include "quaternion.h"

rt_err_t attitude_est_init(void);
quaternion attitude_est_get_quaternion(void);
Euler attitude_est_get_euler(void);
void attitude_inputAcc(const float acc[3]);
void attitude_inputGyr(const float gyr[3]);
void attitude_inputMag(const float mag[3]);
void attitude_loop(void *parameter);
void attitude_est_run(float dT);
void att_gyr_acc_fusion(float dT);
void att_mag_fusion(float dT);
	
#endif
