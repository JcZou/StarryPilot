/*
 * File      : adrc_att.h
 *
 *
 * Change Logs:
 * Date           Author       	Notes
 * 2018-04-09     zoujiachi   	the first version
 */
 
#ifndef __ADRC_ATT_H__
#define __ADRC_ATT_H__

#include "adrc.h"

typedef struct
{
	float sp_rate;
	float v;
	float v1;
	float v2;
	float z1;
	float z2;
}ADRC_Log;

typedef struct
{
	uint16_t size;
	uint16_t head;
	float *data;
}Delay_Block;

void adrc_att_init(float h);
void adrc_att_reset(float h);
void adrc_att_dis_comp(float* in, float* out);
void adrc_att_control(float euler_err[3], const float gyr[3], float out[3], float bth);
void adrc_att_observer_update(const float gyr[3], float bth);

#endif
