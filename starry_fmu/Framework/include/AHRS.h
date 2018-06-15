/*
 * File      : AHRS.h
 *
 *
 * Change Logs:
 * Date           Author       	Notes
 * 2016-07-01     zoujiachi   	the first version
 */
 
#ifndef __AHRS_H__
#define __AHRS_H__

#include "quaternion.h"
#include "ap_math.h"

void AHRS_reset(quaternion * q, const float acc[3],const float mag[3]);
void AHRS_update(quaternion * q,const float gyr[3],const float acc[3],const float mag[3],float dT);
void MahonyAHRS_update(quaternion * q,const float gyr[3],const float acc[3],const float mag[3],float dT);
void MARG_AHRS_update(quaternion* q, float g_x, float g_y, float g_z, float a_x, float a_y, float a_z, float m_x, float m_y, float m_z, float dT);
void AHRS_gyr_acc_fusion(quaternion * q, const float gyr[3], const float acc[3], float dT);
void AHRS_mag_fusion(quaternion * q, const float mag[3], float dT);

#endif
