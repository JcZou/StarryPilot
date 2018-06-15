/*
 * File      : MARG_AHRS.h
 *
 *
 * Change Logs:
 * Date           Author       	Notes
 * 2016-07-01     zoujiachi   	the first version
 */
 
#ifndef __MARG_AHRS_H__
#define __MARG_AHRS_H__

#include "quaternion.h"
#include "ap_math.h"

void MARG_AHRS_Update(quaternion* q, float g_x, float g_y, float g_z, float a_x, float a_y, float a_z, float m_x, float m_y, float m_z, float dT);

#endif