/*
 * File      : position.h
 *
 *
 * Change Logs:
 * Date           Author       	Notes
 * 2017-04-30     zoujiachi   	the first version
 */
 
#ifndef __POSITION_H__
#define __POSITION_H__

#include <rtthread.h>
#include <rtdevice.h>
#include "global.h"

typedef struct
{
	double 	lat;		/* Latitude in degrees */
	double 	lon;		/* Lonitude in degrees */
	float	alt;		/* unit: m, NED frame */
	float	lidar_alt;	/* unit: m, NED frame */
	bool	baro_altitude_set;
	bool	lidar_altitude_set;
	bool	gps_coordinate_set;
	float	mag_decl;	/* magnetic declination, in degree */
}HOME_Pos;

typedef enum
{
	BARO_ALT = 0,
	LIDAR_ALT,
	GPS_COORDINATE
}HOME_Item;

typedef struct
{
	float x;
	float y;
	float vx;
	float vy;
	float ax;
	float ay;
	float ax_bias;
	float ay_bias;
}Position_Info;

typedef struct
{
	float alt;	/* absolute altitude, m, up direction */
	float relative_alt;	/* relative altitude, m, up direction */
	float vz;	/* velocity of z direction, m/s, up direction */
	float az;	/* acceleration of z direction, m/s/s, up direction */
	float az_bias;	/* bias of z axis acceleration, up direction */
}Altitude_Info;

typedef struct
{
	float u_x;
	float u_y;
	float u_z;
	float est_x;
	float est_y;
	float est_z;
	float est_vx;
	float est_vy;
	float est_vz;
	float obs_x;
	float obs_y;
	float obs_z;
	float obs_vx;
	float obs_vy;
	float obs_vz;
}POS_KF_Log;

void pos_est_init(float dT);
void pos_est_reset(void);
void pos_est_update(float dT);
HOME_Pos pos_home_get(void);
void pos_get_home(HOME_Pos* home_pos);
void pos_home_set(HOME_Item item, void* data);
//Position_Info get_pos_info(void);
void pos_try_sethome(void);

void save_alt_info(float alt, float relative_alt, float vz, float az, float az_bias);
	
#endif
