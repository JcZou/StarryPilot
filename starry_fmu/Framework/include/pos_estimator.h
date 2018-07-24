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
}HOME_Pos;

typedef enum
{
	BARO_ALT = 0,
	LIDAR_ALT,
	GPS_COORDINATE
}HOME_Item;

typedef struct
{
 int32_t lat; /*< Latitude, expressed as degrees * 1E7*/
 int32_t lon; /*< Longitude, expressed as degrees * 1E7*/
 int32_t alt; /*< Altitude in meters, expressed as * 1000 (millimeters), AMSL (not WGS84 - note that virtually all GPS modules provide the AMSL as well)*/
 int32_t relative_alt; /*< Altitude above ground in meters, expressed as * 1000 (millimeters)*/
 int16_t vx; /*< Ground X Speed (Latitude, positive north), expressed as m/s * 100*/
 int16_t vy; /*< Ground Y Speed (Longitude, positive east), expressed as m/s * 100*/
 int16_t vz; /*< Ground Z Speed (Altitude, positive down), expressed as m/s * 100*/
}Position_Info;

typedef struct
{
	float alt;	/* absolute altitude, m */
	float relative_alt;	/* relative altitude, m */
	float vz;	/* velocity of z direction, m/s */
	float az;	/* acceleration of z direction, m/s/s */
	float az_bias;	/* bias of z axis acceleration */
}AltInfo;

typedef struct
{
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
void pos_home_set(HOME_Item item, void* data);
Position_Info get_pos_info(void);
	
#endif
