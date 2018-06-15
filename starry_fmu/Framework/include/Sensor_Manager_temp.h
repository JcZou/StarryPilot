/*
 * File      : Sensor_Manager.h
 *
 *
 * Change Logs:
 * Date           Author       	Notes
 * 2016-11-21     zoujiachi   	the first version
 */
 
#ifndef __SENSOR_MANAGER_H__
#define __SENSOR_MANAGER_H__

#include <rtthread.h>
#include <rtdevice.h>

#define SENSOR_TYPE_NUM		5
#define SENSOR_EVENT_ACC	1
#define SENSOR_EVENT_MAG	2
#define SENSOR_EVENT_GYR	3
#define SENSOR_EVENT_BARO	4
#define SENSOR_EVENT_GPS	5

typedef struct
{
	int16_t 	raw[3];
	float		acc[3];
	
	//calibration parameter
	float		offset[3];
	float		gain[3];
	
	//the timestamp of reading sensor data
	uint32_t	timestamp;
}Acc_Report;

typedef struct
{
	int16_t 	raw[3];
	float		mag[3];
	
	//calibration parameter
	float		offset[3];
	float		gain[3];
	
	uint32_t	timestamp;
}Mag_Report;

typedef struct
{
	int16_t 	raw[3];
	float		gyr[3];
	
	//calibration parameter
	float		offset[3];
	float		gain[3];
	
	uint32_t	timestamp;
}Gyr_Report;

typedef struct
{
	uint32_t 	raw_temperature;
	uint32_t 	raw_pressure;
	float 		temperature;
	float 		pressure;
	float 		altitude;
	
	uint32_t	timestamp;
}Baro_Report;

typedef struct
{
	float		lon, lat;
	float		hMSL;	//Height above mean sea level, unit: m
	float		velN;	//NED north velocity, unit: m/s
	float		velE;	//NED east velocity, unit: m/s
	float		velD;	//NED down velocity, unit: m/s
	uint8_t 	satellites_used;	//Number of satellites used

	uint32_t	timestamp;	
}Gps_Report;

typedef struct
{
	Acc_Report	acc_report;
	Mag_Report	mag_report;
	Gyr_Report	gyr_report;
	Baro_Report	baro_report;
	Gps_Report	gps_report;
}Sensor_Report;

#endif
