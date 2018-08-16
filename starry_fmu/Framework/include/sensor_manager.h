/*
 * File      : sensor.h
 *
 *
 * Change Logs:
 * Date           Author       	Notes
 * 2016-6-20      zoujiachi   	the first version
 */
 
#ifndef __SENSOR_H__
#define __SENSOR_H__

#include <rtthread.h>
#include <stdint.h>
#include "ms5611.h"
#include "global.h"
#include "ap_math.h"

//#define USE_EXTERNAL_MAG_DEV

//#define USE_LIDAR

#ifdef USE_LIDAR
	#ifdef BLUEJAY	
		#define USE_LIDAR_I2C
	#else
		#define USE_LIDAR_PWM
	#endif
#endif

//#define ACC_DEVICE_NAME			"lsm303d"
#define ACC_DEVICE_NAME			"mpu6000"
#ifdef USE_EXTERNAL_MAG_DEV
#define MAG_DEVICE_NAME			"hmc5883"
#else
#define MAG_DEVICE_NAME			"lsm303d"
#endif
//#define GYR_DEVICE_NAME			"l3gd20h"
#define GYR_DEVICE_NAME			"mpu6000"
#define BARO_DEVICE_NAME		"ms5611"
#define GPS_DEVICE_NAME			"gps"
#define LIDAR_DEVICE_NAME		"lidar"

#define RAW_TEMPERATURE_POS			0
#define RAW_PRESSURE_POS			1
#define COLLECT_DATA_POS			2

#define GYR_ACC_UPDATE_INTERVAL		2
#define MAG_UPDATE_INTERVAL			10

#define GYR_RAW_POS					0
#define GYR_SCALE_POS				1
#define ACC_RAW_POS					2
#define ACC_SCALE_POS				3
#define MAG_RAW_POS					4
#define MAG_SCLAE_POS				5

/* control cmd */

//common cmd
#define SENSOR_GET_DEVICE_ID		0x00

//acc,mag cmd
#define SENSOR_SET_ACC_RANGE		0x01
#define SENSOR_SET_ACC_SAMPLERATE	0x02
#define SENSOR_SET_MAG_RANGE		0x03
#define SENSOR_SET_MAG_SAMPLERATE	0x04

//gyr cmd
#define SENSOR_SET_GYR_RANGE		0x20

//baro cmd
#define SENSOR_CONVERSION			0x30
#define SENSOR_IS_CONV_FIN			0x31

typedef enum
{
	S_CONV_1 = 0,
	S_RAW_PRESS,
	S_CONV_2,
	S_RAW_TEMP,
	S_COLLECT_REPORT
}Baro_Machine_State;

typedef struct
{
	float altitude;
	float velocity;
	uint32_t time_stamp;
}BaroPosition;

typedef enum
{
	GPS_UNDETECTED,
	GPS_AVAILABLE,
	GPS_INAVAILABLE
}GPS_Status_Def;

typedef struct
{
	GPS_Status_Def status;
	uint8_t fix_cnt;
}GPS_Status;

typedef struct
{
	Vector3f_t velocity;
	Vector3f_t last_pos;
}GPS_Driv_Vel;

rt_err_t device_sensor_init(void);
void sensor_manager_init(void);
void sensor_loop(void *parameter);

extern float _lidar_dis;
extern uint32_t _lidar_recv_stamp;

/* acc API */
rt_err_t sensor_acc_raw_measure(int16_t acc[3]);
rt_err_t sensor_acc_measure(float acc[3]);
rt_err_t sensor_acc_get_calibrated_data(float acc[3]);

/* mag API */
rt_err_t sensor_mag_raw_measure(int16_t mag[3]);
rt_err_t sensor_mag_measure(float mag[3]);
rt_err_t sensor_mag_get_calibrated_data(float mag[3]);
bool sensor_mag_get_update_flag(void);
void sensor_mag_clear_update_flag(void);

/* gyr API */
rt_err_t sensor_gyr_raw_measure(int16_t gyr[3]);
rt_err_t sensor_gyr_measure(float gyr[3]);
rt_err_t sensor_gyr_get_calibrated_data(float gyr[3]);

/* barometer API */
Baro_Machine_State sensor_baro_get_state(void);
MS5611_REPORT_Def* sensor_baro_get_report(void);
BaroPosition sensor_baro_get_position(void);
rt_err_t sensor_process_baro_state_machine(void);
bool sensor_baro_get_update_flag(void);
void sensor_baro_clear_update_flag(void);

/* lidar-lite API */
float lidar_lite_get_dis(void);
bool lidar_lite_is_connect(void);
bool lidar_is_ready(void);
void lidar_lite_store(float dis);

/* gps API */
struct vehicle_gps_position_s gps_get_report(void);
int gps_get_position(Vector3f_t* gps_pos, struct vehicle_gps_position_s gps_report);
int gps_get_velocity(Vector3f_t* gps_vel, struct vehicle_gps_position_s gps_report);
void gps_calc_geometry_distance(Vector3f_t* dis, double lat1, double lon1, double lat2, double lon2);
void gps_calc_geometry_distance2(Vector3f_t* dis, double ref_lat, double ref_lon, double lat, double lon);
void gps_get_status(GPS_Status* gps_sta);

/* common api */
void sensor_collect(void);
void sensor_get_gyr(float gyr[3]);
void sensor_get_acc(float acc[3]);
void sensor_get_mag(float mag[3]);

#endif
