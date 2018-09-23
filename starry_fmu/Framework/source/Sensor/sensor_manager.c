/*
 * File      : sensor_manager.c
 *
 * Change Logs:
 * Date           Author       Notes
 * 2016-06-20     zoujiachi    first version.
 */
 
#include <rthw.h>
#include <rtdevice.h>
#include <rtthread.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include "console.h"
#include "ms5611.h"
#include "gps.h"
#include "param.h"
#include "sensor_manager.h"
#include "lsm303d.h"
#include "l3gd20h.h"
#include "hmc5883.h"
#include "mpu6000.h"
#include "uMCN.h"
#include "filter.h"
#include "delay.h"
#include "lidar.h"
#include "ap_math.h"
#include "hil_interface.h"
#include "control_main.h"
#include "att_estimator.h"
#include "pos_estimator.h"
#include "calibration.h"

#define ADDR_CMD_CONVERT_D1			0x48	/* write to this address to start pressure conversion */
#define ADDR_CMD_CONVERT_D2			0x58	/* write to this address to start temperature conversion */

#define BARO_UPDATE_INTERVAL    10

#define EARTH_RADIUS			6371000

static char *TAG = "Sensor";

static uint32_t gyr_read_time_stamp = 0;
static uint32_t acc_read_time_stamp = 0;
static uint32_t mag_read_time_stamp = 0;
static uint32_t _baro_update_time_stamp = 0;

static rt_device_t acc_device_t;
static rt_device_t mag_device_t;
static rt_device_t gyr_device_t;
static rt_device_t baro_device_t;
static rt_device_t gps_device_t;
//static rt_device_t lidar_device_t;

//for debug use
static struct vehicle_gps_position_s gps_position;
static struct satellite_info_s satellite_info;

static McnNode_t gps_node_t;

struct rt_event event_vehicle;

static volatile bool _baro_update_flag = false;
static volatile bool _mag_update_flag = false;

static GPS_Status _gps_status;

float _lidar_dis = 0.0f;
uint32_t _lidar_recv_stamp = 0;
static uint32_t _lidar_time = 0;
static float _baro_last_alt = 0.0f;
static uint32_t _baro_last_time = 0;
static BaroPosition _baro_pos = {0.0f, 0.0f, 0.0f};
static GPS_Driv_Vel _gps_driv_vel;
static bool _gps_connected = false;

MCN_DEFINE(SENSOR_MEASURE_GYR, 12);	
MCN_DEFINE(SENSOR_MEASURE_ACC, 12);
MCN_DEFINE(SENSOR_MEASURE_MAG, 12);
MCN_DEFINE(SENSOR_GYR, 12);	
MCN_DEFINE(SENSOR_ACC, 12);
MCN_DEFINE(SENSOR_MAG, 12);
MCN_DEFINE(SENSOR_FILTER_GYR, 12);	
MCN_DEFINE(SENSOR_FILTER_ACC, 12);
MCN_DEFINE(SENSOR_FILTER_MAG, 12);
MCN_DEFINE(SENSOR_BARO, sizeof(MS5611_REPORT_Def));
MCN_DEFINE(SENSOR_LIDAR, sizeof(float));
MCN_DEFINE(CORRECT_LIDAR, sizeof(float));
MCN_DEFINE(BARO_POSITION, sizeof(BaroPosition));
MCN_DEFINE(GPS_STATUS, sizeof(GPS_Status));

MCN_DECLARE(GPS_POSITION);

/**************************	ACC API	**************************/
bool sensor_acc_ready(void)
{
	uint32_t time_now = time_nowMs();
	
	if(acc_read_time_stamp - time_now >= 2){
		return true;
	}else{
		return false;
	}
}

rt_err_t sensor_acc_raw_measure(int16_t acc[3])
{
	rt_size_t r_byte;
	
	r_byte = rt_device_read(acc_device_t, ACC_RAW_POS, (void*)acc, 6);
	
	return r_byte == 6 ? RT_EOK : RT_ERROR;
}

rt_err_t sensor_acc_measure(float acc[3])
{
	rt_size_t r_byte;
	
	acc_read_time_stamp = time_nowMs();
	r_byte = rt_device_read(acc_device_t, ACC_SCALE_POS, (void*)acc, 12);
	
	return r_byte == 12 ? RT_EOK : RT_ERROR;
}

rt_err_t sensor_acc_get_calibrated_data(float acc[3])
{
	float acc_f[3];
	rt_err_t res;
	
	res = sensor_acc_measure(acc_f);
	
	// publish non-calibrated data for calibration					 
	mcn_publish(MCN_ID(SENSOR_MEASURE_ACC), acc_f);

	float ofs[3] = {PARAM_GET_FLOAT(CALIBRATION, ACC_X_OFFSET), 
					PARAM_GET_FLOAT(CALIBRATION, ACC_Y_OFFSET), 
					PARAM_GET_FLOAT(CALIBRATION, ACC_Z_OFFSET)};
	float transM[3][3] = {
		{PARAM_GET_FLOAT(CALIBRATION, ACC_TRANS_MAT00), PARAM_GET_FLOAT(CALIBRATION, ACC_TRANS_MAT01), 
			PARAM_GET_FLOAT(CALIBRATION, ACC_TRANS_MAT02)},
		{PARAM_GET_FLOAT(CALIBRATION, ACC_TRANS_MAT10), PARAM_GET_FLOAT(CALIBRATION, ACC_TRANS_MAT11), 
			PARAM_GET_FLOAT(CALIBRATION, ACC_TRANS_MAT12)},
		{PARAM_GET_FLOAT(CALIBRATION, ACC_TRANS_MAT20), PARAM_GET_FLOAT(CALIBRATION, ACC_TRANS_MAT21), 
			PARAM_GET_FLOAT(CALIBRATION, ACC_TRANS_MAT22)},
	};
	
	float ofs_acc[3];
	for(uint8_t i=0 ; i<3 ; i++){
		ofs_acc[i] = acc_f[i] - ofs[i];
	}
	for(uint8_t i=0 ; i<3 ; i++){
		acc[i] = ofs_acc[0]*transM[0][i] + ofs_acc[1]*transM[1][i] + ofs_acc[2]*transM[2][i];
	}

	return res;
}

/**************************	MAG API	**************************/
bool sensor_mag_ready(void)
{
	uint32_t time_now = time_nowMs();
	
	if( (time_now - mag_read_time_stamp) >= 10){
		return true;
	}else{
		return false;
	}
}

rt_err_t sensor_mag_raw_measure(int16_t mag[3])
{
	rt_size_t r_byte;
	r_byte = rt_device_read(mag_device_t, MAG_RAW_POS, (void*)mag, 6);
	
	return r_byte == 6 ? RT_EOK : RT_ERROR;
}

rt_err_t sensor_mag_measure(float mag[3])
{
	rt_size_t r_byte;
	
	mag_read_time_stamp = time_nowMs();
	r_byte = rt_device_read(mag_device_t, MAG_SCLAE_POS, (void*)mag, 12);
	
	return r_byte == 12 ? RT_EOK : RT_ERROR;
}

rt_err_t sensor_mag_get_calibrated_data(float mag[3])
{
	float mag_f[3];
	rt_err_t res;
	
	res = sensor_mag_measure(mag_f);
	
	// publish non-calibrated data for calibration					 
	mcn_publish(MCN_ID(SENSOR_MEASURE_MAG), mag_f);

#ifdef USE_EXTERNAL_MAG_DEV	
	float ofs[3] = {0.16833, 0.051961, -0.030025};
	float transM[3][3] = {
		{1.8408, -0.028278, -0.013698},
		{-0.028278, 1.7414, 0.0057671},
		{-0.013698, 0.0057671, 1.9104}
	};
#else
	float ofs[3] = {PARAM_GET_FLOAT(CALIBRATION, MAG_X_OFFSET), 
					PARAM_GET_FLOAT(CALIBRATION, MAG_Y_OFFSET), 
					PARAM_GET_FLOAT(CALIBRATION, MAG_Z_OFFSET)};
	float transM[3][3] = {
		{PARAM_GET_FLOAT(CALIBRATION, MAG_TRANS_MAT00), PARAM_GET_FLOAT(CALIBRATION, MAG_TRANS_MAT01), 
			PARAM_GET_FLOAT(CALIBRATION, MAG_TRANS_MAT02)},
		{PARAM_GET_FLOAT(CALIBRATION, MAG_TRANS_MAT10), PARAM_GET_FLOAT(CALIBRATION, MAG_TRANS_MAT11), 
			PARAM_GET_FLOAT(CALIBRATION, MAG_TRANS_MAT12)},
		{PARAM_GET_FLOAT(CALIBRATION, MAG_TRANS_MAT20), PARAM_GET_FLOAT(CALIBRATION, MAG_TRANS_MAT21), 
			PARAM_GET_FLOAT(CALIBRATION, MAG_TRANS_MAT22)},
	};
#endif	

	
	float ofs_mag[3];
	for(uint8_t i=0 ; i<3 ; i++){
		ofs_mag[i] = mag_f[i] - ofs[i];
	}
	for(uint8_t i=0 ; i<3 ; i++){
		mag[i] = ofs_mag[0]*transM[0][i] + ofs_mag[1]*transM[1][i] + ofs_mag[2]*transM[2][i];
	}

	return res;
}

bool sensor_mag_get_update_flag(void)
{
	return _mag_update_flag;
}
void sensor_mag_clear_update_flag(void)
{
	_mag_update_flag = false;
}

/**************************	GYR API	**************************/
bool sensor_gyr_ready(void)
{
	uint32_t time_now = time_nowMs();
	
	if(gyr_read_time_stamp - time_now >= 2){
		return true;
	}else{
		return false;
	}
}

rt_err_t sensor_gyr_raw_measure(int16_t gyr[3])
{
	rt_size_t r_size;
	r_size = rt_device_read(gyr_device_t, GYR_RAW_POS, (void*)gyr, 6);
	
	return r_size == 6 ? RT_EOK : RT_ERROR;
}

rt_err_t sensor_gyr_measure(float gyr[3])
{
	rt_size_t r_size;
	
	gyr_read_time_stamp = time_nowMs();
	r_size = rt_device_read(gyr_device_t, GYR_SCALE_POS, (void*)gyr, 12);
	
	return r_size == 12 ? RT_EOK : RT_ERROR;
}

rt_err_t sensor_gyr_get_calibrated_data(float gyr[3])
{
	float gyr_dps[3];
	rt_err_t res;
	
	float gyr_offset[3] = {PARAM_GET_FLOAT(CALIBRATION, GYR_X_OFFSET),
						   PARAM_GET_FLOAT(CALIBRATION, GYR_Y_OFFSET),
						   PARAM_GET_FLOAT(CALIBRATION, GYR_Z_OFFSET)};
	float gyr_gain[3] = {PARAM_GET_FLOAT(CALIBRATION, GYR_X_GAIN),
						 PARAM_GET_FLOAT(CALIBRATION, GYR_Y_GAIN),
						 PARAM_GET_FLOAT(CALIBRATION, GYR_Z_GAIN)};
	
	res = sensor_gyr_measure(gyr_dps);
						 
	// publish non-calibrated data for calibration					 
	mcn_publish(MCN_ID(SENSOR_MEASURE_GYR), gyr_dps);
	
	for(uint8_t i=0 ; i<3 ; i++)
	{
		gyr[i] = (gyr_dps[i] + gyr_offset[i]) * gyr_gain[i];
	}

	return res;
}

uint8_t sensor_get_device_id(char* device_name)
{
	uint8_t device_id = 0xFF;	//unknown device
	
	if(strcmp(device_name , ACC_DEVICE_NAME) == 0)
	{
		rt_device_control(acc_device_t, SENSOR_GET_DEVICE_ID, (void*)&device_id);
	}
	else if(strcmp(device_name , MAG_DEVICE_NAME) == 0)
	{
		rt_device_control(mag_device_t, SENSOR_GET_DEVICE_ID, (void*)&device_id);
	}
	else if(strcmp(device_name , GYR_DEVICE_NAME) == 0)
	{
		rt_device_control(gyr_device_t, SENSOR_GET_DEVICE_ID, (void*)&device_id);
	}
	
	return device_id;
}

/**************************	BARO API **************************/
static Baro_Machine_State baro_state;
static MS5611_REPORT_Def report_baro;

rt_err_t _baro_trig_conversion(uint8_t addr)
{
	return rt_device_control(baro_device_t, SENSOR_CONVERSION, (void*)&addr);
}

rt_bool_t _baro_is_conv_finish(void)
{
	if(rt_device_control(baro_device_t, SENSOR_IS_CONV_FIN, RT_NULL) == RT_EOK)
	{
		return RT_TRUE;
	}else
	{
		return RT_FALSE;
	}
}

rt_err_t _baro_read_raw_temp(void)
{
	rt_err_t err;
	if(rt_device_read(baro_device_t, RAW_TEMPERATURE_POS, NULL, 1))
		err = RT_EOK;
	else
		err = RT_ERROR;
	
	return err;
}

rt_err_t _baro_read_raw_press(void)
{
	rt_err_t err;
	if(rt_device_read(baro_device_t, RAW_PRESSURE_POS, NULL, 1))
		err = RT_EOK;
	else
		err = RT_ERROR;
	
	return err;
}

/* 
* There are 5 steps to get barometer report
* 1: convert D1
* 2: read pressure raw data
* 3: convert D2
* 4: read temperature raw dara
* 5: compute temperature,pressure,altitute according to prom param.
*/
rt_err_t sensor_process_baro_state_machine(void)
{
	rt_err_t err = RT_ERROR;
	
	switch((uint8_t)baro_state)
	{
		case S_CONV_1:
		{
			err = _baro_trig_conversion(ADDR_CMD_CONVERT_D1);
			if(err == RT_EOK)
				baro_state = S_CONV_2;
		}break;
		case S_CONV_2:
		{
			if(!_baro_is_conv_finish()){	//need 9.04ms to converse
				err = RT_EBUSY;
			}else{
				err = _baro_read_raw_press();
				if(err == RT_EOK){
					/* directly start D2 conversion */
					err = _baro_trig_conversion(ADDR_CMD_CONVERT_D2);
					if(err == RT_EOK)
						baro_state = S_COLLECT_REPORT;
					else
						baro_state = S_CONV_1;
				}
				else
					baro_state = S_CONV_1;	//if err, restart
			}
		}break;
		case S_COLLECT_REPORT:
		{
			if(!_baro_is_conv_finish()){	//need 9.04ms to converse
				err = RT_EBUSY;
			}else{
				baro_state = S_CONV_1;
				err = _baro_read_raw_temp();
				if(err == RT_EOK){
					if(rt_device_read(baro_device_t, COLLECT_DATA_POS, (void*)&report_baro, 1)){
						/* start D1 conversion */
						if(_baro_trig_conversion(ADDR_CMD_CONVERT_D1) == RT_EOK)
							baro_state = S_CONV_2;
					}else{
						err = RT_ERROR;
					}
				}
			}
		}break;
	}
	
	return err;
}

bool sensor_baro_ready(void)
{
	uint32_t time_now = time_nowMs();
	if( (time_now - _baro_update_time_stamp) >= 10){
		_baro_update_time_stamp = time_now;
		return true;
	}else{
		return false;
	}
}

bool sensor_baro_get_update_flag(void)
{
#ifdef HIL_SIMULATION
	return hil_baro_poll();
#else
	return _baro_update_flag;
#endif
}
void sensor_baro_clear_update_flag(void)
{
	_baro_update_flag = false;
}

bool sensor_baro_update(void)
{
	rt_err_t res;
	
	if(sensor_baro_get_state() == S_COLLECT_REPORT){
		res = sensor_process_baro_state_machine();
		//get report;
		if(res == RT_EOK){
			_baro_update_flag = true;
			return true;
		}
	}else{
		res = sensor_process_baro_state_machine();
	}

	return false;
}


Baro_Machine_State sensor_baro_get_state(void)
{
	return baro_state;
}

MS5611_REPORT_Def* sensor_baro_get_report(void)
{
#ifdef HIL_SIMULATION
	mcn_copy_from_hub(MCN_ID(SENSOR_BARO), &report_baro);
#endif
	return &report_baro;
}

BaroPosition sensor_baro_get_position(void)
{
	return _baro_pos;
}

/**************************	LIDAR-LITE API **************************/
void lidar_lite_store(float dis)
{
	OS_ENTER_CRITICAL;
	_lidar_dis = dis;
	_lidar_recv_stamp = time_nowMs();
	OS_EXIT_CRITICAL;
}

float lidar_lite_get_dis(void)
{
	float distance;

#ifdef USE_LIDAR_PWM	
	OS_ENTER_CRITICAL;
	distance = _lidar_dis;
	OS_EXIT_CRITICAL;	
	_lidar_time = time_nowMs();
#elif defined USE_LIDAR_I2C
	rt_size_t size = rt_device_read(lidar_device_t, 1, &distance, 1);
	if(size != 1)
		return -1.0f;
	_lidar_time = time_nowMs();
#else
	Console.e(TAG, "err, do not define to use lidar\n");
#endif

	/* compensate distance with angle */
	quaternion att = attitude_est_get_quaternion();
//	float zn[3] = {0.0f, 0.0f, 1.0f};
//	float zb[3];
//	quaternion_inv_rotateVector(att, zn, zb);
//	float cos_tilt = fabs(Vector3_DotProduct(zn, zb));
//	float cor_dis = distance * cos_theta;
	Euler e;
	quaternion_toEuler(&att, &e);
	float cos_tilt = arm_cos_f32(e.roll)*arm_cos_f32(e.pitch);
	float cor_dis = distance * cos_tilt;
	
	mcn_publish(MCN_ID(SENSOR_LIDAR), &distance);
	mcn_publish(MCN_ID(CORRECT_LIDAR), &cor_dis);
	
	return cor_dis;
}

bool lidar_lite_is_connect(void)
{
	uint32_t time_now = time_nowMs();
	uint32_t time_elapse = (time_now>=_lidar_recv_stamp) ? (time_now-_lidar_recv_stamp) : (0xFFFFFFFF-_lidar_recv_stamp+time_now);
	
	/* if more than 50ms no lidar data is received, then we think lidar is disconected */
	if(time_elapse < 50){
		return true;
	}else{
		return false;
	}
}

bool lidar_is_ready(void)
{
	uint32_t time_now = time_nowMs();
	/* read lidar each 20ms */
	uint32_t time_elapse = (time_now>=_lidar_time) ? (time_now-_lidar_time) : (0xFFFFFFFF-_lidar_time+time_now);
	if(time_elapse >= 20){
		return true;
	}else{
		return false;
	}
}

//////////////// GPS Function ///////////////////////
void gps_calc_geometry_distance(Vector3f_t* dis, double ref_lat, double ref_lon, double lat, double lon)
{
	double delta_lat = Deg2Rad(lat - ref_lat);
	double delta_lon = Deg2Rad(lon - ref_lon);
	
	dis->x = (float)(delta_lat * EARTH_RADIUS);
	dis->y = (float)(delta_lon * EARTH_RADIUS * arm_cos_f32(lat));
}

void gps_calc_geometry_distance2(Vector3f_t* dis, double ref_lat, double ref_lon, double lat, double lon)
{
	const double lat_rad = Deg2Rad(lat);
	const double lon_rad = Deg2Rad(lon);

	const double sin_lat = sin(lat_rad);
	const double cos_lat = cos(lat_rad);

	const double cos_d_lon = cos(lon_rad - Deg2Rad(ref_lon));

	const double arg = constrain_float(sin(Deg2Rad(ref_lat)) * sin_lat + cos(Deg2Rad(ref_lat)) * cos_lat * cos_d_lon, -1.0,  1.0);
	const double c = acos(arg);

	double k = 1.0;

	if (fabs(c) > 0) {
		k = (c / sin(c));
	}

	dis->x = (float)(k * (cos(Deg2Rad(ref_lat)) * sin_lat - sin(Deg2Rad(ref_lat)) * cos_lat * cos_d_lon) * EARTH_RADIUS);
	dis->y = (float)(k * cos_lat * sin(lon_rad - Deg2Rad(ref_lon)) * EARTH_RADIUS);
}

struct vehicle_gps_position_s gps_get_report(void)
{
	struct vehicle_gps_position_s gps_pos_t;
	
	mcn_copy_from_hub(MCN_ID(GPS_POSITION), &gps_pos_t);
	
	return gps_pos_t;
}

int gps_get_position(Vector3f_t* gps_pos, struct vehicle_gps_position_s gps_report)
{
	HOME_Pos home = pos_home_get();
	if(home.gps_coordinate_set == false){
		// gps home have not set yet
		return -1;
	}

	//gps_calc_geometry_distance2(gps_pos, home.lat, home.lon, (double)gps_report.lat*1e-7, (double)gps_report.lon*1e-7);
	gps_calc_geometry_distance(gps_pos, home.lat, home.lon, (double)gps_report.lat*1e-7, (double)gps_report.lon*1e-7);
	gps_pos->z = (float)gps_report.alt*1e-3;
	
	return 0;
}

int gps_get_velocity(Vector3f_t* gps_vel, struct vehicle_gps_position_s gps_report)
{
#ifdef USE_GPS_VEL
	gps_vel->x = gps_report.vel_n_m_s;
	gps_vel->y = gps_report.vel_e_m_s;
	gps_vel->z = gps_report.vel_d_m_s;
#else
	OS_ENTER_CRITICAL;
	gps_vel->x = _gps_driv_vel.velocity.x;
	gps_vel->y = _gps_driv_vel.velocity.y;
	gps_vel->z = _gps_driv_vel.velocity.z;
	OS_EXIT_CRITICAL;
#endif
	
	return 0;
}

void gps_get_status(GPS_Status* gps_sta)
{
	mcn_copy_from_hub(MCN_ID(GPS_STATUS), gps_sta);
}

/************************** Public API ***************************/
void sensor_get_gyr(float gyr[3])
{
	mcn_copy_from_hub(MCN_ID(SENSOR_FILTER_GYR), gyr);
}

void sensor_get_acc(float acc[3])
{
	mcn_copy_from_hub(MCN_ID(SENSOR_FILTER_ACC), acc);
}

void sensor_get_mag(float mag[3])
{
	mcn_copy_from_hub(MCN_ID(SENSOR_FILTER_MAG), mag);
}

/**************************	INIT FUNC **************************/
rt_err_t device_sensor_init(void)
{
	rt_err_t res = RT_EOK;

	/* init all sensor drivers */
	res |= rt_lsm303d_init("spi_d1");
	res |= rt_l3gd20h_init("spi_d2");
#ifdef USE_EXTERNAL_MAG_DEV
	res |= rt_hmc5883_init("i2c1");
#endif
	res |= rt_ms5611_init("spi_d3");
	res |= rt_mpu6000_init("spi_d4");
	res |= rt_gps_init("uart4" , &gps_position , &satellite_info);
	
	/* init acc device */
	acc_device_t = rt_device_find(ACC_DEVICE_NAME);
	if(acc_device_t == RT_NULL)
	{
		Console.e(TAG, "can't find acc device\r\n");
		return RT_EEMPTY;
	}
	rt_device_open(acc_device_t , RT_DEVICE_OFLAG_RDWR);

	/* init mag device */
	mag_device_t = rt_device_find(MAG_DEVICE_NAME);
	if(mag_device_t == RT_NULL)
	{
		Console.e(TAG, "can't find mag device\r\n");
		return RT_EEMPTY;
	}else{
		rt_device_open(mag_device_t , RT_DEVICE_OFLAG_RDWR);
	}
	
	/* init gyr device */
	gyr_device_t = rt_device_find(GYR_DEVICE_NAME);
	if(gyr_device_t == RT_NULL)
	{
		Console.e(TAG, "can't find gyr device\r\n");
		return RT_EEMPTY;
	}
	rt_device_open(gyr_device_t , RT_DEVICE_OFLAG_RDWR);
	
	/* init barometer device */
	baro_state = S_CONV_1;
	baro_device_t = rt_device_find(BARO_DEVICE_NAME);
	if(baro_device_t == RT_NULL)
	{
		Console.e(TAG, "can't find baro device\r\n");
		return RT_EEMPTY;
	}
	rt_device_open(baro_device_t , RT_DEVICE_OFLAG_RDWR);
	
	/* init gps device */
	gps_device_t = rt_device_find(GPS_DEVICE_NAME);
	if(gps_device_t == RT_NULL)
	{
		Console.e(TAG, "can't find gps device\r\n");
		return RT_EEMPTY;
	}
	rt_err_t gps_open_res = rt_device_open(gps_device_t , RT_DEVICE_OFLAG_RDWR);
	_gps_connected = gps_open_res == RT_EOK ? true : false;

#ifdef USE_LIDAR_I2C	
	/* init lidar lite device */
	rt_lidar_init("i2c1");
	lidar_device_t = rt_device_find(LIDAR_DEVICE_NAME);
	if(lidar_device_t == RT_NULL)
	{
		Console.e(TAG, "can't find %s device\r\n", LIDAR_DEVICE_NAME);
		return RT_EEMPTY;
	}
	rt_device_open(lidar_device_t , RT_DEVICE_OFLAG_RDWR);
#endif
	
	float null_data[3] = {0, 0, 0};
	/* advertise sensor data */
	int mcn_res;
	mcn_res = mcn_advertise(MCN_ID(SENSOR_MEASURE_GYR));
	if(mcn_res != 0){
		Console.e(TAG, "err:%d, SENSOR_MEASURE_GYR advertise fail!\n", mcn_res);
	}
	mcn_res = mcn_advertise(MCN_ID(SENSOR_MEASURE_ACC));
	if(mcn_res != 0){
		Console.e(TAG, "err:%d, SENSOR_MEASURE_ACC advertise fail!\n", mcn_res);
	}
	mcn_res = mcn_advertise(MCN_ID(SENSOR_MEASURE_MAG));
	if(mcn_res != 0){
		Console.e(TAG, "err:%d, SENSOR_MEASURE_MAG advertise fail!\n", mcn_res);
	}
	mcn_res = mcn_advertise(MCN_ID(SENSOR_GYR));
	if(mcn_res != 0){
		Console.e(TAG, "err:%d, sensor_gyr advertise fail!\n", mcn_res);
	}
	mcn_res = mcn_advertise(MCN_ID(SENSOR_ACC));
	if(mcn_res != 0){
		Console.e(TAG, "err:%d, sensor_acc advertise fail!\n", mcn_res);
	}
	mcn_res = mcn_advertise(MCN_ID(SENSOR_MAG));
	if(mcn_res != 0){
		Console.e(TAG, "err:%d, sensor_mag advertise fail!\n", mcn_res);
	}
	mcn_res = mcn_advertise(MCN_ID(SENSOR_FILTER_GYR));
	if(mcn_res != 0){
		Console.e(TAG, "err:%d, sensor_filter_gyr advertise fail!\n", mcn_res);
	}
	mcn_publish(MCN_ID(SENSOR_FILTER_GYR), &null_data);
	mcn_res = mcn_advertise(MCN_ID(SENSOR_FILTER_ACC));
	if(mcn_res != 0){
		Console.e(TAG, "err:%d, sensor_filter_acc advertise fail!\n", mcn_res);
	}
	mcn_publish(MCN_ID(SENSOR_FILTER_ACC), &null_data);
	mcn_res = mcn_advertise(MCN_ID(SENSOR_FILTER_MAG));
	if(mcn_res != 0){
		Console.e(TAG, "err:%d, sensor_filter_mag advertise fail!\n", mcn_res);
	}
	mcn_publish(MCN_ID(SENSOR_FILTER_MAG), &null_data);
	mcn_res = mcn_advertise(MCN_ID(SENSOR_BARO));
	if(mcn_res != 0){
		Console.e(TAG, "err:%d, sensor_baro advertise fail!\n", mcn_res);
	}
	mcn_res = mcn_advertise(MCN_ID(SENSOR_LIDAR));
	if(mcn_res != 0){
		Console.e(TAG, "err:%d, sensor_lidar advertise fail!\n", mcn_res);
	}
	mcn_res = mcn_advertise(MCN_ID(CORRECT_LIDAR));
	if(mcn_res != 0){
		Console.e(TAG, "err:%d, correct_lidar advertise fail!\n", mcn_res);
	}
	mcn_res = mcn_advertise(MCN_ID(BARO_POSITION));
	if(mcn_res != 0){
		Console.e(TAG, "err:%d, baro_position advertise fail!\n", mcn_res);
	}
	mcn_res = mcn_advertise(MCN_ID(GPS_STATUS));
	if(mcn_res != 0){
		Console.e(TAG, "err:%d, GPS_STATUS advertise fail!\n", mcn_res);
	}
	
	gps_node_t = mcn_subscribe(MCN_ID(GPS_POSITION), NULL);
	if(gps_node_t == NULL)
		Console.e(TAG, "gps_node_t subscribe err\n");
	
	_baro_last_alt = 0.0f;
	_gps_status.status = GPS_UNDETECTED;
	_gps_status.fix_cnt = 0;
	// publish init gps status
	mcn_publish(MCN_ID(GPS_STATUS), &_gps_status);
	
	_gps_driv_vel.velocity.x = _gps_driv_vel.velocity.y = _gps_driv_vel.velocity.z = 0.0f;
	_gps_driv_vel.last_pos.x = _gps_driv_vel.last_pos.y = _gps_driv_vel.last_pos.z = 0.0f;
	
	return res;
}

void sensor_collect(void)
{
	float gyr[3], acc[3], mag[3];

	if(sensor_gyr_get_calibrated_data(gyr) == RT_EOK){
		gyrfilter_input(gyr);
		mcn_publish(MCN_ID(SENSOR_GYR), gyr);
		mcn_publish(MCN_ID(SENSOR_FILTER_GYR), gyrfilter_current());
	}else{
		Console.e(TAG, "fail to get gyr data\n");
	}
	
	if(sensor_acc_get_calibrated_data(acc) == RT_EOK){
		accfilter_input(acc);
		mcn_publish(MCN_ID(SENSOR_ACC), acc);
		mcn_publish(MCN_ID(SENSOR_FILTER_ACC), accfilter_current());
	}else{
		Console.e(TAG, "fail to get acc data\n");
	}
	
	if(sensor_mag_ready()){
		if(sensor_mag_get_calibrated_data(mag) == RT_EOK){
			magfilter_input(mag);
			mcn_publish(MCN_ID(SENSOR_MAG), mag);
			mcn_publish(MCN_ID(SENSOR_FILTER_MAG), magfilter_current());
			_mag_update_flag = true;
		}else{
			Console.e(TAG, "fail to get mag data\n");
		}
	}

	if(sensor_baro_ready()){
		if(sensor_baro_update()){
			
			MS5611_REPORT_Def* baro_report = sensor_baro_get_report();

			float dt = (float)(baro_report->time_stamp-_baro_last_time)*1e-3;
			_baro_pos.time_stamp = baro_report->time_stamp;
			if(dt <= 0.0f)
				dt = 0.02f;
			
			_baro_pos.altitude = -baro_report->altitude; // change to NED coordinate
			float vel = (_baro_pos.altitude-_baro_last_alt)/dt;
			_baro_pos.velocity = _baro_pos.velocity + 0.05*(vel-_baro_pos.velocity);
			
			mcn_publish(MCN_ID(SENSOR_BARO), baro_report);
			mcn_publish(MCN_ID(BARO_POSITION), &_baro_pos);
			
			_baro_last_alt = _baro_pos.altitude;
			_baro_last_time = baro_report->time_stamp;
		}
	}
	
	if(mcn_poll(gps_node_t)){
		
		struct vehicle_gps_position_s gps_pos_t;
		mcn_copy(MCN_ID(GPS_POSITION), gps_node_t, &gps_pos_t);
		
		HOME_Pos home = pos_home_get();
		if(home.gps_coordinate_set == true){
			Vector3f_t pos;
			gps_get_position(&pos, gps_pos_t);
			
			_gps_driv_vel.velocity.x = (pos.x - _gps_driv_vel.last_pos.x) / 0.1f;	// the gps update interval is 100ms
			_gps_driv_vel.velocity.y = (pos.y - _gps_driv_vel.last_pos.y) / 0.1f;
			_gps_driv_vel.velocity.z = (pos.z - _gps_driv_vel.last_pos.z) / 0.1f;
			
			_gps_driv_vel.last_pos.x = pos.x;
			_gps_driv_vel.last_pos.y = pos.y;
			_gps_driv_vel.last_pos.z = pos.z;
		}
		
		// check legality
		if(_gps_status.status!=GPS_AVAILABLE && gps_pos_t.satellites_used>=6 && IN_RANGE(gps_pos_t.eph, 0.0f, 2.5f)){
			
			_gps_status.fix_cnt++;
			
			if(_gps_status.fix_cnt >= 10){
				_gps_status.status = GPS_AVAILABLE;
				
				// gps becomes available, publish
				mcn_publish(MCN_ID(GPS_STATUS), &_gps_status);
			}
		}
		if(_gps_status.status!=GPS_INAVAILABLE && (gps_pos_t.satellites_used<=4 || gps_pos_t.eph>3.5f)){
			
			_gps_status.status = GPS_INAVAILABLE;
			_gps_status.fix_cnt = 0;
			
			mcn_publish(MCN_ID(GPS_STATUS), &_gps_status);
		}
	}
}

void sensor_manager_init(void)
{
	// do something here
}

int handle_gps_shell_cmd(int argc, char** argv)
{
	if(argc > 1){
		if(strcmp(argv[1], "status") == 0){
			char status_str[20] = "";
			GPS_Status gps_status;
			mcn_copy_from_hub(MCN_ID(GPS_STATUS), &gps_status);
				
			if(gps_status.status == GPS_UNDETECTED){
				strcpy(status_str, "UNDETECTED");
			}
			else if(gps_status.status == GPS_AVAILABLE){
				strcpy(status_str, "AVAILABLE");
			}
			else{
				strcpy(status_str, "INAVAILABLE");
			}
			
			struct vehicle_gps_position_s gps_pos = gps_get_report();
			
			Console.print("gps status: %s, satelites:%d, fix type:%d [eph,epv]:[%.3f %.3f], [hdop,vdop]:[%.3f %.3f]\n", status_str, gps_pos.satellites_used, 
				gps_pos.fix_type, gps_pos.eph, gps_pos.epv, gps_pos.hdop, gps_pos.vdop);
		}
	}
	
	return 0;
}

int handle_sensor_shell_cmd(int argc, char** argv)
{
	uint8_t sensor_type = 0;
	uint32_t interval = 1000;	//default is 1s
	uint32_t cnt = 1;
	uint8_t raw_data = 0;
	uint8_t no_cali = 0;
	
	if(argc > 1){
		if(strcmp(argv[1], "acc") == 0){
			sensor_type = 1;
		}
		else if(strcmp(argv[1], "mag") == 0){
			sensor_type = 2;
		}
		else if(strcmp(argv[1], "gyr") == 0){
			sensor_type = 3;
		}else if(strcmp(argv[1], "gps") == 0){
			sensor_type = 4;
		}else{
			Console.print("unknow parameter:%s\n", argv[1]);
			return 1;
		}
		
		for(uint16_t i = 2 ; i < argc ; i++){
			if(strcmp(argv[i], "-t") == 0){
				i++;
				if(i >= argc){
					Console.print("wrong cmd format.\n");
					return 2;
				}
				interval = atoi(argv[i]);
			}
			if(strcmp(argv[i], "-n") == 0){
				i++;
				if(i >= argc){
					Console.print("wrong cmd format.\n");
					return 2;
				}
				cnt = atoi(argv[i]);
			}
			if(strcmp(argv[i], "-r") == 0){
				raw_data = 1;
			}
			if(strcmp(argv[i], "-nc") == 0){
				no_cali = 1;
			}
		}
		
		switch(sensor_type)
		{
			case 1:	//acc
			{
				for(uint32_t i = 0 ; i < cnt ; i++){
					if(raw_data){
						int16_t raw_acc[3];
						sensor_acc_raw_measure(raw_acc);
						Console.print("raw acc:%d %d %d\n", raw_acc[0], raw_acc[1], raw_acc[2]);
					}else if(no_cali){
						float acc[3];
						sensor_acc_measure(acc);
						Console.print("acc:%f %f %f\n", acc[0], acc[1], acc[2]);
					}else{
						float acc[3];
						/* read from topics instead of remeasuring */
						mcn_copy_from_hub(MCN_ID(SENSOR_ACC), acc);
						//sensor_acc_get_calibrated_data(acc);
						Console.print("cali acc:%f %f %f\n", acc[0], acc[1], acc[2]);
					}
					if(cnt > 1)
						rt_thread_delay(interval);
				}
			}break;
			case 2:	//mag
			{
				for(uint32_t i = 0 ; i < cnt ; i++){
					if(raw_data){
						int16_t raw_mag[3];
						sensor_mag_raw_measure(raw_mag);
						Console.print("raw mag:%d %d %d\n", raw_mag[0], raw_mag[1], raw_mag[2]);
					}else if(no_cali){
						float mag[3];
						sensor_mag_measure(mag);
						Console.print("mag:%f %f %f\n", mag[0], mag[1], mag[2]);
					}else{
						float mag[3];
						/* read from topics instead of remeasuring */
						mcn_copy_from_hub(MCN_ID(SENSOR_MAG), mag);
						Console.print("cali mag:%f %f %f\n", mag[0], mag[1], mag[2]);
					}
					if(cnt > 1)
						rt_thread_delay(interval);
				}			
			}break;
			case 3:	//gyr
			{
				for(uint32_t i = 0 ; i < cnt ; i++){
					if(raw_data){
						int16_t raw_gyr[3];
						sensor_gyr_raw_measure(raw_gyr);
						Console.print("raw gyr:%d %d %d\n", raw_gyr[0], raw_gyr[1], raw_gyr[2]);
					}else if(no_cali){
						float gyr[3];
						sensor_gyr_measure(gyr);
						Console.print("gyr:%f %f %f\n", gyr[0], gyr[1], gyr[2]);
					}else{
						float gyr[3];
						//sensor_gyr_get_calibrated_data(gyr);
						mcn_copy_from_hub(MCN_ID(SENSOR_GYR), gyr);
						Console.print("cali gyr:%f %f %f\n", gyr[0], gyr[1], gyr[2]);
					}
					if(cnt > 1)
						rt_thread_delay(interval);
				}	
			}break;
			case 4:	//gps
			{
					if(argc > 2){
						if(strcmp(argv[2], "sethome") == 0){
							ctrl_set_home();
							Console.print("set home success!\n");
						}else{
							for(uint32_t i = 0 ; i < cnt ; i++){
								struct vehicle_gps_position_s gps_report = gps_get_report();
								Console.print("sv:%d lat:%f lon:%f vn:%f ve:%f eph:%f hdop:%f\n", gps_report.satellites_used, (float)gps_report.lat*1e-7, (float)gps_report.lon*1e-7,
									gps_report.vel_n_m_s, gps_report.vel_e_m_s, gps_report.eph, gps_report.hdop);
							}
						}
					}
					if(cnt > 1)
						rt_thread_delay(interval);
			}break;
			default:
				break;
		}
	}
	
	return 0;
}
