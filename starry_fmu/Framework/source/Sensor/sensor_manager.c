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
#include "systime.h"
#include "lidar.h"
#include "ap_math.h"
#include "hil_interface.h"
#include "msh_usr_cmd.h"
#include "calibration.h"

#define BARO_UPDATE_INTERVAL    10

#define EARTH_RADIUS			6371000

static char* TAG = "Sensor";

static uint32_t gyr_read_time_stamp = 0;
static uint32_t acc_read_time_stamp = 0;
static uint32_t mag_read_time_stamp = 0;
static uint32_t _baro_update_time_stamp = 0;

static rt_device_t acc_device_t;
static rt_device_t mag_device_t;
static rt_device_t gyr_device_t;
static rt_device_t baro_device_t;
static rt_device_t gps_device_t;

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

MCN_DEFINE(IMU1, sizeof(SensorIMU));
MCN_DEFINE(MAG, sizeof(SensorMag));
MCN_DEFINE(BARO, sizeof(SensorBaro));
MCN_DEFINE(GPS_uBlox, sizeof(struct vehicle_gps_position_s));

MCN_DECLARE(GPS_POSITION);

/**************************	ACC API	**************************/
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

	float ofs[3] = {PARAM_GET_FLOAT(CALIBRATION, ACC_BIAS_X),
	                PARAM_GET_FLOAT(CALIBRATION, ACC_BIAS_Y),
	                PARAM_GET_FLOAT(CALIBRATION, ACC_BIAS_Z)
	               };
	float transM[3][3] = {
		{
			PARAM_GET_FLOAT(CALIBRATION, ACC_ROT_MAT_1), PARAM_GET_FLOAT(CALIBRATION, ACC_ROT_MAT_2),
			PARAM_GET_FLOAT(CALIBRATION, ACC_ROT_MAT_3)
		},
		{
			PARAM_GET_FLOAT(CALIBRATION, ACC_ROT_MAT_4), PARAM_GET_FLOAT(CALIBRATION, ACC_ROT_MAT_5),
			PARAM_GET_FLOAT(CALIBRATION, ACC_ROT_MAT_6)
		},
		{
			PARAM_GET_FLOAT(CALIBRATION, ACC_ROT_MAT_7), PARAM_GET_FLOAT(CALIBRATION, ACC_ROT_MAT_8),
			PARAM_GET_FLOAT(CALIBRATION, ACC_ROT_MAT_9)
		},
	};

	float ofs_acc[3];

	for(uint8_t i = 0 ; i < 3 ; i++) {
		ofs_acc[i] = acc_f[i] - ofs[i];
	}

	for(uint8_t i = 0 ; i < 3 ; i++) {
		acc[i] = ofs_acc[0] * transM[0][i] + ofs_acc[1] * transM[1][i] + ofs_acc[2] * transM[2][i];
	}

	return res;
}

/**************************	MAG API	**************************/
bool sensor_mag_ready(void)
{
	uint32_t time_now = time_nowMs();

	if((time_now - mag_read_time_stamp) >= 10) {
		return true;
	} else {
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
	float ofs[3] = {PARAM_GET_FLOAT(CALIBRATION, MAG_BIAS_X),
	                PARAM_GET_FLOAT(CALIBRATION, MAG_BIAS_Y),
	                PARAM_GET_FLOAT(CALIBRATION, MAG_BIAS_Z)
	               };
	float transM[3][3] = {
		{
			PARAM_GET_FLOAT(CALIBRATION, MAG_ROT_MAT_1), PARAM_GET_FLOAT(CALIBRATION, MAG_ROT_MAT_2),
			PARAM_GET_FLOAT(CALIBRATION, MAG_ROT_MAT_3)
		},
		{
			PARAM_GET_FLOAT(CALIBRATION, MAG_ROT_MAT_4), PARAM_GET_FLOAT(CALIBRATION, MAG_ROT_MAT_5),
			PARAM_GET_FLOAT(CALIBRATION, MAG_ROT_MAT_6)
		},
		{
			PARAM_GET_FLOAT(CALIBRATION, MAG_ROT_MAT_7), PARAM_GET_FLOAT(CALIBRATION, MAG_ROT_MAT_8),
			PARAM_GET_FLOAT(CALIBRATION, MAG_ROT_MAT_9)
		},
	};
#endif


	float ofs_mag[3];

	for(uint8_t i = 0 ; i < 3 ; i++) {
		ofs_mag[i] = mag_f[i] - ofs[i];
	}

	for(uint8_t i = 0 ; i < 3 ; i++) {
		mag[i] = ofs_mag[0] * transM[0][i] + ofs_mag[1] * transM[1][i] + ofs_mag[2] * transM[2][i];
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

	if(gyr_read_time_stamp - time_now >= 2) {
		return true;
	} else {
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

	float gyr_offset[3] = {PARAM_GET_FLOAT(CALIBRATION, GYR_BIAS_X),
	                       PARAM_GET_FLOAT(CALIBRATION, GYR_BIAS_Y),
	                       PARAM_GET_FLOAT(CALIBRATION, GYR_BIAS_Z)
	                      };
	float gyr_gain[3] = {PARAM_GET_FLOAT(CALIBRATION, GYR_X_GAIN),
	                     PARAM_GET_FLOAT(CALIBRATION, GYR_Y_GAIN),
	                     PARAM_GET_FLOAT(CALIBRATION, GYR_Z_GAIN)
	                    };

	res = sensor_gyr_measure(gyr_dps);

	// publish non-calibrated data for calibration
	mcn_publish(MCN_ID(SENSOR_MEASURE_GYR), gyr_dps);

	for(uint8_t i = 0 ; i < 3 ; i++) {
		gyr[i] = (gyr_dps[i] + gyr_offset[i]) * gyr_gain[i];
	}

	return res;
}

uint8_t sensor_get_device_id(char* device_name)
{
	uint8_t device_id = 0xFF;	//unknown device

	if(strcmp(device_name, ACC_DEVICE_NAME) == 0) {
		rt_device_control(acc_device_t, SENSOR_GET_DEVICE_ID, (void*)&device_id);
	} else if(strcmp(device_name, MAG_DEVICE_NAME) == 0) {
		rt_device_control(mag_device_t, SENSOR_GET_DEVICE_ID, (void*)&device_id);
	} else if(strcmp(device_name, GYR_DEVICE_NAME) == 0) {
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
	if(rt_device_control(baro_device_t, SENSOR_IS_CONV_FIN, RT_NULL) == RT_EOK) {
		return RT_TRUE;
	} else {
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

	switch((uint8_t)baro_state) {
		case S_CONV_1: {
			err = _baro_trig_conversion(1);

			if(err == RT_EOK)
				baro_state = S_CONV_2;
		}
		break;

		case S_CONV_2: {
			//9.04ms for OSR=4096, 4.54ms for OSR=2048
			err = _baro_read_raw_press();

			if(err == RT_EOK) {
				/* directly start D2 conversion */
				err = _baro_trig_conversion(2);

				if(err == RT_EOK)
					baro_state = S_COLLECT_REPORT;
				else
					baro_state = S_CONV_1;
			} else
				baro_state = S_CONV_1;	//if err, restart
		}
		break;

		case S_COLLECT_REPORT: {
			baro_state = S_CONV_1;
			err = _baro_read_raw_temp();

			if(err == RT_EOK) {
				if(rt_device_read(baro_device_t, COLLECT_DATA_POS, (void*)&report_baro, 1)) {
					/* start D1 conversion */
					if(_baro_trig_conversion(1) == RT_EOK)
						baro_state = S_CONV_2;
				} else {
					err = RT_ERROR;
				}
			}
		}
		break;
	}

	return err;
}

bool sensor_baro_ready(void)
{
	uint32_t time_now = time_nowMs();

	if((time_now - _baro_update_time_stamp) >= 10) {
		_baro_update_time_stamp = time_now;
		return true;
	} else {
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

	if(sensor_baro_get_state() == S_COLLECT_REPORT) {
		res = sensor_process_baro_state_machine();

		//get report;
		if(res == RT_EOK) {
			_baro_update_flag = true;
			return true;
		}
	} else {
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

	if(fabs(c) > 0) {
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
//	HOME_Pos home = pos_home_get();
//	if(home.gps_coordinate_set == false){
//		// gps home have not set yet
//		return -1;
//	}

//	//gps_calc_geometry_distance2(gps_pos, home.lat, home.lon, (double)gps_report.lat*1e-7, (double)gps_report.lon*1e-7);
//	gps_calc_geometry_distance(gps_pos, home.lat, home.lon, (double)gps_report.lat*1e-7, (double)gps_report.lon*1e-7);
//	gps_pos->z = (float)gps_report.alt*1e-3;
//
//	return 0;
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
rt_err_t sensor_manager_init(void)
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
	res |= rt_gps_init("uart4", &gps_position, &satellite_info);

	/* init acc device */
	acc_device_t = rt_device_find(ACC_DEVICE_NAME);

	if(acc_device_t == RT_NULL) {
		Console.e(TAG, "can't find acc device\r\n");
		return RT_EEMPTY;
	}

	rt_device_open(acc_device_t, RT_DEVICE_OFLAG_RDWR);

	/* init mag device */
	mag_device_t = rt_device_find(MAG_DEVICE_NAME);

	if(mag_device_t == RT_NULL) {
		Console.e(TAG, "can't find mag device\r\n");
		return RT_EEMPTY;
	} else {
		rt_device_open(mag_device_t, RT_DEVICE_OFLAG_RDWR);
	}

	/* init gyr device */
	gyr_device_t = rt_device_find(GYR_DEVICE_NAME);

	if(gyr_device_t == RT_NULL) {
		Console.e(TAG, "can't find gyr device\r\n");
		return RT_EEMPTY;
	}

	rt_device_open(gyr_device_t, RT_DEVICE_OFLAG_RDWR);

	/* init barometer device */
	baro_state = S_CONV_1;
	baro_device_t = rt_device_find(BARO_DEVICE_NAME);

	if(baro_device_t == RT_NULL) {
		Console.e(TAG, "can't find baro device\r\n");
		return RT_EEMPTY;
	}

	rt_device_open(baro_device_t, RT_DEVICE_OFLAG_RDWR);

	/* init gps device */
	gps_device_t = rt_device_find(GPS_DEVICE_NAME);

	if(gps_device_t == RT_NULL) {
		Console.e(TAG, "can't find gps device\r\n");
		return RT_EEMPTY;
	}

	rt_err_t gps_open_res = rt_device_open(gps_device_t, RT_DEVICE_OFLAG_RDWR);
	_gps_connected = gps_open_res == RT_EOK ? true : false;

#ifdef USE_LIDAR_I2C
	/* init lidar lite device */
	rt_lidar_init("i2c1");
	lidar_device_t = rt_device_find(LIDAR_DEVICE_NAME);

	if(lidar_device_t == RT_NULL) {
		Console.e(TAG, "can't find %s device\r\n", LIDAR_DEVICE_NAME);
		return RT_EEMPTY;
	}

	rt_device_open(lidar_device_t, RT_DEVICE_OFLAG_RDWR);
#endif

	float null_data[3] = {0, 0, 0};
	/* advertise sensor data */
	int mcn_res;
	mcn_res = mcn_advertise(MCN_ID(SENSOR_MEASURE_GYR));

	if(mcn_res != 0) {
		Console.e(TAG, "err:%d, SENSOR_MEASURE_GYR advertise fail!\n", mcn_res);
	}

	mcn_res = mcn_advertise(MCN_ID(SENSOR_MEASURE_ACC));

	if(mcn_res != 0) {
		Console.e(TAG, "err:%d, SENSOR_MEASURE_ACC advertise fail!\n", mcn_res);
	}

	mcn_res = mcn_advertise(MCN_ID(SENSOR_MEASURE_MAG));

	if(mcn_res != 0) {
		Console.e(TAG, "err:%d, SENSOR_MEASURE_MAG advertise fail!\n", mcn_res);
	}

	mcn_res = mcn_advertise(MCN_ID(SENSOR_GYR));

	if(mcn_res != 0) {
		Console.e(TAG, "err:%d, sensor_gyr advertise fail!\n", mcn_res);
	}

	mcn_res = mcn_advertise(MCN_ID(SENSOR_ACC));

	if(mcn_res != 0) {
		Console.e(TAG, "err:%d, sensor_acc advertise fail!\n", mcn_res);
	}

	mcn_res = mcn_advertise(MCN_ID(SENSOR_MAG));

	if(mcn_res != 0) {
		Console.e(TAG, "err:%d, sensor_mag advertise fail!\n", mcn_res);
	}

	mcn_res = mcn_advertise(MCN_ID(SENSOR_FILTER_GYR));

	if(mcn_res != 0) {
		Console.e(TAG, "err:%d, sensor_filter_gyr advertise fail!\n", mcn_res);
	}

	mcn_publish(MCN_ID(SENSOR_FILTER_GYR), &null_data);
	mcn_res = mcn_advertise(MCN_ID(SENSOR_FILTER_ACC));

	if(mcn_res != 0) {
		Console.e(TAG, "err:%d, sensor_filter_acc advertise fail!\n", mcn_res);
	}

	mcn_publish(MCN_ID(SENSOR_FILTER_ACC), &null_data);
	mcn_res = mcn_advertise(MCN_ID(SENSOR_FILTER_MAG));

	if(mcn_res != 0) {
		Console.e(TAG, "err:%d, sensor_filter_mag advertise fail!\n", mcn_res);
	}

	mcn_publish(MCN_ID(SENSOR_FILTER_MAG), &null_data);
	mcn_res = mcn_advertise(MCN_ID(SENSOR_BARO));

	if(mcn_res != 0) {
		Console.e(TAG, "err:%d, sensor_baro advertise fail!\n", mcn_res);
	}

	mcn_res = mcn_advertise(MCN_ID(SENSOR_LIDAR));

	if(mcn_res != 0) {
		Console.e(TAG, "err:%d, sensor_lidar advertise fail!\n", mcn_res);
	}

	mcn_res = mcn_advertise(MCN_ID(CORRECT_LIDAR));

	if(mcn_res != 0) {
		Console.e(TAG, "err:%d, correct_lidar advertise fail!\n", mcn_res);
	}

	mcn_res = mcn_advertise(MCN_ID(BARO_POSITION));

	if(mcn_res != 0) {
		Console.e(TAG, "err:%d, baro_position advertise fail!\n", mcn_res);
	}

	mcn_res = mcn_advertise(MCN_ID(GPS_STATUS));

	if(mcn_res != 0) {
		Console.e(TAG, "err:%d, GPS_STATUS advertise fail!\n", mcn_res);
	}

	mcn_res = mcn_advertise(MCN_ID(IMU1));

	if(mcn_res != 0) {
		Console.e(TAG, "err:%d, IMU1 advertise fail!\n", mcn_res);
	}

	mcn_res = mcn_advertise(MCN_ID(MAG));

	if(mcn_res != 0) {
		Console.e(TAG, "err:%d, MAG advertise fail!\n", mcn_res);
	}

	mcn_res = mcn_advertise(MCN_ID(BARO));

	if(mcn_res != 0) {
		Console.e(TAG, "err:%d, BARO advertise fail!\n", mcn_res);
	}

	mcn_res = mcn_advertise(MCN_ID(GPS_uBlox));

	if(mcn_res != 0) {
		Console.e(TAG, "err:%d, GPS_uBlox advertise fail!\n", mcn_res);
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
	SensorIMU imu1;
	SensorMag mag_field;
	SensorBaro barometer;

	if(sensor_gyr_measure(imu1.gyr_dps) == RT_EOK && sensor_acc_measure(imu1.acc_mps2) == RT_EOK) {
		imu1.timestamp_ms = time_nowMs();
		mcn_publish(MCN_ID(IMU1), &imu1);
	} else {
		Console.e(TAG, "fail to get imu data\n");
	}

	if(sensor_mag_ready()) {
		if(sensor_mag_measure(mag_field.mag_ga) == RT_EOK) {
			mag_field.timestamp_ms = time_nowMs();
			mcn_publish(MCN_ID(MAG), &mag_field);
		} else {
			Console.e(TAG, "fail to get mag data\n");
		}
	}

	if(sensor_baro_ready()) {
		if(sensor_baro_update()) {
			MS5611_REPORT_Def* baro_report = sensor_baro_get_report();

			barometer.pressure_Pa = baro_report->pressure;
			barometer.temperature_deg = baro_report->temperature;
			barometer.timestamp_ms = baro_report->time_stamp;
			mcn_publish(MCN_ID(BARO), &barometer);
		}
	}

	if(mcn_poll(gps_node_t)) {

		struct vehicle_gps_position_s gps_pos_t;
		mcn_copy(MCN_ID(GPS_POSITION), gps_node_t, &gps_pos_t);


	}
}

int handle_gps_shell_cmd(int argc, char** argv)
{
	if(argc > 1) {
		if(strcmp(argv[1], "status") == 0) {
			char status_str[20] = "";
			GPS_Status gps_status;
			mcn_copy_from_hub(MCN_ID(GPS_STATUS), &gps_status);

			if(gps_status.status == GPS_UNDETECTED) {
				strcpy(status_str, "UNDETECTED");
			} else if(gps_status.status == GPS_AVAILABLE) {
				strcpy(status_str, "AVAILABLE");
			} else {
				strcpy(status_str, "INAVAILABLE");
			}

			struct vehicle_gps_position_s gps_pos = gps_get_report();

			Console.print("gps status: %s, satelites:%d, fix type:%d [eph,epv]:[%.3f %.3f], [hdop,vdop]:[%.3f %.3f]\n", status_str, gps_pos.satellites_used,
			              gps_pos.fix_type, gps_pos.eph, gps_pos.epv, gps_pos.hdop, gps_pos.vdop);
		}
	}

	return 0;
}

int handle_sensor_shell_cmd(int argc, char** argv, int optc, sh_optv* optv)
{
	uint8_t sensor_type = 0;
	uint32_t period = 1000;	//default is 1s
	uint32_t read_num = 1;
	uint8_t data_type = 1;	// 0: raw data, 1: scaled data

	if(argc > 1) {
		if(strcmp(argv[1], "acc") == 0) {
			sensor_type = 1;
		} else if(strcmp(argv[1], "mag") == 0) {
			sensor_type = 2;
		} else if(strcmp(argv[1], "gyr") == 0) {
			sensor_type = 3;
		} else if(strcmp(argv[1], "baro") == 0) {
			sensor_type = 4;
		} else if(strcmp(argv[1], "gps") == 0) {
			sensor_type = 5;
		} else {
			Console.print("unknow parameter:%s\n", argv[1]);
			return 1;
		}

		// handle option
		for(uint16_t i = 0 ; i < optc ; i++) {

			if(strcmp(optv[i].opt, "--type") == 0 || strcmp(optv[i].opt, "-t") == 0) {

				if(optv[i].val == NULL)
					continue;

				if(strcmp(optv[i].val, "raw") == 0) {
					data_type = 0;
				} else {
					data_type = 1;	// default read scaled data
				}
			}

			if(strcmp(optv[i].opt, "--period") == 0 || strcmp(optv[i].opt, "-p") == 0) {

				if(!shell_is_number(optv[i].val))
					continue;

				period = atoi(optv[i].val);
			}

			if(strcmp(optv[i].opt, "--num") == 0 || strcmp(optv[i].opt, "-n") == 0) {

				if(!shell_is_number(optv[i].val))
					continue;

				read_num = atoi(optv[i].val);
			}
		}

		switch(sensor_type) {
			case 1: {	//acc
				for(uint32_t i = 0 ; i < read_num ; i++) {

					// TODO, read data from uMCN topic instead of driver
					if(data_type == 0) {
						int16_t raw_acc[3];
						sensor_acc_raw_measure(raw_acc);
						Console.print("%d %d %d\n", raw_acc[0], raw_acc[1], raw_acc[2]);
					} else if(data_type == 1) {
						float acc[3];
						sensor_acc_measure(acc);
						Console.print("%f %f %f\n", acc[0], acc[1], acc[2]);
					} else {
						Console.print("unknown data type\n");
						break;
					}

					if(read_num > 1)
						rt_thread_delay(period);
				}
			}
			break;

			case 2: {	//mag
				for(uint32_t i = 0 ; i < read_num ; i++) {
					if(data_type == 0) {
						int16_t raw_mag[3];
						sensor_mag_raw_measure(raw_mag);
						Console.print("%d %d %d\n", raw_mag[0], raw_mag[1], raw_mag[2]);
					} else if(data_type == 1) {
						float mag[3];
						sensor_mag_measure(mag);
						Console.print("%f %f %f\n", mag[0], mag[1], mag[2]);
					} else {
						Console.print("unknown data type\n");
						break;
					}

					if(read_num > 1)
						rt_thread_delay(period);
				}
			}
			break;

			case 3: {	//gyr
				for(uint32_t i = 0 ; i < read_num ; i++) {
					if(data_type == 0) {
						int16_t raw_gyr[3];
						sensor_gyr_raw_measure(raw_gyr);
						Console.print("%d %d %d\n", raw_gyr[0], raw_gyr[1], raw_gyr[2]);
					} else if(data_type == 1) {
						float gyr[3];
						sensor_gyr_measure(gyr);
						Console.print("%f %f %f\n", gyr[0], gyr[1], gyr[2]);
					} else {
						Console.print("unknown data type\n");
						break;
					}

					if(read_num > 1)
						rt_thread_delay(period);
				}
			}
			break;

			case 4: {	//baro

			} break;

			case 5: {	//gps

			} break;

			default: {
				Console.print("unknown sensor type:%d\n", sensor_type);
			}
			break;
		}
	}

	return 0;
}
