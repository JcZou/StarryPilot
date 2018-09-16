/*
 * File      : position.c
 *
 * Change Logs:
 * Date           Author       Notes
 * 2017-04-30     zoujiachi    first version.
 */
 
#include <rthw.h>
#include <rtdevice.h>
#include <rtthread.h>
#include <math.h>
#include "fir.h"
#include "ms5611.h"
#include "pos_estimator.h"
#include "kf.h"
#include "sensor_manager.h"
#include "console.h"
#include "gps.h"
#include "quaternion.h"
#include "att_estimator.h"
#include "global.h"
#include "delay.h"
#include "control_alt.h"
#include "filter.h"
#include "uMCN.h"
#include "fifo.h"
#include "declination.h"

#ifdef HIL_SIMULATION
	#define KF_GPS_POS_DELAY		0
	#define KF_BARO_POS_DELAY		0
	#define KF_GPS_VEL_DELAY		0
	#define KF_BARO_VEL_DELAY		0
#else
	#define KF_GPS_POS_DELAY		100
	#define KF_BARO_POS_DELAY		100
	#define KF_GPS_VEL_DELAY		120
	#define KF_BARO_VEL_DELAY		200
#endif

#define KF_MAX_DELAY_OFFFSET	20

static HOME_Pos _home_pos;
static McnNode_t alt_node_t;
static McnNode_t gps_node_t;
static KF_Def pos_kf[3];
static FIFO _hist_x[3][2];
static float _acc_bias[3] = {0,0,0};

static Altitude_Info _altInfo;

static char *TAG = "POS";

MCN_DEFINE(ALT_INFO, sizeof(Altitude_Info));
MCN_DEFINE(POS_INFO, sizeof(Position_Info));
MCN_DEFINE(POS_KF, sizeof(POS_KF_Log));
MCN_DEFINE(HOME_POS, sizeof(HOME_Pos));

MCN_DECLARE(BARO_POSITION);
MCN_DECLARE(GPS_POSITION);
MCN_DECLARE(GPS_STATUS);

static float q_x = 1.0f;
static float q_y = 1.0f;
static float q_z = 1.0f;
static float q_vx = 1.0f;
static float q_vy = 1.0f;
static float q_vz = 1.0f;

static float r_x = 0.25f;
static float r_y = 0.25f;
static float r_z = 1.0f;
static float r_vx = 0.4f;
static float r_vy = 0.4f;
static float r_vz = 2.5f;

void save_alt_info(float alt, float relative_alt, float vz, float az, float az_bias)
{
	_altInfo.alt = alt;
	_altInfo.relative_alt = relative_alt;
	_altInfo.vz = vz;
	_altInfo.az = az;
	_altInfo.az_bias = az_bias;
}

void pos_home_set(HOME_Item item, void* data)
{
	if(item == BARO_ALT){
		float alt = *(float*)data;
		_home_pos.alt = alt;
		_home_pos.baro_altitude_set = true;
	}
	if(item == LIDAR_ALT){
		float alt = *(float*)data;
		_home_pos.lidar_alt = alt;
		_home_pos.lidar_altitude_set = true;
	}
	if(item == GPS_COORDINATE){
		double *gps_coor = data;
		_home_pos.lat = gps_coor[0];
		_home_pos.lon = gps_coor[1];
		_home_pos.gps_coordinate_set = true;
		
		//calculate magnetic declination
		_home_pos.mag_decl = compass_get_declination(_home_pos.lat, _home_pos.lon);
	}
	
	mcn_publish(MCN_ID(HOME_POS), &_home_pos);
}

HOME_Pos pos_home_get(void)
{
	return _home_pos;
}

void pos_get_home(HOME_Pos* home_pos)
{
	mcn_copy_from_hub(MCN_ID(HOME_POS), home_pos);
}

void pos_try_sethome(void)
{
	GPS_Status gps_status;
	
	mcn_copy_from_hub(MCN_ID(GPS_STATUS), &gps_status);
	
	if(mcn_poll(alt_node_t) && _home_pos.baro_altitude_set==false){
		
		// TODO: check legality of altitude
		BaroPosition baro_pos;
		mcn_copy(MCN_ID(BARO_POSITION), alt_node_t, &baro_pos);
		
		pos_home_set(BARO_ALT, &baro_pos.altitude);
		Console.print("alt home set to:%f\n", baro_pos.altitude);
	}
	if(mcn_poll(gps_node_t) && _home_pos.gps_coordinate_set==false){
		
		// check if gps is available
		if(gps_status.status == GPS_AVAILABLE){
			
			struct vehicle_gps_position_s gps_pos = gps_get_report();
			double gps_coordinate[2] = {(double)gps_pos.lat*1e-7, (double)gps_pos.lon*1e-7};
			
			pos_home_set(GPS_COORDINATE, gps_coordinate);
			Console.print("gps home set to:%f %f\n", gps_coordinate[0], gps_coordinate[1]);
		}
	}
}

void pos_est_update(float dT)
{
	float accE[3];
	GPS_Status gps_status;
	
	mcn_copy_from_hub(MCN_ID(GPS_STATUS), &gps_status);
	
	if(mcn_poll(alt_node_t) && _home_pos.baro_altitude_set==false){
		
		// TODO: check legality of altitude
		BaroPosition baro_pos;
		mcn_copy(MCN_ID(BARO_POSITION), alt_node_t, &baro_pos);
		
		pos_home_set(BARO_ALT, &baro_pos.altitude);
		Console.print("alt home set to:%f\n", baro_pos.altitude);
	}
	if(mcn_poll(gps_node_t) && _home_pos.gps_coordinate_set==false){
		
		// check if gps is available
		if(gps_status.status == GPS_AVAILABLE){
			
			struct vehicle_gps_position_s gps_pos = gps_get_report();
			double gps_coordinate[2] = {(double)gps_pos.lat*1e-7, (double)gps_pos.lon*1e-7};
			
			pos_home_set(GPS_COORDINATE, gps_coordinate);
			Console.print("gps home set to:%f %f\n", gps_coordinate[0], gps_coordinate[1]);
		}
	}
	
	float acc[3];
	sensor_get_acc(acc);
	/* transfer acceleration from body frame to navigation frame */
	quaternion q = attitude_est_get_quaternion();
	quaternion_rotateVector(&q, acc, accE);	
	/* remove gravity */
	accE[2] += 9.8f;
	
	pos_kf[0].u.element[0][0] = accE[0] - _acc_bias[0];
	pos_kf[1].u.element[0][0] = accE[1] - _acc_bias[1];
	pos_kf[2].u.element[0][0] = accE[2] - _acc_bias[2];
	
	Vector3f_t pos = {0,0,0};
	Vector3f_t vel = {0,0,0};
	struct vehicle_gps_position_s gps_pos = gps_get_report();

	if(gps_status.status == GPS_AVAILABLE && _home_pos.gps_coordinate_set){
		
		gps_get_position(&pos, gps_pos);
		gps_get_velocity(&vel, gps_pos);
	}
	
	pos_kf[0].z.element[0][0] = pos.x;
	pos_kf[0].z.element[1][0] = vel.x;
	pos_kf[1].z.element[0][0] = pos.y;
	pos_kf[1].z.element[1][0] = vel.y;
	
#ifdef USE_LIDAR
	pos.z = lidar_lite_get_dis() - get_home_alt();
	vel.z = 0; //TODO
#else
	BaroPosition baro_pos;
	mcn_copy_from_hub(MCN_ID(BARO_POSITION), &baro_pos);
	if(_home_pos.baro_altitude_set){
		pos.z = baro_pos.altitude;
		vel.z = baro_pos.velocity;
	}
#endif
	pos_kf[2].z.element[0][0] = pos.z;
	pos_kf[2].z.element[1][0] = vel.z;
	
	/* predict process */
	KF_Predict(&pos_kf[0]);
	KF_Predict(&pos_kf[1]);
	KF_Predict(&pos_kf[2]);
	
	// store history state
	for(uint8_t i = 0 ; i < 3 ; i++){
		for(uint8_t j = 0 ; j < 2 ; j++){
			fifo_push(&_hist_x[i][j], pos_kf[i].x.element[j][0]);
		}
	}
	
	// calculate observer delay offset
	uint32_t now = time_nowMs();
	int interval = (int)(1e3f*dT);
	uint32_t gps_pos_hist_offset = (now - (gps_pos.timestamp_position-KF_GPS_POS_DELAY))/interval;
	uint32_t baro_pos_hist_offset = (now - (baro_pos.time_stamp-KF_BARO_POS_DELAY))/interval;
	uint32_t gps_vel_hist_offset = (now - (gps_pos.timestamp_velocity-KF_GPS_VEL_DELAY))/interval;
	uint32_t baro_vel_hist_offset = (now - (baro_pos.time_stamp-KF_BARO_VEL_DELAY))/interval;
	
	// constrain offset
	gps_pos_hist_offset = constrain_uint32(gps_pos_hist_offset, 0, KF_MAX_DELAY_OFFFSET);
	baro_pos_hist_offset = constrain_uint32(baro_pos_hist_offset, 0, KF_MAX_DELAY_OFFFSET);
	gps_vel_hist_offset = constrain_uint32(gps_vel_hist_offset, 0, KF_MAX_DELAY_OFFFSET);
	baro_vel_hist_offset = constrain_uint32(baro_vel_hist_offset, 0, KF_MAX_DELAY_OFFFSET);

	uint32_t hist_offset[3][2] = {
		{gps_pos_hist_offset, gps_vel_hist_offset},
		{gps_pos_hist_offset, gps_vel_hist_offset},
		{baro_pos_hist_offset, baro_vel_hist_offset}
	};
	
	float delta_x[3][2];
	for(uint8_t i = 0 ; i < 3 ; i++){
		
		// calculate delta state
		for(uint8_t j = 0 ; j < 2 ; j++){
			float hist_val = fifo_read_back(&_hist_x[i][j], hist_offset[i][j]);
			delta_x[i][j] = pos_kf[i].x.element[j][0] - hist_val;
			
			// set current state to history value
			pos_kf[i].x.element[j][0] = hist_val;
		}
		
		// calculate bias
		_acc_bias[i] += (pos_kf[i].x.element[1][0] - pos_kf[i].z.element[1][0])*dT*0.1;
		_acc_bias[i] = constrain_float(_acc_bias[i], -0.5f, 0.5f);
	}
	
	/* update process */
	KF_Update(&pos_kf[0]);
	KF_Update(&pos_kf[1]);
	KF_Update(&pos_kf[2]);
	
	for(uint8_t i = 0 ; i < 3 ; i++){
		// add delta state back
		for(uint8_t j = 0 ; j < 2 ; j++){
			pos_kf[i].x.element[j][0] += delta_x[i][j];
		}
	}

	/* save altitude information */
	//save_alt_info(ekf.x.element[0][0], ekf.x.element[0][0]-get_home_alt(), ekf.x.element[1][0], accE[2]);
	HOME_Pos home = pos_home_get();
	// change direction from down to up
	save_alt_info(-pos_kf[2].x.element[0][0], -(pos_kf[2].x.element[0][0]-home.alt), -pos_kf[2].x.element[1][0], -accE[2], -_acc_bias[2]);
	/* publish altitude information */
	mcn_publish(MCN_ID(ALT_INFO), &_altInfo);

	POS_KF_Log pos_kf_log;
	pos_kf_log.u_x = pos_kf[0].u.element[0][0];
	pos_kf_log.u_y = pos_kf[1].u.element[0][0];
	pos_kf_log.u_z = pos_kf[2].u.element[0][0];
	pos_kf_log.est_x = pos_kf[0].x.element[0][0];
	pos_kf_log.est_vx = pos_kf[0].x.element[1][0];
	pos_kf_log.est_y = pos_kf[1].x.element[0][0];
	pos_kf_log.est_vy = pos_kf[1].x.element[1][0];
	pos_kf_log.est_z = pos_kf[2].x.element[0][0];
	pos_kf_log.est_vz = pos_kf[2].x.element[1][0];
	pos_kf_log.obs_x = pos_kf[0].z.element[0][0];
	pos_kf_log.obs_vx = pos_kf[0].z.element[1][0];
	pos_kf_log.obs_y = pos_kf[1].z.element[0][0];
	pos_kf_log.obs_vy = pos_kf[1].z.element[1][0];
	pos_kf_log.obs_z = pos_kf[2].z.element[0][0];
	pos_kf_log.obs_vz = pos_kf[2].z.element[1][0];
	mcn_publish(MCN_ID(POS_KF), &pos_kf_log);
}

void pos_est_reset(void)
{	
	_home_pos.baro_altitude_set = false;
	_home_pos.lidar_altitude_set = false;
	_home_pos.gps_coordinate_set = false;
	_home_pos.mag_decl = 0.0f;
	
	/* reset home position when vehicle unlock */
	//set_home_cur_alt();
}

void pos_est_init(float dT)
{
	int mcn_res = mcn_advertise(MCN_ID(ALT_INFO));
	if(mcn_res != 0){
		Console.e(TAG, "ALT_INFO advertise err:%d\n", mcn_res);
	}
	mcn_res = mcn_advertise(MCN_ID(POS_INFO));
	if(mcn_res != 0){
		Console.e(TAG, "POS_INFO advertise err:%d\n", mcn_res);
	}
	mcn_res = mcn_advertise(MCN_ID(POS_KF));
	if(mcn_res != 0){
		Console.e(TAG, "POS_KF advertise err:%d\n", mcn_res);
	}
	mcn_res = mcn_advertise(MCN_ID(HOME_POS));
	if(mcn_res != 0){
		Console.e(TAG, "HOME_POS advertise err:%d\n", mcn_res);
	}
	HOME_Pos hp;
	hp.baro_altitude_set = 0;
	hp.gps_coordinate_set = 0;
	hp.lidar_altitude_set = 0;
	// publish to init HOME_Pos
	mcn_publish(MCN_ID(HOME_POS), &hp);
	
	alt_node_t = mcn_subscribe(MCN_ID(BARO_POSITION), NULL);
	if(alt_node_t == NULL)
		Console.e(TAG, "alt_node_t subscribe err\n");
	gps_node_t = mcn_subscribe(MCN_ID(GPS_POSITION), NULL);
	if(gps_node_t == NULL)
		Console.e(TAG, "gps_node_t subscribe err\n");
	
	// create kalman filter for position estimation
	KF_Create(&pos_kf[0], 2, 1);
	KF_Create(&pos_kf[1], 2, 1);
	KF_Create(&pos_kf[2], 2, 1);
	
	// initialize position kalman filter
	float F[] = 
	{
		1, dT,
		0,  1,
	};	
	float B[] = 
	{
		 0,
		dT,
	};
	float H[] = 
	{
		1, 0,
		0, 1,
	};
	float Q[] = 
	{
		1, 0,
		0, 1,
	};	
	float R[9*9] = 
	{
		1, 0,
		0, 1,
	};
	float *P = Q;
	float x_init[2] = {0, 0};
	
	Q[0] = (double)q_x*q_x*dT*dT;
	Q[3] = (double)q_vx*q_vx*dT*dT;
	R[0] = (double)r_x*r_x;
	R[3] = (double)r_vx*r_vx;
	KF_Init(&pos_kf[0], F, B, H, P, Q, R, x_init, true, dT);
	Q[0] = (double)q_y*q_y*dT*dT;
	Q[3] = (double)q_vy*q_vy*dT*dT;
	R[0] = (double)r_y*r_y;
	R[3] = (double)r_vy*r_vy;
	KF_Init(&pos_kf[1], F, B, H, P, Q, R, x_init, true, dT);
	Q[0] = (double)q_z*q_z*dT*dT;
	Q[3] = (double)q_vz*q_vz*dT*dT;
	R[0] = (double)r_z*r_z;
	R[3] = (double)r_vz*r_vz;
	KF_Init(&pos_kf[2], F, B, H, P, Q, R, x_init, true, dT);

	for(int i = 0 ; i < 3 ; i++){
		for(int j = 0 ; j < 2 ; j++){
			fifo_create(&_hist_x[i][j], KF_MAX_DELAY_OFFFSET+1);
			fifo_flush(&_hist_x[i][j]);
		}
	}

	_home_pos.baro_altitude_set = false;
	_home_pos.lidar_altitude_set = false;
	_home_pos.gps_coordinate_set = false;
	_home_pos.mag_decl = 0.0f;
}

int handle_pos_est_shell_cmd(int argc, char** argv)
{
	if(argc > 1){
		if(strcmp(argv[1], "state") == 0){
			POS_KF_Log pos_kf;
			mcn_copy_from_hub(MCN_ID(POS_KF), &pos_kf);
				
			Console.print("kf states, x:%f y:%f z:%f vx:%f vy:%f vz:%f\n", pos_kf.est_x, pos_kf.est_y, pos_kf.est_z,
				pos_kf.est_vx, pos_kf.est_vy, pos_kf.est_vz);
		}
	}
	
	return 0;
}
