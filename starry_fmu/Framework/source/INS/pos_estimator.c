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
#include "FIR.h"
#include "ms5611.h"
#include "pos_estimator.h"
#include "kalman.h"
#include "ekf.h"
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

#define EVENT_POS_UPDATE        (1<<0)
#define EVENT_ACC_UPDATE		(1<<1)

#define EVENT_SET_HOME        (1<<0)

//#define POS_UPDATE_INTERVAL     50
#define POS_UPDATE_INTERVAL     25
#define BARO_UPDATE_INTERVAL    10

#define TO_DEGREE(a)	((float)a*1e-7)
//#define Deg2Rad(a)		(a*PI/180.0f)

const float EARTH_RADIUS = 6371393;	/* average earth radius, meter */
//const float PI = 3.1415926536;

static struct rt_timer timer_pos;
static struct rt_timer timer_baro;

static struct rt_event event_position;
static struct rt_event event_sethome;

static rt_device_t baro_device_t1, gps_device_t1;
static struct vehicle_gps_position_s gps_position;
static struct satellite_info_s satellite_info;

//static MS5611_REPORT_Def _baro_report;

static HOME_Pos home_pos;
kalman2_state state_x;
kalman2_state state_y;
kalman2_state state_z;
static uint8_t home_flag = 0;
static uint8_t init_flag = 0;
//static float dt = POS_UPDATE_INTERVAL * 0.001f;
static float dt = 0.02f;
static float cur_alt = 0.0f;
static bool _baroReportReceive = false;
static AltInfo _altInfo;
static bool _altInfo_ready = false;
static bool _ekf2_set_init_state = false;
static uint32_t _pos_est_time_stamp;
static float pre_lidat_alt = 0.0f;
static float saltation = 0.0f;

static float alt_throttle = 0.0f;

float pre_alt;
float calc_v;

static EKF_Def ekf;

Position_Info pos_info;

static char *TAG = "POS";

MCN_DEFINE(ALT_INFO, sizeof(AltInfo));

static void timer_pos_update(void* parameter)
{
	/* send acc update event */
	//rt_event_send(&event_position, EVENT_POS_UPDATE);
}

static void timer_baro_update(void* parameter)
{
//	rt_err_t res;
//	
//	if(sensor_baro_get_state() == S_COLLECT_REPORT){
//		res = sensor_process_baro_state_machine();
//		//get report;
//		if(res == RT_EOK){
//			OS_ENTER_CRITICAL;
//			baro_report = *sensor_baro_get_report();
//			OS_EXIT_CRITICAL;
//			_baroReportReceive = true;
//			rt_event_send(&event_position, EVENT_POS_UPDATE);
//		}
//	}else{
//		res = sensor_process_baro_state_machine();
//	}
}

bool _is_baro_report_available(void)
{
	return _baroReportReceive;
}

MS5611_REPORT_Def _get_baro_report(void)
{
	MS5611_REPORT_Def tempBaroReport;
	
	OS_ENTER_CRITICAL;
	//tempBaroReport = baro_report;
	OS_EXIT_CRITICAL;
	
	return tempBaroReport;
}

float _pos_get_baro_alt(void)
{
//	float alt;
//	
//	rt_enter_critical();
//	alt = baro_report.altitude;
//	rt_exit_critical();
//	
//	return alt;
}

float pos_get_alt_throttle(void)
{
	return alt_throttle;
}

uint8_t setInitialState(void)
{
	if(!home_flag)
		return 0;
	
//	float cov_ax = 0.002633;
//	//float cov_vx = 0.108990;
//	float cov_vx = 0.1;
//	//float cov_sx = 1364.084034;
//	float cov_sx = 0.1;
	//float cov_ax = 0.529286;
	float cov_ax = 0.1f;
	//float cov_vx = 0.097278;
	float cov_vx = 0.1f;
	//float cov_sy = 6171.600817;
	float cov_sx = 1;
	
	//float cov_ay = 0.529286;
	float cov_ay = 0.1f;
	//float cov_vy = 0.097278;
	float cov_vy = 0.1f;
	//float cov_sy = 6171.600817;
	float cov_sy = 1;
	
	//float cov_az = 0.019341;
	//float cov_az = 0.019341;
	//float cov_az = 0.09;
	float cov_az = 1;
	//float cov_vz = 0.046193;
	float cov_vz = 1;
	//float cov_sz = 0.033301;
	float cov_sz = 1;
	
	/* set initial state to home */
	state_x.x[0] = (float)home_pos.lat;
	state_x.x[1] = 0.0f;
	state_y.x[0] = (float)home_pos.lon;
	//state_y.x[0] = (float)home_pos.lat;
	state_y.x[1] = 0.0f;
	state_z.x[0] = (float)home_pos.alt;
	state_z.x[1] = 0.0f;

	/* Tx is the operator which transfer Delta(x)(meter) to Delta(lat)(1e7 degree) */
	/* Tx = 180/(PI*R), where R is the average radius of earth */
	float Tx = 180.0f/(PI*EARTH_RADIUS)*1e7;
	/* Ax = [ 1  dt*Tx ] */
	/*	    [ 0     1  ] */
	state_x.A[0][0] = 1.0f;
	state_x.A[0][1] = dt*Tx;
	state_x.A[1][0] = 0.0f;
	state_x.A[1][1] = 1.0f;		
	/* Ty is the operator which transfer Delta(y)(meter) to Delta(lon)(1e7 degree) */
	/* Ty = 180/(PI*r), r = R*cos(sita), sita = lat*PI/180, where R is the average radius of earth */
	float sita = Deg2Rad(90.0f-state_x.x[0]*1e-7);
	float r = EARTH_RADIUS*arm_sin_f32(sita);
	float Ty = 180.0f/(PI*r)*1e7;
	
	printf("Tx:%f Ty:%f\n", Tx, Ty);
	
	/* Ay = [ 1  dt*Ty ] */
	/*	    [ 0     1  ] */
	state_y.A[0][0] = 1.0f;
	state_y.A[0][1] = dt*Ty;
	state_y.A[1][0] = 0.0f;
	state_y.A[1][1] = 1.0f;
	/* Az = [ 1  dt] */
	/*	    [ 0  1 ] */
	state_z.A[0][0] = 1.0f;
	state_z.A[0][1] = dt;
	state_z.A[1][0] = 0.0f;
	state_z.A[1][1] = 1.0f;
	
	/* Bx = [ Tx*dt^2/2  dt ]^T */
	state_x.B[0] = Tx*dt*dt/2;
	//state_x.B[0] = 0;
	state_x.B[1] = dt;	
	/* By = [ Ty*dt^2/2  dt ]^T */
	state_y.B[0] = Ty*dt*dt/2;
	//state_y.B[0] = 0;
	state_y.B[1] = dt;	
	/* Bz = [ dt^2/2  dt ]^T */
	state_z.B[0] = dt*dt/2;
	//state_z.B[0] = 0;
	state_z.B[1] = dt;
	/* H = I, don't need to initialize H, because we has ignored H */
	state_x.H[0][0] = state_y.H[0][0] = state_z.H[0][0] = 1.0f;
	state_x.H[0][1] = state_y.H[0][1] = state_z.H[0][1] = 0.0f;
	state_x.H[1][0] = state_y.H[1][0] = state_z.H[1][0] = 0.0f;
	state_x.H[1][1] = state_y.H[1][1] = state_z.H[1][1] = 1.0f;
	/* Q = cov(a)^2 * B * B^T */
	state_x.q[0][0] = Tx*Tx*cov_ax*cov_ax*dt*dt*dt*dt/4;
	state_x.q[0][1] = Tx*cov_ax*cov_ax*dt*dt*dt/2;
	state_x.q[1][0] = Tx*cov_ax*cov_ax*dt*dt*dt/2;
	state_x.q[1][1] = cov_ax*cov_ax*dt*dt;
	state_y.q[0][0] = Ty*Ty*cov_ay*cov_ay*dt*dt*dt*dt/4;
	state_y.q[0][1] = Ty*cov_ay*cov_ay*dt*dt*dt/2;
	state_y.q[1][0] = Ty*cov_ay*cov_ay*dt*dt*dt/2;
	state_y.q[1][1] = cov_ay*cov_ay*dt*dt;
	state_z.q[0][0] = cov_az*cov_az*dt*dt*dt*dt/4;
	state_z.q[0][1] = cov_az*cov_az*dt*dt*dt/2;
	state_z.q[1][0] = cov_az*cov_az*dt*dt*dt/2;
	state_z.q[1][1] = cov_az*cov_az*dt*dt;

//	state_x.q[0][0] = 0;
//	state_x.q[0][1] = 0;
//	state_x.q[1][0] = 0;
//	state_x.q[1][1] = cov_ax*cov_ax*dt*dt;
//	state_y.q[0][0] = 0;
//	state_y.q[0][1] = 0;
//	state_y.q[1][0] = 0;
//	state_y.q[1][1] = cov_ay*cov_ay*dt*dt;
//	state_z.q[0][0] = 0;
//	state_z.q[0][1] = 0;
//	state_z.q[1][0] = 0;
//	state_z.q[1][1] = cov_az*cov_az*dt*dt;
	printf("\nQx:\n");
	printf("%f\n", state_x.q[0][0]);
	printf("%f\n", state_x.q[0][1]);
	printf("%f\n", state_x.q[1][0]);
	printf("%f\n", state_x.q[1][1]);
	printf("Qy:\n");
	printf("%f\n", state_y.q[0][0]);
	printf("%f\n", state_y.q[0][1]);
	printf("%f\n", state_y.q[1][0]);
	printf("%f\n", state_y.q[1][1]);
	printf("Qz:\n");
	printf("%f\n", state_z.q[0][0]);
	printf("%f\n", state_z.q[0][1]);
	printf("%f\n", state_z.q[1][0]);
	printf("%f\n", state_z.q[1][1]);
	/* R = [ cov(s)^2        0    ] */
	/*     [    0        cov(v)^2 ] */
	state_x.r[0][0] = cov_sx*cov_sx;
	state_x.r[0][1] = 0.0f;
	state_x.r[1][0] = 0.0f;
	state_x.r[1][1] = cov_vx*cov_vx;
	state_y.r[0][0] = cov_sy*cov_sy;
	state_y.r[0][1] = 0.0f;
	state_y.r[1][0] = 0.0f;
	state_y.r[1][1] = cov_vy*cov_vy;
	state_z.r[0][0] = cov_sz*cov_sz;
	state_z.r[0][1] = 0.0f;
	state_z.r[1][0] = 0.0f;
	state_z.r[1][1] = cov_vz*cov_vz;
	printf("\nRx:\n");
	printf("%f\n", state_x.r[0][0]);
	printf("%f\n", state_x.r[0][1]);
	printf("%f\n", state_x.r[1][0]);
	printf("%f\n", state_x.r[1][1]);
	printf("Ry:\n");
	printf("%f\n", state_y.r[0][0]);
	printf("%f\n", state_y.r[0][1]);
	printf("%f\n", state_y.r[1][0]);
	printf("%f\n", state_y.r[1][1]);
	printf("Rz:\n");
	printf("%f\n", state_z.r[0][0]);
	printf("%f\n", state_z.r[0][1]);
	printf("%f\n", state_z.r[1][0]);
	printf("%f\n", state_z.r[1][1]);
	
	state_x.p[0][0] = state_x.r[0][0];
	state_x.p[0][1] = state_x.r[0][1];
	state_x.p[1][0] = state_x.r[1][0];
	state_x.p[1][1] = state_x.r[1][1];
	state_y.p[0][0] = state_y.r[0][0];
	state_y.p[0][1] = state_y.r[0][1];
	state_y.p[1][0] = state_y.r[1][0];
	state_y.p[1][1] = state_y.r[1][1];
	state_z.p[0][0] = state_z.r[0][0];
	state_z.p[0][1] = state_z.r[0][1];
	state_z.p[1][0] = state_z.r[1][0];
	state_z.p[1][1] = state_z.r[1][1];
	
	init_flag = 1;
	
	return 1;
}

void set_home_alt(float alt)
{
	home_pos.alt = alt;
}

void _set_home_alt(void)
{
	MS5611_REPORT_Def* baro_report = sensor_baro_get_report();
	sensor_baro_clear_update_flag();
	home_pos.alt = baro_report->altitude;
	
#ifdef USE_LIDAR
	home_pos.lidar_alt = lidar_lite_get_dis();	//lidar value
#endif
}

void set_home_cur_alt(void)
{
#ifdef USE_LIDAR
	home_pos.lidar_alt = ekf.x.element[0][0]+home_pos.lidar_alt;
#else
	home_pos.alt = ekf.x.element[0][0]+home_pos.alt;
#endif
}

float get_home_alt(void)
{
#ifdef USE_LIDAR
	return home_pos.lidar_alt;
#else
	return home_pos.alt;
#endif
}

uint8_t set_home(uint32_t lon, uint32_t lat, float alt)
{
	home_pos.lon = lon;
	home_pos.lat = lat;
	home_pos.alt = alt;
	
	home_flag = 1;
	
	//setInitialState();
	
	rt_event_send(&event_sethome, EVENT_SET_HOME);
	
	cur_alt = home_pos.alt;
	
	return home_flag;
}

void set_home_with_current_pos(void)
{
	uint32_t lon, lat;
	float alt;
	gps_position = get_gps_position();
	
//	lon = TO_DEGREE(gps_position.lon);
//	lat = TO_DEGREE(gps_position.lat);
	lon = gps_position.lon;
	lat = gps_position.lat;
	//alt = baro_report.altitude;
	
	Console.w(TAG, "set cur home with lon:%d lat:%d alt:%f\n", lon, lat, alt);
	
	set_home(lon, lat, alt);
}

void update_pos_info(Position_Info* p_i, int32_t lat, int32_t lon, int32_t alt, 
						int32_t relative_alt, int16_t vx, int16_t vy, int16_t vz)
{
	p_i->lat = lat;
	p_i->lon = lon;
	p_i->alt = alt;
	p_i->relative_alt = relative_alt;
	p_i->vx = vx;
	p_i->vy = vy;
	p_i->vz = vz;
}

bool alt_info_ready(void)
{
	return _altInfo_ready;
}

void alt_info_clear(void)
{
	_altInfo_ready = false;
}

void save_alt_info(float alt, float relative_alt, float vz, float az, float az_bias)
{
	_altInfo.alt = alt;
	_altInfo.relative_alt = relative_alt;
	_altInfo.vz = vz;
	_altInfo.az = az;
	_altInfo.az_bias = az_bias;
	_altInfo_ready = true;
}

AltInfo get_alt_info(void)
{
	return _altInfo;
}

Position_Info get_pos_info(void)
{
	Position_Info temp_info;
	
	OS_ENTER_CRITICAL;
	temp_info = pos_info; 
	OS_EXIT_CRITICAL;
	
	return temp_info;
}

float lidat_alt_saltation(float lidar_alt, float dT)
{	
//		float delta_alt = ekf.z.element[0][0]-pre_lidat_alt;
//		float derivative = delta_alt/ekf.T;
//		if(fabs(derivative) > 5){
//			alt_saltation += delta_alt;
//		}
//		//alt_saltation = alt_saltation + ()
//		
//		float cor_alt = ekf.z.element[0][0]-alt_saltation;
//		float baro_alt = baro_report->altitude - home_pos.alt;
//		float saltation_observe = ekf.z.element[0][0] - baro_alt;
//		alt_saltation = alt_saltation + (saltation_observe - alt_saltation) * 0.1f;
//		cor_alt = ekf.z.element[0][0]-alt_saltation;
//		//cor_alt = cor_alt + (baro_alt-cor_alt)*0.05f;
//		
//		Console.print("%.2f %.2f %.2f\n", ekf.z.element[0][0], cor_alt, baro_alt);
//		pre_lidat_alt = ekf.z.element[0][0];
	
	float delta_alt = lidar_alt-pre_lidat_alt;
	float derivative = delta_alt/dT;
	if(fabs(derivative) >= 5.0f){
		saltation += delta_alt;
	}
	pre_lidat_alt = lidar_alt;
	
	//Console.print("%.2f %.2f %.2f\n", lidar_alt, lidar_alt - saltation, derivative);
	
	return lidar_alt - saltation;
}

void pos_est_init(void)
{
	/* EKF2 update each 20ms */
	//EKF2_Init(&ekf, 0.02f);

	int mcn_res = mcn_advertise(MCN_ID(ALT_INFO));
	if(mcn_res != 0){
		Console.e(TAG, "ALT_INFO advertise err:%d\n", mcn_res);
	}

	_ekf2_set_init_state = false;
	_pos_est_time_stamp = 0;
}

void pos_est_reset(void)
{
	pre_lidat_alt = 0.0f;
	saltation = 0.0f;
	
	/* reset home position when vehicle unlock */
	set_home_cur_alt();
}


void pos_est_update(float dT)
{
	if(_ekf2_set_init_state){
		const float* acc;
		float accE[3];
		
		acc = accfilter_current();
		/* transfer acceleration from body grame to navigation frame */
		quaternion_rotateVector(attitude_est_get_quaternion(), acc, accE);	
		/* remove gravity */
		accE[2] += 9.8f;

		pre_alt = ekf.x.element[0][0];

#ifndef USE_LIDAR
		MS5611_REPORT_Def* baro_report = sensor_baro_get_report();
		sensor_baro_clear_update_flag();
#endif			
		
		ekf.u.element[0][0] = 0.0f;
		ekf.u.element[1][0] = accE[2];
		
		/* the observed altitude is relative altitude */
#ifdef USE_LIDAR
		ekf.z.element[0][0] = lidar_lite_get_dis() - get_home_alt();
		/* correct with lidat altitude saltation */
		//ekf.z.element[0][0] = lidat_alt_saltation(ekf.z.element[0][0], dT);
#else
		ekf.z.element[0][0] = baro_report->altitude - get_home_alt();
#endif
		ekf.z.element[1][0] = calc_v;
		
		//Console.print("%.3f %.3f\n", accE[2], ekf.z.element[0][0]);
		
		EKF2_Update(&ekf);

		//calc_v = calc_v*0.7f+(pre_alt-ekf.x.element[0][0])/ekf.T*0.3f;
		//calc_v = calc_v + ((pre_alt-ekf.x.element[0][0])/ekf.T-calc_v)*0.5569f;
		calc_v = (pre_alt-ekf.x.element[0][0])/ekf.T;
		
		/* save altitude information */
		//save_alt_info(ekf.x.element[0][0], ekf.x.element[0][0]-get_home_alt(), ekf.x.element[1][0], accE[2]);
		save_alt_info(ekf.x.element[0][0]+get_home_alt(), ekf.x.element[0][0], ekf.x.element[1][0], accE[2], EKF2_GetZ_Bias());
		/* publish altitude information */
		mcn_publish(MCN_ID(ALT_INFO), &_altInfo);
		
		//Console.print("%.3f %.3f\n", ekf.u.element[1][0], ekf.z.element[0][0]);
		//Console.print("%.2f %.2f %.2f\n", ekf.u.element[1][0], lidar_lite_get_dis() - get_home_alt(), baro_report->altitude - get_home_alt());
		//Console.print("%.3f %.3f %.3f %.3f\n", filter_acc, baro_report->altitude, ekf.x.element[0][0], ekf.x.element[1][0]);

//		static uint32_t time = 0;
//		Console.print_eachtime(&time, 500, "rel_h:%.3f v:%.3f home:%.3f h:%.3f\n", _altInfo.relative_alt, _altInfo.vz, get_home_alt(),
//				ekf.x.element[0][0]);
	}else{
		/* wait until baro report is available */
		if(sensor_baro_get_update_flag()){
			//EKF2_Init(&ekf, 0.1, 0.1, 0.05, 0.05, dT);
//			EKF2_Init(&ekf, 0.05, 0.05, 0.05, 0.05, dT);
			EKF2_Init(&ekf, 0.04, 0.04, 0.05, 0.05, dT);
			
			//MS5611_REPORT_Def* baro_report = sensor_baro_get_report();
			//sensor_baro_clear_update_flag();
			
			/* set initial state for EKF2 */
			ekf.x.element[0][0] = 0.0f;
			ekf.x.element[1][0] = 0.0f;
			ekf.last_x.element[0][0] = 0.0f;
			ekf.last_x.element[1][0] = 0.0f;
			/* set home to current altitude */
			_set_home_alt();
			
			pre_alt = ekf.x.element[0][0];
			calc_v = ekf.x.element[1][0];

			_ekf2_set_init_state = true;
		}	
	}
}
