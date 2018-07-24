/*
 * File      : att_estimator.c
 *
 * Change Logs:
 * Date           Author       Notes
 * 2016-07-01     zoujiachi    first version.
 */
 
#include <rthw.h>
#include <rtdevice.h>
#include <rtthread.h>
#include <math.h>
#include "quaternion.h"
#include "sensor_manager.h"
#include "AHRS.h"
#include "MARG_AHRS.h"
#include "filter.h"
#include "delay.h"
#include "console.h"
#include "att_estimator.h"
#include "control_main.h"
#include "uMCN.h"


#define ATT_EST_INTERVAL			4

MCN_DECLARE(SENSOR_ACC);
MCN_DECLARE(SENSOR_MAG);

MCN_DEFINE(ATT_EULER, sizeof(Euler));	
MCN_DEFINE(ATT_QUATERNION, sizeof(quaternion));

static char *TAG = "Att_Est";

static quaternion drone_attitude;

void attitude_est_run(float dT)
{	
	const float* gyr_t = gyrfilter_current();
	const float* acc_t = accfilter_current();
	const float* mag_t = magfilter_current();
	
#if   defined ( AHRS_USE_DEFAULT ) 
	AHRS_update(&drone_attitude, gyrfilter_current(), accfilter_current(), magfilter_current(), dT);
#elif defined ( AHRS_USE_MARG )
	MARG_AHRS_Update(&drone_attitude, gyr_t[0], gyr_t[1], gyr_t[2], -acc_t[0], -acc_t[1], -acc_t[2], mag_t[0], mag_t[1], mag_t[2], dT);
#elif defined ( AHRS_USE_MAHONY )
	MahonyAHRS_update(&drone_attitude, gyrfilter_current(), accfilter_current(), magfilter_current(), dT);
#else
	#error Please select AHRS method.
#endif
	
	mcn_publish(MCN_ID(ATT_QUATERNION), &drone_attitude);
	Euler euler = attitude_est_get_euler();
	mcn_publish(MCN_ID(ATT_EULER), &euler);
}

rt_err_t attitude_est_init(void)
{
	/* initialize attitude accordind to the acc and mag value */
#ifdef HIL_SIMULATION
	float acc[3] = {0.0, 0.0, -9.8};
	float mag[3] = {1.0, 0.0, 0.0};
#else
	float acc[3], mag[3];
	sensor_acc_get_calibrated_data(acc);
	sensor_mag_get_calibrated_data(mag);
#endif
	AHRS_reset(&drone_attitude, acc, mag);
	
	
	int mcn_res = mcn_advertise(MCN_ID(ATT_QUATERNION));
	if(mcn_res != 0){
		Console.e(TAG, "err:%d, ATT_QUATERNION advertise fail!\n", mcn_res);
	}
	mcn_res = mcn_advertise(MCN_ID(ATT_EULER));
	if(mcn_res != 0){
		Console.e(TAG, "err:%d, ATT_EULER advertise fail!\n", mcn_res);
	}

	/* delay 10ms to make sure mag data is ready while AHRS_calculating() is invoked */
	rt_thread_delay(10);

	return RT_EOK;
}

void attitude_est_reset(void)
{
	float acc[3];
	float mag[3];
	
	mcn_copy_from_hub(MCN_ID(SENSOR_ACC), acc);
	mcn_copy_from_hub(MCN_ID(SENSOR_MAG), mag);
	
	AHRS_reset(&drone_attitude, acc, mag);
}

quaternion attitude_est_get_quaternion(void)
{
	quaternion att;
	
	OS_ENTER_CRITICAL;
	att = drone_attitude;
	OS_EXIT_CRITICAL;
	
    return att;
}

Euler attitude_est_get_euler(void)
{
	quaternion att;
	Euler e;
	
	OS_ENTER_CRITICAL;
	att = drone_attitude;
	OS_EXIT_CRITICAL;
	
	quaternion_toEuler(att, &e);
	
    return e;
}

int handle_att_est_shell_cmd(int argc, char** argv)
{
	if(argc > 1){
		if(strcmp(argv[1], "show") == 0){
			Euler e = attitude_est_get_euler();
			Console.print("Euler: roll:%f pitch:%f yaw:%f	unit:deg\n", Rad2Deg(e.roll), Rad2Deg(e.pitch), Rad2Deg(e.yaw));
			quaternion q = attitude_est_get_quaternion();
			Console.print("Quaternion: w:%f x:%f y:%f z:%f\n", q.w, q.x, q.y, q.z);
		}
		if(strcmp(argv[1], "reset") == 0){
			attitude_est_reset();
			Console.print("reset attitude success\n");
		}
	}
	
	return 0;
}

