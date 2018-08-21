/*
 * File      : control_alt.c
 *
 *
 * Change Logs:
 * Date           Author       	Notes
 * 2017-09-29     zoujiachi   	the first version
 */

#include <math.h>
#include "control_alt.h"
#include "param.h"
#include "rc.h"
#include "pwm.h"
#include "control_main.h"
#include "quaternion.h"
#include "att_estimator.h"
#include "pos_estimator.h"
#include "motor.h"
#include "global.h"
#include "console.h"
#include "filter.h"
#include "att_pid.h"
#include "starryio_manager.h"
#include "pid.h"
#include "kf.h"
#include "delay.h"

#define MAX_ALT_RATE		1.0f		/* in m/s */
#define MAX_ALTITUDE		0.4f
#define MIN_ALTITUDE		-2.0f
#define ALT_ERROR_BOUND		1.0f		/* in meter */
#define ALT_ACCUM_BOUND		0.5f		/* in m/s */
//#define RATE_ACCUM_BOUND	0.1f	
#define RATE_ACCUM_BOUND	0.4f

P_Controler alt_controller;
P_Controler vel_controller;
PID_Controler acc_controller;

void alt_controller_reset(void)
{
	p_controller_init(
		&alt_controller, 
		1,		//limit_err_flag
		0,	//feed_forward_flag
		0.01,	//dt
		PARAM_GET_FLOAT(ALT_CONTROLLER, ALT_ERR_MIN),	//min_err
		PARAM_GET_FLOAT(ALT_CONTROLLER, ALT_ERR_MAX),	//max_err
		PARAM_GET_FLOAT(ALT_CONTROLLER, ALT_P),	//kp
		PARAM_GET_FLOAT(ALT_CONTROLLER, ALT_OUTPUT_MIN),	//min_output
		PARAM_GET_FLOAT(ALT_CONTROLLER, ALT_OUTPUT_MAX));	//max_output

	p_controller_init(
		&vel_controller, 
		1,		//limit_err_flag
		PARAM_GET_INT32(ALT_CONTROLLER, FEEDFORWARD_EN),	//feed_forward_flag
		0.01,	//dt
		PARAM_GET_FLOAT(ALT_CONTROLLER, VEL_ERR_MIN),	//min_err
		PARAM_GET_FLOAT(ALT_CONTROLLER, VEL_ERR_MAX),	//max_err
		PARAM_GET_FLOAT(ALT_CONTROLLER, ALT_RATE_P),	//kp
		PARAM_GET_FLOAT(ALT_CONTROLLER, VEL_OUTPUT_MIN),	//min_output
		PARAM_GET_FLOAT(ALT_CONTROLLER, VEL_OUTPUT_MAX) );	//max_output

	pid_controller_init(  
		&acc_controller,
		1,	//limit_err_flag
		1,	//integrate_limit_flag
		0,	//integral_separation_flag
		0,	//feed_forward_flag
		0.01f,	//dt
		PARAM_GET_FLOAT(ALT_CONTROLLER, ACC_ERR_MIN),	//min_err
		PARAM_GET_FLOAT(ALT_CONTROLLER, ACC_ERR_MAX),	//max_err
		300.0f,	//integral_separation_err
		PARAM_GET_FLOAT(ALT_CONTROLLER, ACC_I_MIN),	//min_integrate
		PARAM_GET_FLOAT(ALT_CONTROLLER, ACC_I_MAX),	//max_integrate
		PARAM_GET_FLOAT(ALT_CONTROLLER, ALT_ACC_P),	//kp
		PARAM_GET_FLOAT(ALT_CONTROLLER, ALT_ACC_I),	//ki
		PARAM_GET_FLOAT(ALT_CONTROLLER, ALT_ACC_D),	//kd
		PARAM_GET_FLOAT(ALT_CONTROLLER, ACC_OUTPUT_MIN),	//min_output
		PARAM_GET_FLOAT(ALT_CONTROLLER, ACC_OUTPUT_MAX) );	//max_output
	//pid_controller_set_lpf(&acc_controller, 30, 0.004f);
	if(PARAM_GET_INT32(ALT_CONTROLLER, ACC_ERR_LPF_EN))
		pid_controller_set_err_lpf(&acc_controller, PARAM_GET_FLOAT(ALT_CONTROLLER, ACC_ERR_LPF_FREQ), 100);
	pid_controller_set_bias(&acc_controller, 500);	//set hover throttle
	/* init the integrate to minimal value */
	acc_controller._integrate = PARAM_GET_FLOAT(ALT_CONTROLLER, ACC_I_MIN);

}

float alt_to_vel(float alt_sp, float alt_cur)
{
	alt_controller.reference = alt_sp;	//cm
	alt_controller.current = alt_cur;	//cm
	
	return p_controller_update(&alt_controller);
}

float vel_to_accel(float vel_sp, float vel_cur)
{
	vel_controller.reference = vel_sp;
	vel_controller.current = vel_cur;
	
	/* calculate feedforward */
	vel_controller.feedforward = (vel_controller.reference - vel_controller._last_ref)/(vel_controller.dt);
	
	return p_controller_update(&vel_controller);
}

float accel_to_throttle(float acc_sp, float acc_cur)
{
	acc_controller.reference = acc_sp;
	acc_controller.current = acc_cur;

	return pid_controller_update(&acc_controller);
}

