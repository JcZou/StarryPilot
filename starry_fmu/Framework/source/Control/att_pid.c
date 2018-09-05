/*
 * File      : att_pid.c
 *
 * Change Logs:
 * Date           Author       Notes
 * 2017-03-01     zoujiachi    first version.
 */

#include <math.h>
#include "param.h"
#include "ap_math.h"
#include "global.h"
#include "console.h"
#include "filter.h"
#include "adrc_att.h"

#define MIN_TAKEOFF_THROTTLE		0.3f
#define RATES_I_LIMIT				0.2f
#define I_LIMIT						1.0f

static float i_accum[3] = {0,0,0};
float rate_i_accum[3] = {0,0,0};
static float pre_gyr[3];
static float pre_err[3];
static float _derivative[3] = {0, 0, 0};
static float err_alpha = 0.0f;
static float d_alpha = 0.0f;
//static float err_lpf[3] = {0,0,0};
//static float pre_err_rate[3] = {0,0,0};

uint8_t att_pid_init(float dt)
{
	for(int i = 0 ; i < 3 ; i++){
		i_accum[i] = 0.0f;
		rate_i_accum[i] = 0.0f;
		pre_gyr[i] = 0.0f;
		pre_err[i] = 0.0f;
	}
	
	err_alpha = lpf_get_alpha(50, dt);
	d_alpha = lpf_get_alpha(50, dt);
	_derivative[0] = _derivative[1] = _derivative[2] = 0.0f;
	//err_lpf[0] = err_lpf[1] = err_lpf[2] = 0.0f;
	//pre_err_rate[0] = pre_err_rate[1] = pre_err_rate[2] = 0.0f;
	
    return 0;
}

extern ADRC_Log adrc_log;
uint8_t att_pid_update(const float input[3],float output[3], const float gyr_rad[3], float dt, float throttle)
{
	float att_p[3] = {	PARAM_GET_FLOAT(ATT_CONTROLLER, ATT_ROLL_P), 
						PARAM_GET_FLOAT(ATT_CONTROLLER, ATT_PITCH_P), 
						PARAM_GET_FLOAT(ATT_CONTROLLER, ATT_YAW_P)};
	
	float att_rate_p[3] = { PARAM_GET_FLOAT(ATT_CONTROLLER, ATT_ROLL_RATE_P),
							PARAM_GET_FLOAT(ATT_CONTROLLER, ATT_PITCH_RATE_P),
							PARAM_GET_FLOAT(ATT_CONTROLLER, ATT_YAW_RATE_P)};
	
	float att_rate_i[3] = { PARAM_GET_FLOAT(ATT_CONTROLLER, ATT_ROLL_RATE_I),
							PARAM_GET_FLOAT(ATT_CONTROLLER, ATT_PITCH_RATE_I),
							PARAM_GET_FLOAT(ATT_CONTROLLER, ATT_YAW_RATE_I)};
	
	float att_rate_d[3] = { PARAM_GET_FLOAT(ATT_CONTROLLER, ATT_ROLL_RATE_D),
							PARAM_GET_FLOAT(ATT_CONTROLLER, ATT_PITCH_RATE_D),
							PARAM_GET_FLOAT(ATT_CONTROLLER, ATT_YAW_RATE_D)};
	
	float att_output_limit[3] = {	PARAM_GET_FLOAT(ATT_CONTROLLER, ATT_ROLLOUT_LIM),
									PARAM_GET_FLOAT(ATT_CONTROLLER, ATT_PITCHOUT_LIM),
									PARAM_GET_FLOAT(ATT_CONTROLLER, ATT_YAWOUT_LIM)};
	
	float att_rate_i_limit[3] = {	PARAM_GET_FLOAT(ATT_CONTROLLER, ATT_ROLLR_I_LIM),
									PARAM_GET_FLOAT(ATT_CONTROLLER, ATT_PITCHR_I_LIM),
									PARAM_GET_FLOAT(ATT_CONTROLLER, ATT_YAWR_I_LIM)};
					
    /* outter ring controls angle */
    float rates_sp[3] = {0};
    for(int i=0;i<3;i++)
    {
		/* outter ring only contains P controller */
		rates_sp[i] = input[i] * att_p[i];
    }
	
	adrc_log.sp_rate = rates_sp[1];

	float err_rates[3] = {0};
    for(int i = 0 ; i<3 ; i++)
    {
        err_rates[i] = rates_sp[i] - gyr_rad[i];
		//err_lpf[i] = err_lpf[i] + err_alpha*(err_rates[i]-err_lpf[i]);
//		float deriv = (pre_gyr[i] - gyr_rad[i]) / dt * att_rate_d[i];
		float deriv = (err_rates[i] - pre_err[i]) / dt * att_rate_d[i];
		_derivative[i] = _derivative[i] + d_alpha*(deriv - _derivative[i]);
		output[i] = err_rates[i] * att_rate_p[i] + _derivative[i] + rate_i_accum[i];
		//pre_gyr[i] = gyr_rad[i];
		constrain(&output[i], -att_output_limit[i], att_output_limit[i]);
		pre_err[i] = err_rates[i];
    }
	
	
	//TODO: should check the throttle here?
	//if(throttle > MIN_TAKEOFF_THROTTLE){
	if(throttle > 0.3f){
		for(int i=0;i<3;i++){
			float rate_i = rate_i_accum[i] + err_rates[i] * att_rate_i[i] * dt;
//			if(rate_i > -RATES_I_LIMIT && rate_i < RATES_I_LIMIT 
//					&& output[i] > -RATES_I_LIMIT && output[i] < RATES_I_LIMIT){
//				rate_i_accum[i] = rate_i;
//			}
			if(rate_i > -att_rate_i_limit[i] && rate_i < att_rate_i_limit[i]){
				rate_i_accum[i] = rate_i;
			}
		}
	}
	
	//param_release();
	
	return 0;
}

uint8_t att_yaw_pid_control(const float input, float* output, float gyr_rad, float dt, float throttle)
{
	float att_p = PARAM_GET_FLOAT(ATT_CONTROLLER, ATT_YAW_P);
	
	float att_rate_p = PARAM_GET_FLOAT(ATT_CONTROLLER, ATT_YAW_RATE_P);
	
	float att_rate_i = PARAM_GET_FLOAT(ATT_CONTROLLER, ATT_YAW_RATE_I);
	
	float att_rate_d = PARAM_GET_FLOAT(ATT_CONTROLLER, ATT_YAW_RATE_D);
	
	float att_output_limit = PARAM_GET_FLOAT(ATT_CONTROLLER, ATT_YAWOUT_LIM);
	
	float att_rate_i_limit = PARAM_GET_FLOAT(ATT_CONTROLLER, ATT_YAWR_I_LIM);
					
    /* outter ring controls angle */
	float rates_sp = input * att_p;

	/* outter ring controls angular rate */
	float err_rates = rates_sp - gyr_rad;
	float deriv = (err_rates - pre_err[2]) / dt * att_rate_d;
	_derivative[2] = _derivative[2] + d_alpha*(deriv - _derivative[2]);
	*output = err_rates * att_rate_p + _derivative[2] + rate_i_accum[2];
	constrain(output, -att_output_limit, att_output_limit);
	pre_err[2] = err_rates;
	
	
	//TODO: should check the throttle here?
	//if(throttle > MIN_TAKEOFF_THROTTLE){
	if(throttle > 0.3f){
		float rate_i = rate_i_accum[2] + err_rates * att_rate_i * dt;
		if(rate_i > -att_rate_i_limit && rate_i < att_rate_i_limit){
			rate_i_accum[2] = rate_i;
		}
	}
	
	return 0;
}

