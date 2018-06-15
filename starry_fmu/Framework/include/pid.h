
#ifndef __PID_H__
#define __PID_H__

#include "global.h"
#include "filter.h"

typedef struct
{
	/* public var */
    uint8_t limit_err_flag;
    uint8_t integral_separation_flag;
	uint8_t feed_forward_flag;
	uint8_t use_lpf;
	uint8_t use_err_lpf;
	float dt;
	float min_err;
    float max_err;
    float integral_separation_err;
	float min_integrate;
    float max_integrate;
    float kp;
    float ki;
    float kd;
    float output;
    float min_output;
	float max_output;
	float bias;
	float feedforward;
	
	float reference;
    float current;
    /* private var */
    float _error;
    float _last_error;
	float _last_ref;
	float _integrate;
	float _derivative;
    float _alpha;	//LPF
	float _err_alpha;
	float _err_lpf;
	Butter2 _butter;
}PID_Controler;

typedef struct
{
	/* public var */
	uint8_t limit_err_flag;
	uint8_t feed_forward_flag;
	uint8_t use_err_lpf;
	float dt;
	float min_err;
    float max_err;
    float kp;
    float min_output;
	float max_output;
	
	float reference;
    float current;
	float feedforward;
	
	/* private var */
	float _output;
	float _error;
	float _last_ref;
	Butter2 _butter;
}P_Controler;

void pid_controller_init(  
		PID_Controler *controller,
		uint8_t limit_err_flag,	//limit_err_flag
		uint8_t integrate_limit_flag,	//integrate_limit_flag
		uint8_t integral_separation_flag,	//integral_separation_flag
		uint8_t feed_forward_flag,	//feed_forward_flag
		float dt,	//dt
		float min_err,	//min_err
		float max_err,	//max_err
		float integral_separation_err,	//integral_separation_err
		float min_integrate,	//min_integrate
		float max_integrate,	//max_integrate
		float kp,	//kp
		float ki,	//ki
		float kd,	//kd
		float min_output,	//min_output
		float max_output );	//max_output
float pid_controller_update(PID_Controler *controller);
int pid_controller_set_lpf(PID_Controler *controller, float cutoff_freq, float dt);
int pid_controller_set_err_lpf(PID_Controler *controller, float cutoff_freq, float sample_freq);
void pid_controller_set_bias(PID_Controler *controller, float bias);

void p_controller_init(
		P_Controler *controller, 
		uint8_t limit_err_flag,
		uint8_t feed_forward_flag,
		float dt,
		float min_err,
		float max_err,
		float kp,
		float min_output,
		float max_output );
float p_controller_update(P_Controler *controller);

#endif
