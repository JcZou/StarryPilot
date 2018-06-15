
#include "pid.h"
#include "ap_math.h"
#include "console.h"
#include "filter.h"

void pid_controller_init(  
		PID_Controler *controller,
		uint8_t limit_err_flag,
		uint8_t integrate_limit_flag,
		uint8_t integral_separation_flag,
		uint8_t feed_forward_flag,
		float dt,
		float min_err,
		float max_err,
		float integral_separation_err,
		float min_integrate,
		float max_integrate,
		float kp,
		float ki,
		float kd,
		float min_output,
		float max_output )
{
	controller->limit_err_flag = limit_err_flag;
	controller->integral_separation_flag = integral_separation_flag;
	controller->feed_forward_flag = feed_forward_flag;
	controller->dt = dt;
    controller->min_err = min_err;
    controller->max_err = max_err;
    controller->integral_separation_err = integral_separation_err;
    controller->min_integrate = min_integrate;
    controller->max_integrate = max_integrate;
    controller->kp = kp;
    controller->ki = ki;
    controller->kd = kd;
    controller->min_output = min_output;
    controller->max_output = max_output;
	
	controller->_last_error = 0.0f;
	controller->_integrate = 0.0f;
	controller->_derivative = 0.0f;
	controller->_last_ref = 0.0f;
	controller->bias = 0.0f;
	controller->feedforward = 0.0f;
	controller->output = 0.0f;
	controller->use_lpf = 0;
	controller->use_err_lpf = 0;
	controller->_err_lpf = 0.0f;
}

float pid_controller_update(PID_Controler *controller)
{
	/* store last reference */
	controller->_last_ref = controller->reference;
	
	/* calculate error */
	controller->_error = controller->reference - controller->current;
	/* error low pass filter */
	if(controller->use_err_lpf){	
		//float err = butter2_filter_process(&controller->_butter, controller->_error);
		controller->_err_lpf = controller->_err_lpf + controller->_err_alpha * (controller->_error - controller->_err_lpf);
		controller->_error = controller->_err_lpf;
	}
	if(controller->limit_err_flag){
		constrain(&controller->_error, controller->min_err, controller->max_err);
	}
	/* calculate p */
	float p_o = controller->_error * controller->kp;

	/* calculate i */
	float _di = controller->_error * controller->dt * controller->ki;
	if(controller->integral_separation_flag){
		if(fabs(controller->_error) < controller->integral_separation_err){
			controller->_integrate += _di;
		}
	}else{
		controller->_integrate += _di;
	}
	constrain(&controller->_integrate, controller->min_integrate, controller->max_integrate);
	float i_o = controller->_integrate;

	/* calculate d */
	float derivative = (controller->_error - controller->_last_error) / controller->dt * controller->kd;
	if(controller->use_lpf){
		// low pass filter
		controller->_derivative = controller->_derivative + controller->_alpha * (derivative - controller->_derivative);
	}else{
		controller->_derivative = derivative;
	}
	float d_o = controller->_derivative;
	
	/* calculate output */
	controller->output = p_o + i_o + d_o + controller->bias;
	if(controller->feed_forward_flag){
		/* add feedforward */
		controller->output += controller->feedforward;
	}
	constrain(&controller->output, controller->min_output, controller->max_output);
	
	/* record last error */
	controller->_last_error = controller->_error;
	
	return controller->output;
}

int pid_controller_set_lpf(PID_Controler *controller, float cutoff_freq, float dt)
{
	if(cutoff_freq <= 0.0f || controller->use_lpf == 0){
		return 1;
	}
	float rc = 1.0f/(2.0f*PI*cutoff_freq);
	controller->_alpha = dt / (dt + rc);
	
	controller->use_lpf = 1;
	
	return 0;
}

int pid_controller_set_err_lpf(PID_Controler *controller, float cutoff_freq, float sample_freq)
{
//	butter2_set_cutoff_frequency(&controller->_butter, sample_freq, cutoff_freq);
//	butter2_reset(&controller->_butter, 0);
	
	controller->_err_alpha = lpf_get_alpha(cutoff_freq, 1.0f/sample_freq);
	
	controller->use_err_lpf = 1;
	
	return 0;
}

void pid_controller_set_bias(PID_Controler *controller, float bias)
{
	controller->bias = bias;
}


void p_controller_init(
		P_Controler *controller, 
		uint8_t limit_err_flag,
		uint8_t feed_forward_flag,
		float dt,
		float min_err,
		float max_err,
		float kp,
		float min_output,
		float max_output )
{
	controller->limit_err_flag = limit_err_flag;
	controller->feed_forward_flag = feed_forward_flag;
	controller->dt = dt;
	controller->min_err = min_err;
	controller->max_err = max_err;
	controller->kp = kp;
	controller->min_output = min_output;
	controller->max_output = max_output;
	
	controller->_output = 0.0f;
	controller->feedforward = 0.0f;
	controller->_last_ref = 0.0f;
	controller->use_err_lpf = 0;
}

float p_controller_update(P_Controler *controller)
{
	/* store last reference */
	controller->_last_ref = controller->reference;
	
	/* calculate error */
	controller->_error = controller->reference - controller->current;
	/* error low pass filter */
	if(controller->use_err_lpf){
		controller->_error = butter2_filter_process(&controller->_butter, controller->_error);
	}
	if(controller->limit_err_flag){
		constrain(&controller->_error, controller->min_err, controller->max_err);
	}
	
	/* calculate output */
	float p_o = controller->_error * controller->kp;
	controller->_output = p_o;
	if(controller->feed_forward_flag){
		/* add feedforward */
		controller->_output += controller->feedforward;
	}
	constrain(&controller->_output, controller->min_output, controller->max_output);
	return controller->_output;
}

int p_controller_set_err_lpf(P_Controler *controller, float cutoff_freq, float sample_freq)
{
	butter2_set_cutoff_frequency(&controller->_butter, sample_freq, cutoff_freq);
	butter2_reset(&controller->_butter, 0);
	
	controller->use_err_lpf = 1;
	
	return 0;
}