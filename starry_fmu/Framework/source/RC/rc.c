/*
 * File      : remote_controller.c
 *
 *
 * Change Logs:
 * Date           Author       	Notes
 * 2017-08-29     zoujiachi   	the first version
 */

#include <string.h>
#include "global.h"
#include "console.h"
#include "motor.h"
#include "rc.h"
#include "delay.h"
#include "control_main.h"
#include "pos_estimator.h"
#include "control_alt.h"
#include "uMCN.h"

/* If more than RC_LOST_SIGNAL_TIME ms we don't receive ppm signal, then we think rc is disconnected */
#define RC_LOST_SIGNAL_TIME		300

typedef enum
{
	RC_LOCK_SIGNAL = 0,
	RC_UNLOCK_SIGNAL,
	RC_RETURN_CENTRAL_SIGNAL,
	RC_NORMAL_SIGNAL,
}RC_SIGNAL;

static char* TAG = "RC";
static float _chan_val[CHAN_NUM];
static bool _rc_connect = false;
static uint32_t _time_last_receive = 0;
RC_STATUS _rc_status = RC_LOCK_STATUS;

MCN_DEFINE(RC_STATUS, sizeof(RC_STATUS));	

float rc_raw2chanval(uint32_t raw)
{
	uint32_t raw_temp = raw;
	
	if(raw_temp > 2000)
		raw_temp = 2000;
	if(raw_temp < 1000)
		raw_temp = 1000;
	
	return (float)(raw_temp-1000)/1000;
}

float rc_get_chanval(RC_CHANEL rc_chan)
{
	float chan_val;
	if((int)rc_chan >= CHAN_NUM)
		return -1;
	
	rt_enter_critical();
	chan_val = _chan_val[rc_chan];
	rt_exit_critical();
	
	return chan_val;
}

bool rc_get_connect_status(void)
{
	if(time_nowMs() - _time_last_receive > RC_LOST_SIGNAL_TIME){
		_rc_connect = false;
	}else{
		_rc_connect = true;
	}
	
	return _rc_connect;
}

RC_SIGNAL rc_get_ppm_signal(void)
{
	if(_chan_val[CHAN_THROTTLE]<0.1f && _chan_val[CHAN_PITCH]<0.1f){
		return RC_LOCK_SIGNAL;
	}else if(_chan_val[CHAN_THROTTLE]<0.1f && _chan_val[CHAN_PITCH]>0.9f){
		return RC_UNLOCK_SIGNAL;
	}else if(_chan_val[CHAN_THROTTLE]<0.1f && _chan_val[CHAN_ROLL]>0.4f && _chan_val[CHAN_ROLL]<0.6f
			&& _chan_val[CHAN_PITCH]>0.4f && _chan_val[CHAN_PITCH]<0.6f){
		return RC_RETURN_CENTRAL_SIGNAL;
	}else{
		return RC_NORMAL_SIGNAL;
	}
}

void rc_enter_status(RC_STATUS status)
{
	if(status == RC_LOCK_STATUS){
		ctrl_lock_vehicle();
		_rc_status = status;
		Console.print("enter RC_LOCK_STATUS\n");
	}else if(status == RC_LOCK_READY_STATUS){
		_rc_status = RC_LOCK_READY_STATUS;
		Console.print("enter RC_LOCK_READY_STATUS\n");
	}else if(status == RC_UNLOCK_STATUS){
		ctrl_unlock_vehicle();
		_rc_status = status;
		Console.print("enter RC_UNLOCK_STATUS\n");
		///* reset home position when vehicle unlock */
		//set_home_cur_alt();
		alt_controller_reset();
		pos_est_reset();
	}else if(status == RC_UNLOCK_READY_STATUS){
		_rc_status = RC_UNLOCK_READY_STATUS;
		Console.print("enter RC_UNLOCK_READY_STATUS\n");
	}
	
	/* publish rc status */
	mcn_publish(MCN_ID(RC_STATUS), &status);
}

// chan_val: 0~1.0
void rc_handle_ppm_signal(float* chan_val)
{
	for(int i = 0 ; i < CHAN_NUM ; i++){
		_chan_val[i] = chan_val[i];
	}
	
	_time_last_receive = time_nowMs();
	
	switch((int)_rc_status)
	{
		case RC_LOCK_STATUS:
		{
			if(rc_get_ppm_signal() == RC_UNLOCK_SIGNAL){
				rc_enter_status(RC_UNLOCK_READY_STATUS);
			}
		}break;
		case RC_LOCK_READY_STATUS:
		{
			if(rc_get_ppm_signal() == RC_RETURN_CENTRAL_SIGNAL){
				rc_enter_status(RC_LOCK_STATUS);
			}
		}break;
		case RC_UNLOCK_STATUS:
		{
			if(rc_get_ppm_signal() == RC_LOCK_SIGNAL){
				rc_enter_status(RC_LOCK_READY_STATUS);
			}
		}break;
		case RC_UNLOCK_READY_STATUS:
		{
			if(rc_get_ppm_signal() == RC_RETURN_CENTRAL_SIGNAL){
				rc_enter_status(RC_UNLOCK_STATUS);
			}
		}break;
		default:
		{
			Console.e(TAG, "rc unknown status:%d\n", _rc_status);
		}break;
	}
}

int handle_rc_shell_cmd(int argc, char** argv)
{
	if(argc > 1){
		if(strcmp(argv[1], "status") == 0){
			Console.print("rc connected: %s\n", rc_get_connect_status() ? "true" : "false");
			for(int i = 0 ; i < CHAN_NUM ; i++){
				Console.print("ch%d:%.2f ", i+1, _chan_val[i]);
			}
			Console.print("\n");
		}
	}
	
	return 0;
}
