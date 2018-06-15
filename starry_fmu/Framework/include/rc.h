/*
 * File      : remote_controller.h
 *
 *
 * Change Logs:
 * Date           Author       	Notes
 * 2017-08-29     zoujiachi   	the first version
 */

#ifndef __REMOTE_CONTROLLER_H__
#define __REMOTE_CONTROLLER_H__

#include <stdint.h>

#define CHAN_NUM			6

#ifdef BLUEJAY
typedef enum
{
	CHAN_THROTTLE = 0,	//channel 1
	CHAN_ROLL,	
	CHAN_PITCH,
	CHAN_YAW,
	CHAN_CTRL_MODE,
	CHAN_THROTTLE_SWITCH,
}RC_CHANEL;
#else
typedef enum
{
	CHAN_ROLL = 0,	//channel 1
	CHAN_PITCH,
	CHAN_THROTTLE,
	CHAN_YAW,
	CHAN_CTRL_MODE,
	CHAN_THROTTLE_SWITCH,
}RC_CHANEL;
#endif

typedef enum
{
	RC_LOCK_STATUS = 0,
	RC_LOCK_READY_STATUS,
	RC_UNLOCK_STATUS,
	RC_UNLOCK_READY_STATUS,
}RC_STATUS;

float rc_raw2chanval(uint32_t raw);
float rc_get_chanval(RC_CHANEL rc_chan);
void rc_enter_status(RC_STATUS status);
void rc_handle_ppm_signal(float* chan_val);

#endif
