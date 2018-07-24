/*
 * File      : control.h
 *
 * Change Logs:
 * Date           Author       Notes
 * 2017-02-25     zoujiachi    first version.
 */

#ifndef _CONTROL_H__
#define _CONTROL_H__

//#include <rtthread.h>
//#include <rtdevice.h>
#include <stdint.h>
#include "quaternion.h"
#include "global.h"

#ifdef BLUEJAY
#define MAX_THROTTLE_NUM	6
#else
#define MAX_THROTTLE_NUM	4
#endif

typedef enum
{
	frame_type_1,	/* X frame */
	frame_type_2,	/* + frame */
	frame_type_3,	/* รื frame */
	frame_type_4,
}FrameType;

typedef struct
{
	double lat;
	double lon;
	uint8_t home_set;
}HomePosition;

void control_loop(void *parameter);
void control_init(void);
int control_vehicle(float dT);
void control_attitude(float dT);
void control_altitude(float dT);
void ctrl_set_throttle(float* throttle, uint8_t throttle_num);
void ctrl_unlock_vehicle(void);
void ctrl_lock_vehicle(void);
void ctrl_set_basethrottle(float throttle);
float ctrl_get_basethrottle(void);
void calculate_target_attitude(float dT);
void ctrl_att_adrc_update(void);
uint8_t control_request(bool request);
uint8_t control_set(char* name, float val);
HomePosition ctrl_get_home(void);
uint8_t ctrl_set_home(void);

#endif
