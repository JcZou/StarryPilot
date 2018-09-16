/*
 * File      : copetr_main.h
 *
 *
 * Change Logs:
 * Date           Author       	Notes
 * 2017-10-27     zoujiachi   	the first version
 */
 
#ifndef __COPTER_MAIN_H__
#define __COPTER_MAIN_H__

#include "global.h"

//#define AHRS_PERIOD			2
//#define CONTROL_PERIOD		2
//#define POS_EST_PERIOD		10
#define AHRS_PERIOD			2
#define EKF_PERIOD			4
#define CONTROL_PERIOD		4
#define POS_EST_PERIOD		10

typedef enum
{
	AHRS_Period = 0,
	Control_Period,
	Pos_Period,
}Copter_Period;

void copter_entry(void *parameter);
uint32_t copter_get_event_period(Copter_Period event);

#endif