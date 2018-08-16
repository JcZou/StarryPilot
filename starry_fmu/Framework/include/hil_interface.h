/*
 * File      : hil_interface.h
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-03-06     zoujiachi    first version.
 */

#ifndef __HIL_INTERFACE_H__
#define __HIL_INTERFACE_H__

#include "global.h"
 
typedef enum
{
	HIL_SENSOR_LEVEL,
	HIL_STATE_LEVEL
}HIL_Option;

int hil_sensor_collect(void);
int hil_interface_init(HIL_Option hil_op);
bool hil_baro_poll(void);

#endif
