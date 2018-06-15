/*
 * File      : control_alt.h
 *
 *
 * Change Logs:
 * Date           Author       	Notes
 * 2017-09-29     zoujiachi   	the first version
 */
 
#ifndef __CONTROL_ALT_H__
#define __CONTROL_ALT_H__

#include "global.h"

void alt_controller_reset(void);
//void ctrl_alt_setCurThrottle(float throttle);
void ctrl_alt_setDesAlt(float alt);
float ctrl_alt_getDesAlt(void);
float ctrl_alt_updateDesAlt(float throttle, float dT);
float ctrl_alt_pid_update(float des_alt, float cur_alt, float cur_vel, float cur_accel, float dt);
float alt_to_vel(float alt_sp, float alt_cur);
float vel_to_accel(float vel_sp, float vel_cur);
float accel_to_throttle(float acc_sp, float acc_cur);

#endif
