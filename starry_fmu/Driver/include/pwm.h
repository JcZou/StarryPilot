/*
 * File      : pwm.h
 *
 *
 * Change Logs:
 * Date           Author       	Notes
 * 2016-06-22     zoujiachi   	the first version
 */

#ifndef __PWM_H__
#define __PWM_H__

//#include <rtdevice.h>

#define PWM_CMD_FREQ		0x01
#define PWM_CMD_SWITCH		0x02

int stm32_pwm_init(void);

#endif


