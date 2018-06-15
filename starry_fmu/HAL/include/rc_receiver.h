/*
 * File      : rc_receiver.h
 *
 *
 * Change Logs:
 * Date           Author       	Notes
 * 2016-12-16     zoujiachi   	the first version
 */
 
#ifndef __RC_RECEIVER_H__
#define __RC_RECEIVER_H__

#include "stm32f4xx.h"
#include <rtthread.h>

rt_err_t rt_rc_init(char* serial_device_name);

#endif
