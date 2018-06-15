/*
 * File      : mpu6000.h
 *
 *
 * Change Logs:
 * Date           Author       	Notes
 * 2018-04-14     zoujiachi   	the first version
 */
 
#ifndef __MPU6000_H__
#define __MPU6000_H__

#include "stm32f4xx.h"
#include <rtthread.h>

rt_err_t rt_mpu6000_init(char* spi_device_name);

#endif
