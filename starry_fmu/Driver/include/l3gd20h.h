/*
 * File      : l3gd20h_sensor.h
 *
 *
 * Change Logs:
 * Date           Author       	Notes
 * 2016-6-17      zoujiachi   	the first version
 */
 
#ifndef __L3GD20H_SENSOR_H__
#define __L3GD20H_SENSOR_H__

#include "stm32f4xx.h"
#include <rtthread.h>

rt_err_t rt_l3gd20h_init(char* spi_device_name);
rt_err_t l3gd20h_gyr_read_raw(int16_t gyr[3]);
rt_err_t l3gd20h_gyr_read_rad(float gyr[3]);

#endif
