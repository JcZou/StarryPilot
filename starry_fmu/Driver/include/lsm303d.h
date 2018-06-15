/*
 * File      : lsm303d_sensor_sensor.h
 *
 *
 * Change Logs:
 * Date           Author       	Notes
 * 2016-6-17      zoujiachi   	the first version
 */
 
#ifndef __LSM303D_SENSOR_H__
#define __LSM303D_SENSOR_H__

#include "stm32f4xx.h"
#include <rtthread.h>

rt_err_t rt_lsm303d_init(char* spi_device_name);
rt_err_t lsm303d_mag_read(int16_t mag[3]);
rt_err_t lsm303d_acc_read(int16_t acc[3]);

#endif
