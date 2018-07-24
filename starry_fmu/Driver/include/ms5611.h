/*
 * File      : ms5611_sensor.h
 *
 *
 * Change Logs:
 * Date           Author       	Notes
 * 2016-6-30      zoujiachi   	the first version
 */
 
#ifndef __MS5611_SENSOR_H__
#define __MS5611_SENSOR_H__

#include "stm32f4xx.h"
#include <rtthread.h>

typedef struct
{
	u16 factory_data;
	u16 c1;
	u16 c2;
	u16 c3;
	u16 c4;
	u16 c5;
	u16 c6;
	u16 crc;
}MS5611_PROM_Def;

typedef struct
{
	u32 raw_temperature;
	u32 raw_pressure;
	float temperature;
	float pressure;
	float altitude;
	u32 time_stamp;
}MS5611_REPORT_Def;

rt_err_t rt_ms5611_init(char* spi_device_name);

#endif
