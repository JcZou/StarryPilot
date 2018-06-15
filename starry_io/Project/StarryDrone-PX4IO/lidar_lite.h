/**
  ******************************************************************************
  * @file    lidar_lite.h 
  * @author  J Zou
  * @version V1.0
  * @date    21-Nov-2017
  * @brief   Lidar-Lite Driver
  ******************************************************************************
*/  

#ifndef  _LIDAR_LITE_H_
#define  _LIDAR_LITE_H_

#include "stm32f10x.h"

uint8_t lidar_lite_init(void);
uint8_t send_lidar_distance(void);
uint8_t lidar_lite_ready(void);
void lidar_lite_clear_ready(void);

#endif
