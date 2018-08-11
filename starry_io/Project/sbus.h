/**
  ******************************************************************************
  * @file    sbus.h 
  * @author  weety
  * @version V1.0
  * @date    8-Aug-2018
  * @brief   Sbus Decoder
  ******************************************************************************
*/  

#ifndef __SBUS_H__
#define __SBUS_H__
#include <stdio.h>
#include <stdbool.h>
#include "time.h"
#include "protocol.h"
#include "usart.h"

unsigned sbus_dropped_frames(void);

void sbus1_output(uint16_t *values, uint16_t num_values);
void sbus2_output(uint16_t *values, uint16_t num_values);
bool sbus_input(uint16_t *values, uint16_t *num_values, bool *sbus_failsafe, 
		bool *sbus_frame_drop, uint16_t max_channels);
uint8_t send_sbus_value(void);

uint8_t sbus_init(void);
#endif
