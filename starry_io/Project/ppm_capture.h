/**
  ******************************************************************************
  * @file    ppm_capture.h 
  * @author  J Zou
  * @version V1.0
  * @date    16-Feb-2017
  * @brief   PPM Decoder
  ******************************************************************************
  */  

#ifndef  _PPM_CAPTURE_H_
#define  _PPM_CAPTURE_H_

#include "stm32f10x.h"

enum
{
	STA_EDGE_1 = 0,
	STA_EDGE_2,
	STA_EDGE_3,
	STA_EDGE_4,
	STA_EDGE_5,
	STA_EDGE_6,
	STA_EDGE_7,
	STA_EDGE_8,
	STA_EDGE_9
};

typedef struct
{
	uint8_t		status;
	uint8_t		bad_frame;
	uint16_t	last_ic;
	uint32_t	ppm_val[8];	/* ppm raw value in microseconds */
}ppm_status_machine_param;

uint8_t ppm_capture_init(void);
void get_ppm_value(uint32_t val[8]);
uint8_t ppm_ready(void);
void ppm_clear_ready(void);
uint8_t send_ppm_value(void);
uint8_t config_ppm_send_freq(uint8_t freq);

#endif
