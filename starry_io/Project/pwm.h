/**
  ******************************************************************************
  * @file    pwm.h 
  * @author  J Zou
  * @version V1.0
  * @date    10-Oct-2018
  * @brief   PWM Driver
  ******************************************************************************
*/  

#ifndef __PWM_H__
#define __PWM_H__

#include "global.h"

#define TIMER_FREQUENCY	3000000						// Timer frequency: 3M
#define PWM_DEFAULT_FREQUENCY	50					// pwm default frequqncy: 50Hz
#define MAX_PWM_CHAN	8							// maximal pwm output channel number

// PWM configure command
#define PWM_CMD_SET_FREQ      0x01
#define PWM_CMD_ENABLE        0x02

// PWM channel id
#define	PWM_CHAN_1            1
#define	PWM_CHAN_2            2
#define	PWM_CHAN_3            4
#define	PWM_CHAN_4            8
#define	PWM_CHAN_5            16
#define	PWM_CHAN_6            32
#define PWM_CHAN_7            64
#define	PWM_CHAN_8            128
#define	PWM_CHAN_ALL          0xFF

typedef struct{
  float duty_cyc[MAX_PWM_CHAN];
  uint8_t chan_id;
}PWM_CHAN_MSG;

typedef struct{
  uint8_t cmd;
      int val;
}PWM_CONFIG_MSG;

uint8_t pwm_init(void);
uint8_t pwm_write(float* duty_cyc, uint8_t chan_id);
uint8_t pwm_read(float* buffer, uint8_t chan_id);
uint8_t pwm_configure(uint8_t cmd, void *args);

#endif
