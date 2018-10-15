#ifndef __PWM_IO_H__
#define __PWM_IO_H__

#define MAX_PWM_MAIN_CHAN      8

#include <rtthread.h>
#include <rtdevice.h>

typedef struct{
  float duty_cyc[MAX_PWM_MAIN_CHAN];
  uint8_t chan_id;
}PWM_CHAN_MSG;

typedef struct{
  uint8_t cmd;
      int val;
}PWM_CONFIG_MSG;

extern float _remote_pwm_duty_cycle[MAX_PWM_MAIN_CHAN];
extern rt_sem_t _sem_pwm_chan_recv;

#endif