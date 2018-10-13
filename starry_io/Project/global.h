#ifndef __GLOBAL_H__
#define __GLOBAL_H__

#ifndef USE_STDPERIPH_DRIVER
 #define USE_STDPERIPH_DRIVER
#endif 

#include <stdio.h>
#include "stm32f10x.h"

//#define USE_LIDAR
#define USE_PWM_OUTPUT

uint8_t constrain(float *val, float min_val, float max_val);

#endif

