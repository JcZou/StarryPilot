#include <math.h>
#include "ekf.h"

#define NUM_X	10
#define NUM_U	6
#define NUM_Z	10

float32_t  X_f32[NUM_X];
float32_t  U_f32[NUM_U];
float32_t  Z_f32[NUM_Z];

float32_t  F_f32[NUM_X*NUM_X];
//float32_t  B_f32[NUM_B];
//float32_t  H_f32[NUM_H];

uint8_t EKF_Init(uint16_t dim_x, uint16_t dim_u, uint16_t dim_z)
{
	
	
	return 0;
}
