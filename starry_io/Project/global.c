#include "global.h"

uint8_t constrain(float *val, float min_val, float max_val)
{
	if(*val > max_val){
		*val = max_val;
		return 1;
	}
	if(*val < min_val){
		*val = min_val;
		return 2;
	}
	
	return 0;
}
