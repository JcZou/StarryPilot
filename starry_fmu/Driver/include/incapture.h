#ifndef __INCAPTURE_H__
#define __INCAPTURE_H__

#include "global.h"

typedef struct {
	uint32_t left_count;
	uint32_t right_count;
} wheel_encoder;

void capture_init(void);
wheel_encoder capture_read(void);

#endif