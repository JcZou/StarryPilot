
#ifndef _BALANCE_CAR_MOTOR_H_
#define _BALANCE_CAR_MOTOR_H_

#include "global.h"

void balance_car_motor_init(void);
void balance_car_standby(int enable);
// set motor speed
// left, right: [-1, 1], where the sign indicate the direction
// motor value(1~7): pwm1, Ain1, Ain2, pwm2, Bin2, Bin3, standby
void balance_car_motor_set(float left, float right);

#endif