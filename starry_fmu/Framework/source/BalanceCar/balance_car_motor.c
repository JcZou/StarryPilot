
#include "motor.h"
#include "balance_car_motor.h"
#include "console.h"

#define MOTOR_USED	0x7F

static rt_device_t motor_device_t;
static float _standby = 0.0f;

void balance_car_motor_init(void)
{
	// main output motor
	motor_device_t = rt_device_find("motor");
	if(motor_device_t == RT_NULL){
		Console.e("Balance_Motor", "can't find motor device\n");
		return;
	}
	rt_device_open(motor_device_t , RT_DEVICE_OFLAG_RDWR);
	
	_standby = 0.0f;
}

void balance_car_standby(int enable)
{
	float val;
	if(enable){
		val = 0.0f;
	}else{
		val = 1.0f;
	}
	
	rt_device_write(motor_device_t, MOTOR_CH7, &val, 7);
}

// set motor speed
// left, right: [-1, 1], where the sign indicate the direction
// motor value(1~7): pwm1, Ain1, Ain2, pwm2, Bin2, Bin3, standby
void balance_car_motor_set(float left, float right)
{
	float motor_val[7];
	float Ain1, Ain2, Bin1, Bin2;
	
	if(left){
		motor_val[0] = left;
		motor_val[1] = 0.0f;
		motor_val[2] = 1.0f;
	}else{
		motor_val[0] = -left;
		motor_val[1] = 1.0f;
		motor_val[2] = 0.0f;
	}
	
	if(right){
		motor_val[3] = right;
		motor_val[4] = 1.0f;
		motor_val[5] = 0.0f;
	}else{
		motor_val[3] = -right;
		motor_val[4] = 0.0f;
		motor_val[5] = 1.0f;
	}
	
	motor_val[6] = _standby;
	
	rt_device_write(motor_device_t, MOTOR_USED, motor_val, 7);
}