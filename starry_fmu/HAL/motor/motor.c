/*
 * File      : motor.c
 *
 *
 * Change Logs:
 * Date           Author       	Notes
 * 2017-02-25     zoujiachi   	the first version
 */
 
#include "motor.h"
#include "pwm.h"
#include "console.h"
#include <stdlib.h>
#include <string.h>

static struct rt_device_motor _hw_motor;
static char* TAG = "motor";

static rt_size_t _motor_read(rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t size)
{
	struct rt_device_motor *motor = (struct rt_device_motor *)dev;
	float throttle_dc[MOTOR_NUM];
	
	motor->ops->pwm_read(dev, (uint8_t)pos, throttle_dc);
	
	*((float*)buffer) = (throttle_dc[0] - MOTOR_MIN_DC) / (MOTOR_MAX_DC - MOTOR_MIN_DC);
	*((float*)buffer+1) = (throttle_dc[1] - MOTOR_MIN_DC) / (MOTOR_MAX_DC - MOTOR_MIN_DC);
	*((float*)buffer+2) = (throttle_dc[2] - MOTOR_MIN_DC) / (MOTOR_MAX_DC - MOTOR_MIN_DC);
	*((float*)buffer+3) = (throttle_dc[3] - MOTOR_MIN_DC) / (MOTOR_MAX_DC - MOTOR_MIN_DC);
#if MOTOR_NUM == 6
	*((float*)buffer+4) = (throttle_dc[4] - MOTOR_MIN_DC) / (MOTOR_MAX_DC - MOTOR_MIN_DC);
	*((float*)buffer+5) = (throttle_dc[5] - MOTOR_MIN_DC) / (MOTOR_MAX_DC - MOTOR_MIN_DC);
#endif
	
    return size;
}

static rt_size_t _motor_write(rt_device_t dev, rt_off_t pos, const void *buffer, rt_size_t size)
{
	struct rt_device_motor *motor = (struct rt_device_motor *)dev;
	static float throttle_dc[MOTOR_NUM];
	float throttle;
	
	if(size > MOTOR_NUM){
		Console.e(TAG, "unsupport motor num:%ld\n", size);
		return 0;
	}
	
	/* set motor throttle */
	if( pos >= MOTOR_CH1 && pos <= MOTOR_CH_ALL ){
		if(pos & MOTOR_CH1){
			throttle = *(float*)buffer;
			
			if(throttle > 1.0f)
				throttle = 1.0f;
			if(throttle < 0.0f)
				throttle = 0.0f;
			
			throttle_dc[0] = MOTOR_MIN_DC + throttle * (MOTOR_MAX_DC - MOTOR_MIN_DC);
		}
		if(pos & MOTOR_CH2){
			throttle = *((float*)buffer+1);
			
			if(throttle > 1.0f)
				throttle = 1.0f;
			if(throttle < 0.0f)
				throttle = 0.0f;
			
			throttle_dc[1] = MOTOR_MIN_DC + throttle * (MOTOR_MAX_DC - MOTOR_MIN_DC);
		}
		if(pos & MOTOR_CH3){
			throttle = *((float*)buffer+2);
			
			if(throttle > 1.0f)
				throttle = 1.0f;
			if(throttle < 0.0f)
				throttle = 0.0f;
			
			throttle_dc[2] = MOTOR_MIN_DC + throttle * (MOTOR_MAX_DC - MOTOR_MIN_DC);
		}
		if(pos & MOTOR_CH4){
			throttle = *((float*)buffer+3);
			
			if(throttle > 1.0f)
				throttle = 1.0f;
			if(throttle < 0.0f)
				throttle = 0.0f;
			
			throttle_dc[3] = MOTOR_MIN_DC + throttle * (MOTOR_MAX_DC - MOTOR_MIN_DC);
		}
#if MOTOR_NUM > 4
		if(pos & MOTOR_CH5){
			throttle = *((float*)buffer+4);
			
			if(throttle > 1.0f)
				throttle = 1.0f;
			if(throttle < 0.0f)
				throttle = 0.0f;
			
			throttle_dc[4] = MOTOR_MIN_DC + throttle * (MOTOR_MAX_DC - MOTOR_MIN_DC);
		}
		if(pos & MOTOR_CH6){
			throttle = *((float*)buffer+5);
			
			if(throttle > 1.0f)
				throttle = 1.0f;
			if(throttle < 0.0f)
				throttle = 0.0f;
			
			throttle_dc[5] = MOTOR_MIN_DC + throttle * (MOTOR_MAX_DC - MOTOR_MIN_DC);
		}
#endif
		
		motor->ops->pwm_write(dev, (uint8_t)pos, throttle_dc);
	}
	
    return size;
}

static rt_err_t  _motor_control(rt_device_t dev, rt_uint8_t cmd, void *args)
{
	struct rt_device_motor *motor = (struct rt_device_motor *)dev;
	
	motor->ops->pwm_configure(dev, cmd, args);
	
    return 0;
}

int handle_motor_shell_cmd(int argc, char** argv)
{
	if(argc > 1){
		if(strcmp(argv[1], "setall") == 0 && argc == 3){
			float base_throttle = atof(argv[2]);
			float throttle[MOTOR_NUM];
			if(base_throttle >= 0.0f && base_throttle <= 1.0f){
				for(int i = 0 ; i < MOTOR_NUM ; i++)
					throttle[i] = MOTOR_MIN_DC + base_throttle * (MOTOR_MAX_DC - MOTOR_MIN_DC);
				
				_hw_motor.ops->pwm_write((rt_device_t)&_hw_motor, MOTOR_CH_ALL, throttle);
				Console.print("motor have been set to %.2f\n", base_throttle);
			}else{
				Console.print("throttle must between 0.0~1.0\n");
				return 1;
			}
		}
		else if(strcmp(argv[1], "set") == 0 && argc == 2 + MOTOR_NUM){
			float throttle[MOTOR_NUM];
			float base_throttle[MOTOR_NUM];
			for(int i = 0 ; i < MOTOR_NUM ; i++){
				base_throttle[i] = atof(argv[2+i]);
				if(base_throttle[i] < 0.0f || base_throttle[i] > 1.0f){
					Console.print("throttle must between 0.0~1.0\n");
					return 1;
				}
				throttle[i] = MOTOR_MIN_DC + base_throttle[i] * (MOTOR_MAX_DC - MOTOR_MIN_DC);
			}
			_hw_motor.ops->pwm_write((rt_device_t)&_hw_motor, MOTOR_CH_ALL, throttle);
			Console.print("motor have been set to: ");
			for(int i = 0 ; i < MOTOR_NUM ; i++)
				Console.print("%.2f ", base_throttle[i]);
			Console.print("\n");
		}
		else if(strcmp(argv[1], "get") == 0){
			float throttle_dc[4];
			float throttle[4];
			
			_hw_motor.ops->pwm_read((rt_device_t)&_hw_motor, MOTOR_CH_ALL, throttle_dc);
			
			for(int i = 0 ; i < MOTOR_NUM ; i++){
				throttle[i] = (throttle_dc[i] - MOTOR_MIN_DC) / (MOTOR_MAX_DC - MOTOR_MIN_DC);
				Console.print("%.2f ", throttle[i]);
			}
			Console.print("\n");
		}
		else if(strcmp(argv[1], "switch") == 0 && argc == 3){
			if(strcmp(argv[2], "on") == 0){
				int on_off = 1;
				_hw_motor.ops->pwm_configure((rt_device_t)&_hw_motor, PWM_CMD_SWITCH, (void*)&on_off);
				Console.print("motor switch on success\n");
			}else if(strcmp(argv[2], "off") == 0){
				int on_off = 0;
				_hw_motor.ops->pwm_configure((rt_device_t)&_hw_motor, PWM_CMD_SWITCH, (void*)&on_off);
				Console.print("motor switch off success\n");
			}
		}else{
			Console.print("incorrect cmd, using help!\n");
			return 1;
		}
	}
	
	return 0;
}

int rt_device_motor_register(const char *name, const struct rt_pwm_ops *ops, void *user_data)
{
    _hw_motor.parent.type         = RT_Device_Class_Miscellaneous;
    _hw_motor.parent.rx_indicate  = RT_NULL;
    _hw_motor.parent.tx_complete  = RT_NULL;

    _hw_motor.parent.init         = RT_NULL;
    _hw_motor.parent.open         = RT_NULL;
    _hw_motor.parent.close        = RT_NULL;
    _hw_motor.parent.read         = _motor_read;
    _hw_motor.parent.write        = _motor_write;
    _hw_motor.parent.control      = _motor_control;

    _hw_motor.ops                 = ops;
    _hw_motor.parent.user_data    = user_data;

    /* register a character device */
    rt_device_register(&_hw_motor.parent, name, RT_DEVICE_FLAG_RDWR);

    return 0;
}
