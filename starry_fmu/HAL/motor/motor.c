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
#include "pwm_io.h"
#include "console.h"
#include "ap_math.h"
#include "starryio_manager.h"
#include "starryio_protocol.h"
#include <stdlib.h>
#include <string.h> 

static struct rt_device_motor _hw_motor;		// main output, in io, up to 8 output
static struct rt_device_motor _hw_motor_aux;	// aux output, in fmu, up to 6 output
static char* TAG = "motor";

static rt_size_t _motor_read(rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t size)
{
	struct rt_device_motor *motor = (struct rt_device_motor *)dev;
	float throttle_dc[8];
	
	int res = motor->ops->pwm_read(dev, (uint8_t)pos, throttle_dc);
	
	// read error
	if(res)
		return 0;

	for(uint8_t i = 0 ; i < size ; i++){
		*((float*)buffer) = (throttle_dc[i] - MOTOR_MIN_DC) / (MOTOR_MAX_DC - MOTOR_MIN_DC);
	}

    return size;
}

static rt_size_t _motor_write(rt_device_t dev, rt_off_t pos, const void *buffer, rt_size_t size)
{
	struct rt_device_motor *motor = (struct rt_device_motor *)dev;
	static float throttle_dc[8];
	float throttle;
	
	Motor_Chan_Info *chan_info = (Motor_Chan_Info*)dev->user_data;
	if(size > chan_info->max_output_chan_num){
		Console.e(TAG, "unsupport motor num:%ld\n", size);
		return 0;
	}
	
	/* set motor throttle */
	for(uint8_t i = 0 ; i < size ; i++){
		if(pos & (1<<i)){
			throttle = *((float*)buffer+i);

			constrain(&throttle, 0.0f, 1.0f);
			throttle_dc[i] = MOTOR_MIN_DC + throttle * (MOTOR_MAX_DC - MOTOR_MIN_DC);
		}
	}

	motor->ops->pwm_write(dev, (uint8_t)pos, throttle_dc);
	
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
	//TODO: add -aux flag to control motor out channel
	if(argc > 1){
		if(strcmp(argv[1], "setall") == 0 && argc == 3){
			float base_throttle = atof(argv[2]);
			float throttle[8] = {0.0f};
			if(base_throttle >= 0.0f && base_throttle <= 1.0f){
				for(int i = 0 ; i < MAX_PWM_IO_CHAN ; i++)
					throttle[i] = MOTOR_MIN_DC + base_throttle * (MOTOR_MAX_DC - MOTOR_MIN_DC);
				
				_hw_motor.ops->pwm_write((rt_device_t)&_hw_motor, MOTOR_CH_ALL, throttle);
				Console.print("motor have been set to %.2f\n", base_throttle);
			}else{
				Console.print("throttle must between 0.0~1.0\n");
				return 1;
			}
		}
		else if(strcmp(argv[1], "set") == 0 && argc == 2 + MAX_PWM_IO_CHAN){
			float throttle[MAX_PWM_IO_CHAN];
			float base_throttle[MAX_PWM_IO_CHAN];
			for(int i = 0 ; i < MAX_PWM_IO_CHAN ; i++){
				base_throttle[i] = atof(argv[2+i]);
				if(base_throttle[i] < 0.0f || base_throttle[i] > 1.0f){
					Console.print("throttle must between 0.0~1.0\n");
					return 1;
				}
				throttle[i] = MOTOR_MIN_DC + base_throttle[i] * (MOTOR_MAX_DC - MOTOR_MIN_DC);
			}
			_hw_motor.ops->pwm_write((rt_device_t)&_hw_motor, MOTOR_CH_ALL, throttle);
			Console.print("motor have been set to: ");
			for(int i = 0 ; i < MAX_PWM_IO_CHAN ; i++)
				Console.print("%.2f ", base_throttle[i]);
			Console.print("\n");
		}
		else if(strcmp(argv[1], "get") == 0){
			float throttle_dc[4];
			float throttle[4];
			
			_hw_motor.ops->pwm_read((rt_device_t)&_hw_motor, MOTOR_CH_ALL, throttle_dc);
			
			for(int i = 0 ; i < MAX_PWM_IO_CHAN ; i++){
				throttle[i] = (throttle_dc[i] - MOTOR_MIN_DC) / (MOTOR_MAX_DC - MOTOR_MIN_DC);
				Console.print("%.2f ", throttle[i]);
			}
			Console.print("\n");
		}
		else if(strcmp(argv[1], "switch") == 0 && argc == 3){
			if(strcmp(argv[2], "on") == 0){
				int on_off = 1;
				_hw_motor.ops->pwm_configure((rt_device_t)&_hw_motor, PWM_CMD_ENABLE, (void*)&on_off);
				Console.print("motor switch on success\n");
			}else if(strcmp(argv[2], "off") == 0){
				int on_off = 0;
				_hw_motor.ops->pwm_configure((rt_device_t)&_hw_motor, PWM_CMD_ENABLE, (void*)&on_off);
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
	struct rt_device_motor *motor_dev_t = RT_NULL;

	Motor_Chan_Info *chan_info_t = (Motor_Chan_Info*)user_data;
	if(chan_info_t->motor_dev_type == MOTOR_DEV_MAIN){
		motor_dev_t = &_hw_motor;
	}else{
		motor_dev_t = &_hw_motor_aux;
	}

    motor_dev_t->parent.type         = RT_Device_Class_Miscellaneous;
    motor_dev_t->parent.rx_indicate  = RT_NULL;
    motor_dev_t->parent.tx_complete  = RT_NULL;

    motor_dev_t->parent.init         = RT_NULL;
    motor_dev_t->parent.open         = RT_NULL;
    motor_dev_t->parent.close        = RT_NULL;
    motor_dev_t->parent.read         = _motor_read;
    motor_dev_t->parent.write        = _motor_write;
    motor_dev_t->parent.control      = _motor_control;

    motor_dev_t->ops                 = ops;
    motor_dev_t->parent.user_data    = user_data;

    /* register a character device */
    rt_device_register(&motor_dev_t->parent, name, RT_DEVICE_FLAG_RDWR);

    return 0;
}
