/*
 * File      : led.c
 *
 *
 * Change Logs:
 * Date           Author       	Notes
 * 2016-6-6    	  zoujiachi   	the first version
 */
 

#include <rtthread.h>
#include <rtdevice.h>
#include "i2c_soft.h"
#include "led.h"
#include "console.h"
#include "uMCN.h"
#include "rc.h"

#define FMU_LED_PIN		43
#define SALVE_ADDR		0x55	//7 bit addr
#define DUTY_OUT0		0x81
#define DUTY_OUT1		0x82
#define DUTY_OUT2		0x83
#define ENABLE_SHDN		0x84
#define WRITE			0
#define READ			1

#define BRIGHT	0x01

static rt_device_t _pin_device;
static rt_device_t _i2c_device;
static uint8_t _rgbled_color;

static char* TAG = "LED";

MCN_DECLARE(RC_STATUS);

static McnNode_t rc_status_node_t;

uint8_t color_index[][4] = 
{
	{LED_RED 	, 0x00 , 0x00 , BRIGHT},
	{LED_GREEN 	, 0x00 , BRIGHT , 0x00},
	{LED_BLUE 	, BRIGHT , 0x00 , 0x00},
	{LED_YELLOW	, 0x00 , BRIGHT , BRIGHT},
	{LED_WHITE	, BRIGHT , BRIGHT , BRIGHT},
};

uint8_t TCA62724_write_reg(uint8_t duty0, uint8_t duty1, uint8_t duty2)
{
	uint16_t flags = 0x0000;
	rt_off_t pos = (rt_off_t)((flags << 16) | SALVE_ADDR);
	
	uint8_t buffer[6];
	buffer[0] = DUTY_OUT0;
	buffer[1] = duty0;
	buffer[2] = DUTY_OUT1;
	buffer[3] = duty1;
	buffer[4] = DUTY_OUT2;
	buffer[5] = duty2;
	
	rt_size_t w_bytes = rt_device_write(_i2c_device, pos, (void*)buffer, sizeof(buffer));
	
	return w_bytes>0 ? 0 : 1;
}

uint8_t TCA62724_read_reg(uint8_t reg_val[2])
{
	uint16_t flags = 0x0000;
	rt_off_t pos = (rt_off_t)((flags << 16) | SALVE_ADDR | 1);
	
	uint8_t buffer[2];
	
	rt_device_read(_i2c_device, pos, (void*)buffer, sizeof(buffer));
	
	reg_val[0] = buffer[0];
	reg_val[1] = buffer[1];
	
	return 0;
}

uint8_t TCA62724_blink_control(uint8_t on_ff)
{
	uint16_t flags = RT_I2C_WR;
	rt_off_t pos = (rt_off_t)((flags << 16) | SALVE_ADDR);
	
	uint8_t buffer[2];
	buffer[0] = ENABLE_SHDN;
	
	if(on_ff)
		buffer[1] = 0x03;
	else
		buffer[1] = 0x00;
	
	rt_device_write(_i2c_device, pos, (void*)buffer, sizeof(buffer));
	
	return 0;
}

uint8_t TCA62724_set_color(LED_COLOR color)
{
	return TCA62724_write_reg(color_index[color][1], color_index[color][2], color_index[color][3]);
}

uint8_t TCA62724_set_color_with_bright(LED_COLOR color, uint8_t bright)
{
	return TCA62724_write_reg(color_index[color][1]?bright:0, color_index[color][2]?bright:0, color_index[color][3]?bright:0);
}

void led_on(void)
{
	struct rt_device_pin_status pin_sta = {FMU_LED_PIN , 0};
	
	if(_pin_device != RT_NULL)
	{
		_pin_device->write(_pin_device, 0, (void*)&pin_sta, sizeof(&pin_sta));
	}
}

void led_off(void)
{
	struct rt_device_pin_status pin_sta = {FMU_LED_PIN , 1};;
	
	if(_pin_device != RT_NULL)
	{
		_pin_device->write(_pin_device, 0, (void*)&pin_sta, sizeof(&pin_sta));
	}
}

int device_led_init(void)
{
	struct rt_device_pin_mode mode = {FMU_LED_PIN , PIN_MODE_OUTPUT , PIN_OUT_TYPE_OD};
	
	_pin_device = rt_device_find("pin");
	
	if(_pin_device != RT_NULL)
	{
		rt_device_open(_pin_device , RT_DEVICE_OFLAG_RDWR);
		_pin_device->control(_pin_device , 0 , &mode);
    }
	else
	{
		Console.e(TAG, "can not find pin device\n");
		return 1;
	}
	
	rt_err_t res = device_i2c_init("i2c2");
	if(res != RT_EOK){
		Console.e(TAG, "i2c2 init fail\n");
		return 1;
	}
	
	_i2c_device = rt_device_find("i2c2");
	
	if(_i2c_device != RT_NULL)
	{
		rt_err_t err = rt_device_open(_i2c_device , RT_DEVICE_OFLAG_RDWR);
		if(err != RT_EOK){
			Console.e(TAG, "led i2c dev open fail\n");
			return 1;
		}
	}
	else
	{
		Console.e(TAG, "can not find i2c2 device\n");
		return 1;
	}
	
	return 0;
}

void led_rc_status_cb(void *parameter)
{
	RC_STATUS status;
	mcn_copy(MCN_ID(RC_STATUS), rc_status_node_t, &status);
	
	if(status == RC_LOCK_STATUS){
		_rgbled_color = LED_BLUE;
	}else if(status == RC_UNLOCK_STATUS){
		_rgbled_color = LED_GREEN;
	}
}

void led_entry(void *parameter)
{
	static int _inc;
	static int bright = 0;
	uint32_t delay_time;
	
	_rgbled_color = LED_BLUE;
	
	TCA62724_blink_control(1);
	TCA62724_set_color(_rgbled_color);
	
	rc_status_node_t = mcn_subscribe(MCN_ID(RC_STATUS), led_rc_status_cb);
	if(rc_status_node_t == NULL)
		Console.e(TAG, "RC_STATUS subscribe err\n");

	while(1)
	{
		if(bright == 0) _inc = 1;
		if(bright == 15) _inc = -1;
		
		bright += _inc;
		
		if(bright == 0){
			delay_time = 100;
		}else{
			if(bright == 15){
				delay_time = 300;
			}else{
				delay_time = 50;
			}
		}
		
		TCA62724_set_color_with_bright(_rgbled_color, bright);
		rt_thread_delay(delay_time);
	}
}