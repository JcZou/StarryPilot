/*
 * File      : motor.h
 *
 *
 * Change Logs:
 * Date           Author       	Notes
 * 2017-02-25     zoujiachi   	the first version
 */

#ifndef __MOTOR_H__
#define __MOTOR_H__

#include <rtthread.h>
#include <rtdevice.h>

#define MOTOR_NUM	4

#define MOTOR_MIN_DC	0.05f		/* minimal duty cycle: 1ms/20ms=0.05 */
#define MOTOR_MAX_DC	0.1f		/* minimal duty cycle: 2ms/20ms=0.1 */	

#define	MOTOR_CTRL_FREQUENCY	0x01

#define PWM_CMD_SET_FREQ		0x01
#define PWM_CMD_ENABLE		    0x02

/* motor device and operations for RT-Thread */
struct rt_device_motor
{
    struct rt_device parent;
    const struct rt_pwm_ops *ops;
};

typedef enum
{
	MOTOR_CH1 = 1,
	MOTOR_CH2 = 2,
	MOTOR_CH3 = 4,
	MOTOR_CH4 = 8,
	MOTOR_CH5 = 16,
	MOTOR_CH6 = 32,
	MOTOR_CH7 = 64,
	MOTOR_CH8 = 128,
	MOTOR_CH_ALL = 0xFF,
}Motor_Channel;

typedef enum
{
	MOTOR_DEV_MAIN = 1,		// in io, up to 8 output
	MOTOR_DEV_AUX,			// in fmu, up to 6 output
}MOTOR_DEVICE;

typedef struct
{
	uint8_t motor_dev_type;	// 0: main output 1: aux output
	uint8_t max_output_chan_num;	// maximal output channel number
}Motor_Chan_Info;

struct rt_pwm_ops
{
    void (*pwm_configure)(rt_device_t dev, rt_uint8_t cmd, void *args);
    void (*pwm_write)(struct rt_device *device, uint8_t chan_id, float* duty_cyc);
    int (*pwm_read)(struct rt_device *device, uint8_t chan_id, float* buffer);
};

int rt_device_motor_register(const char *name, const struct rt_pwm_ops *ops, void *user_data);

#endif

