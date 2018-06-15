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

#ifdef BLUEJAY
#define MOTOR_NUM	6
#else
#define MOTOR_NUM	4
#endif

#define MOTOR_MIN_DC	0.05f	/* minimal duty cycle: 1/20=0.05 */
#define MOTOR_MAX_DC	0.1f		/* minimal duty cycle: 2/20=0.1 */

#define	MOTOR_CTRL_FREQUENCY	0x01

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
#if MOTOR_NUM == 4
	MOTOR_CH_ALL = 0x0F,
#elif MOTOR_NUM == 6
	MOTOR_CH_ALL = 0x3F,
#endif
}Motor_Channel;

struct rt_pwm_ops
{
    void (*pwm_configure)(rt_device_t dev, rt_uint8_t cmd, void *args);
    void (*pwm_write)(struct rt_device *device, uint8_t chanel, float* duty_cyc);
    int (*pwm_read)(struct rt_device *device, uint8_t chanel, float* buffer);
};

int rt_device_motor_register(const char *name, const struct rt_pwm_ops *ops, void *user_data);

#endif

