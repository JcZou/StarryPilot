
#include "stm32f4xx.h"
#include "pwm_io.h"
#include "starryio_manager.h"
#include "starryio_protocol.h"
#include "motor.h"

rt_sem_t _sem_pwm_chan_recv;
static int _pwm_freq;
static float _pwm_duty_cycle[MAX_PWM_MAIN_CHAN] = {0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00};
float _remote_pwm_duty_cycle[MAX_PWM_MAIN_CHAN];
static Motor_Chan_Info _io_chan_info = {MOTOR_DEV_MAIN, MAX_PWM_MAIN_CHAN};

void pwm_io_configure(rt_device_t dev, rt_uint8_t cmd, void *args)
{
	if(cmd == PWM_CMD_SET_FREQ){
		_pwm_freq = *((int*)args);

        PWM_CONFIG_MSG pwm_conf_msg = {PWM_CMD_SET_FREQ, _pwm_freq};
        send_package(CMD_CONFIG_PWM_CHANNEL, (uint8_t*)&pwm_conf_msg, sizeof(pwm_conf_msg));
	}else if(cmd == PWM_CMD_ENABLE){
		int enable = *((int*)args);
        PWM_CONFIG_MSG pwm_conf_msg = {PWM_CMD_ENABLE, enable};

        send_package(CMD_CONFIG_PWM_CHANNEL, (uint8_t*)&pwm_conf_msg, sizeof(pwm_conf_msg));
	}
}

void pwm_io_write(struct rt_device *device, uint8_t chan_id, float* duty_cyc)
{
    PWM_CHAN_MSG pwm_chan_msg;

    for(uint8_t i = 0 ; i < MAX_PWM_MAIN_CHAN ; i++){
        pwm_chan_msg.duty_cyc[i] = duty_cyc[i];
        _pwm_duty_cycle[i] = pwm_chan_msg.duty_cyc[i];
    }
    pwm_chan_msg.chan_id = chan_id;

    send_package(CMD_SET_PWM_CHANNEL, (uint8_t*)&pwm_chan_msg, sizeof(pwm_chan_msg));
}

int pwm_io_read(struct rt_device *device, uint8_t chan_id, float* buffer)
{
    send_package(CMD_GET_PWM_CHANNEL, NULL, 0);

    //rt_err_t err = rt_sem_take(_sem_pwm_chan_recv, 100);
	rt_thread_delay(1000);
    if(1){
        for(uint8_t i = 0 ; i < MAX_PWM_MAIN_CHAN ; i++){
            if(chan_id & (1<<i))
                buffer[i] = _remote_pwm_duty_cycle[i];
        }

        return 0;
    }
	
    // Timeout
	return 1;
}

const static struct rt_pwm_ops _pwm_io_ops =
{
    pwm_io_configure,
    pwm_io_write,
    pwm_io_read,
};

int pwm_io_init(void)
{
    _sem_pwm_chan_recv = rt_sem_create("sem_pwm", 0, RT_IPC_FLAG_FIFO);

	rt_device_motor_register("motor", &_pwm_io_ops, &_io_chan_info);
	
	return 0;
}