/*
 * File      : starryio_manager.h
 *
 * Change Logs:
 * Date           Author       Notes
 * 2017-02-13     zoujiachi   	the first version
 */
 
#ifndef __PX4IO_MANAGER_H__
#define __PX4IO_MANAGER_H__

#include "stm32f4xx.h"
#include <rtthread.h>

void starryio_entry(void *parameter);
rt_device_t starryio_get_device(void);
void px4io_reset_rx_ind(void);
rt_err_t request_reboot(void);
rt_err_t reply_sync(void);
uint8_t send_package(uint8_t cmd, uint8_t* data, uint16_t len);

extern uint8_t ppm_send_freq;

#endif 

