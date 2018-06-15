/*
 * File      : application.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-01-05     Bernard      the first version
 * 2014-04-27     Bernard      make code cleanup. 
 */

#include <board.h>
#include <rtdevice.h>
#include <rtthread.h>
#include <stdio.h>
#include "board.h"
#include "att_estimator.h"
#include "global.h"
#include "px4io_manager.h"
#include "control_main.h"
#include "mavlink_protocol.h"
#include "pos_estimator.h"
#include "led.h"
#include "param.h"
#include "sensor_manager.h"
#include "console.h"
#include "statistic.h"
#include "copter_main.h"
#include "file_manager.h"
#include "logger.h"

#ifdef RT_USING_LWIP
#include <lwip/sys.h>
#include <lwip/api.h>
#include <netif/ethernetif.h>
#include "stm32f4xx_eth.h"
#endif

#ifdef RT_USING_GDB
#include <gdb_stub.h>
#endif

#include "sdio.h"

static rt_thread_t tid0;

static char thread_mavlink_stack[2048];
struct rt_thread thread_mavlink_handle;

static char thread_px4io_stack[1024];
struct rt_thread thread_px4io_handle;

static char thread_copter_stack[2048];
struct rt_thread thread_copter_handle;

static char thread_logger_stack[1024];
struct rt_thread thread_logger_handle;

static char thread_led_stack[512];
struct rt_thread thread_led_handle;

FATFS FatFs;

void vehicle_main_loop(void *parameter);
void rt_init_thread_entry(void* parameter)
{
	rt_err_t res;
	
	statistic_init();
	
    /* GDB STUB */
#ifdef RT_USING_GDB
    gdb_set_device("uart6");
    gdb_start();
#endif
	
	//fm_init("0:");
	param_init();
	device_led_init();
	device_sensor_init();
	device_mavlink_init();
	
	//rt_console_set_device(CONSOLE_DEVICE);
	
	//Console.print("heap base:%p end:%p\n", STM32_SRAM_BEGIN, STM32_SRAM_END);

    /* LwIP Initialization */
#ifdef RT_USING_LWIP
    {
        extern void lwip_sys_init(void);

        /* register ethernetif device */
        eth_system_device_init();

        rt_hw_stm32_eth_init();

        /* init lwip system */
        lwip_sys_init();
        rt_kprintf("TCP/IP initialized!\n");
    }
#endif
	
	/* create thread */
	res = rt_thread_init(&thread_copter_handle,
						   "copter",
						   copter_entry,
						   RT_NULL,
						   &thread_copter_stack[0],
						   sizeof(thread_copter_stack),COPTER_THREAD_PRIORITY,1);
	if (res == RT_EOK)
		rt_thread_startup(&thread_copter_handle);
	
	res = rt_thread_init(&thread_px4io_handle,
						   "px4io",
						   px4io_loop,
						   RT_NULL,
						   &thread_px4io_stack[0],
						   sizeof(thread_px4io_stack),PX4IO_THREAD_PRIORITY,1);
	if (res == RT_EOK)
		rt_thread_startup(&thread_px4io_handle);
	
	res = rt_thread_init(&thread_mavlink_handle,
						   "mavlink",
						   mavlink_loop,
						   RT_NULL,
						   &thread_mavlink_stack[0],
						   sizeof(thread_mavlink_stack),MAVLINK_THREAD_PRIORITY,1);
	if (res == RT_EOK)
		rt_thread_startup(&thread_mavlink_handle);
	
	res = rt_thread_init(&thread_logger_handle,
						   "logger",
						   logger_entry,
						   RT_NULL,
						   &thread_logger_stack[0],
						   sizeof(thread_logger_stack),LED_THREAD_PRIORITY,1);
	if (res == RT_EOK)
		rt_thread_startup(&thread_logger_handle);
	
	res = rt_thread_init(&thread_led_handle,
						   "led",
						   led_entry,
						   RT_NULL,
						   &thread_led_stack[0],
						   sizeof(thread_led_stack),LED_THREAD_PRIORITY,1);
	if (res == RT_EOK)
		rt_thread_startup(&thread_led_handle);
	
	/* delete itself */
	rt_thread_delete(tid0);
}

int rt_application_init()
{
	usb_cdc_init();
	fm_init("0:");
	console_init(CONSOLE_INTERFACE_SERIAL);
	rt_console_set_device(CONSOLE_DEVICE);
	rt_show_version();
	
    tid0 = rt_thread_create("init",
        rt_init_thread_entry, RT_NULL,
        2048, RT_THREAD_PRIORITY_MAX/2, 20);

    if (tid0 != RT_NULL)
        rt_thread_startup(tid0);

    return 0;
}

/*@}*/
