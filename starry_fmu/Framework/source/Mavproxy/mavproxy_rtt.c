/*
* File      : mavproxy_rtt.c
*
*
* Change Logs:
* Date			Author			Notes
* 2018-08-06	weety		the first version
*/

#include <rthw.h>
#include <rtdevice.h>
#include <rtthread.h>
#include <stdio.h>
#include <string.h>
#include "console.h"
#include "mavproxy.h"
#include "shell.h"

#define EVENT_MAVLINK_DEV_RX		(1<<1)
#define EVENT_MAVLINK_DEV_TX		(1<<2)

#define MAV_PKG_RETRANSMIT
#define MAX_RETRY_NUM				5

static char *TAG = "MAV_Dev";

extern rt_device_t _console_device;

#define UART_CONSOLE_DEV_NAME "uart3"
#define MAVLINK_CONSOLE_DEV_NAME "mav"
static rt_device_t _mavlink_console_dev = RT_NULL;

#define USB_MAVLINK_DEV_NAME "usb"
#define UART_MAVLINK_DEV_NAME "uart2"
static rt_device_t _mavlink_dev = RT_NULL;
static struct rt_event event_mavlink_dev;
#define MSEC_TO_TICKS(ms) ((ms) * RT_TICK_PER_SECOND / 1000)
#define MAVLINK_DEV_TIMEOUT MSEC_TO_TICKS(15)
static int usb_is_connected = 0;
static int mavlink_dev_need_switch = 0;
static struct rt_mutex mav_send_lock;
static struct rt_mutex mav_read_lock;

rt_err_t mavproxy_tx_done(rt_device_t dev, void *buffer);
rt_err_t mavproxy_recv_ind(rt_device_t dev, rt_size_t size);

void usbd_is_connected(int connect)
{
	if (usb_is_connected != connect) {
		usb_is_connected = connect;
		mavlink_dev_need_switch = 1;
	}
}

rt_err_t mavproxy_tx_done(rt_device_t dev, void *buffer)
{
	return rt_event_send(&event_mavlink_dev, EVENT_MAVLINK_DEV_TX);
}

int mavlink_dev_switch(void)
{
	const char *dev_name = usb_is_connected ? USB_MAVLINK_DEV_NAME : UART_MAVLINK_DEV_NAME;
	rt_uint16_t oflag = usb_is_connected ? (RT_DEVICE_OFLAG_RDWR) : \
		(RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_DMA_RX | RT_DEVICE_FLAG_DMA_TX);
	rt_device_t new_dev = NULL;

	rt_mutex_take(&mav_read_lock, RT_WAITING_FOREVER);
	new_dev = rt_device_find(dev_name);
	if(new_dev == NULL) {
		Console.e(TAG, "err not find %s device\n", dev_name);
		rt_mutex_release(&mav_read_lock);
		return -RT_EEMPTY;
	} else {
		if (_mavlink_dev == new_dev) {
			rt_mutex_release(&mav_read_lock);
			return RT_EOK;
		}
		if (_mavlink_dev) {
			rt_device_close(_mavlink_dev);
			rt_device_set_rx_indicate(_mavlink_dev, RT_NULL);
			rt_device_set_tx_complete(_mavlink_dev, RT_NULL);
		}
		rt_device_open(new_dev , oflag);
		/* set receive indicate function */
		rt_err_t err = rt_device_set_rx_indicate(new_dev, mavproxy_recv_ind);
		if(err != RT_EOK)
			Console.e(TAG, "set mavlink receive indicate err:%d\n", err);
		
		if (!usb_is_connected) {
			rt_device_set_tx_complete(new_dev, mavproxy_tx_done);
		}
		_mavlink_dev = new_dev;
	}

	rt_mutex_release(&mav_read_lock);

	return RT_EOK;
}

uint8_t mavlink_lowlevel_write(uint8_t* buff, uint16_t len)
{
	rt_err_t res;
	uint16_t s_bytes = 0;
	rt_uint32_t recv_set = 0;

	rt_mutex_take(&mav_send_lock, RT_WAITING_FOREVER);

	if (mavlink_dev_need_switch) {
		mavlink_dev_switch();
		mavlink_dev_need_switch = 0;
	}

	if(_mavlink_dev) {
		s_bytes = rt_device_write(_mavlink_dev, 0, (void*)buff, len);
#ifdef MAV_PKG_RETRANSMIT
		uint8_t retry = 0;
		while((retry < MAX_RETRY_NUM) && (s_bytes != len)) {
			rt_thread_delay(1);
			s_bytes = rt_device_write(_mavlink_dev, 0, (void*)buff, len);
			retry++;
		}
#endif
		if (!usb_is_connected && !mavlink_dev_need_switch) {
			res = rt_event_recv(&event_mavlink_dev, EVENT_MAVLINK_DEV_TX, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, 
									MSEC_TO_TICKS(30), &recv_set);
			if (res != RT_EOK) {
				Console.e(TAG, "mav tx err%d\n", res);
			}
		}
	}

	rt_mutex_release(&mav_send_lock);
	
	if(s_bytes == len)
		return 0;
	else
		return 1;
}

int mavlink_lowlevel_read(uint8_t* buff, uint16_t len)
{
	int size = 0;
	rt_err_t res = RT_EOK;
	rt_uint32_t recv_set = 0;

	rt_mutex_take(&mav_read_lock, RT_WAITING_FOREVER);

	do {
		size = rt_device_read(_mavlink_dev, 0,  buff, len);
		if (size <= 0) {
			res = rt_event_recv(&event_mavlink_dev, EVENT_MAVLINK_DEV_RX, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, 
								MAVLINK_DEV_TIMEOUT, &recv_set);
			if ((res != RT_EOK) && mavlink_dev_need_switch) {
				break;
			}
		}
	} while (size <= 0);

	rt_mutex_release(&mav_read_lock);

	return size;
}

rt_err_t mavproxy_recv_ind(rt_device_t dev, rt_size_t size)
{
	return rt_event_send(&event_mavlink_dev, EVENT_MAVLINK_DEV_RX);
}

int mavproxy_console_proc(int count)
{
	if (_mavlink_console_dev) {
		if (_mavlink_console_dev->user_data == RT_NULL) {
			_mavlink_console_dev->user_data = (void*)-1;
			console_redirect_device(MAVLINK_CONSOLE_DEV_NAME);
			rt_console_set_device(MAVLINK_CONSOLE_DEV_NAME);
			finsh_set_device(MAVLINK_CONSOLE_DEV_NAME);
		}
		if (_mavlink_console_dev->rx_indicate) {
			_mavlink_console_dev->rx_indicate(_mavlink_console_dev, count);
		}
	}

	return 0;
}

int handle_exit_shell_cmd(int argc, char** argv)
{
	if (_mavlink_console_dev) {
		if (_mavlink_console_dev->user_data == (void*)-1) {
			Console.print("Redirect console device to %s\n", UART_CONSOLE_DEV_NAME);
			_mavlink_console_dev->user_data = RT_NULL;
			console_redirect_device(UART_CONSOLE_DEV_NAME);
			rt_console_set_device(UART_CONSOLE_DEV_NAME);
			finsh_set_device(UART_CONSOLE_DEV_NAME);
			Console.print("\n");
		}
	}

	return 0;
}

void mavproxy_lowlevel_init(void)
{
	rt_err_t res;

	rt_mutex_init(&mav_send_lock, "mav_send", RT_IPC_FLAG_FIFO);
	rt_mutex_init(&mav_read_lock, "mav_read", RT_IPC_FLAG_FIFO);

	/* create event */
	rt_event_init(&event_mavlink_dev, "mavlink_dev", RT_IPC_FLAG_FIFO);

	_mavlink_console_dev = rt_device_find(MAVLINK_CONSOLE_DEV_NAME);
	if(!_mavlink_console_dev)
		Console.e(TAG, "mavlink console device not found\n");

	if (!_mavlink_dev) {
		_mavlink_dev = rt_device_find(UART_MAVLINK_DEV_NAME);
		if(_mavlink_dev == NULL) {
			Console.e(TAG, "err not find %s device\n", UART_MAVLINK_DEV_NAME);
		} else {
			rt_device_open(_mavlink_dev , RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_DMA_RX | RT_DEVICE_FLAG_DMA_TX);
			rt_device_set_tx_complete(_mavlink_dev, mavproxy_tx_done);
			res = rt_device_set_rx_indicate(_mavlink_dev, mavproxy_recv_ind);
			if(res != RT_EOK)
				Console.e(TAG, "set mavlink receive indicate err:%d\n", res);
		}
	}
}

