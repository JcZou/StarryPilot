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
#define MSEC_TO_TICK(ms) ((ms) * RT_TICK_PER_SECOND / 1000)
#define MAVLINK_DEV_TIMEOUT MSEC_TO_TICK(15)
static int usb_is_connected = 0;
static int mavlink_dev_need_switch = 0;

rt_err_t mavproxy_tx_done(rt_device_t dev, void *buffer);
rt_err_t mavproxy_recv_ind(rt_device_t dev, rt_size_t size);
extern void usbd_set_connect_callback(void (*callback)(int));

struct rt_barrier
{
	int count;
	struct rt_semaphore sem;
	struct rt_mutex mutex;
};
typedef struct rt_barrier* rt_barrier_t;

int rt_barrier_init(rt_barrier_t          barrier, const char *name,
                         unsigned  count, rt_uint8_t flag)
{
	if (!barrier)
		return -RT_ERROR;

	barrier->count = count;
	rt_sem_init(&barrier->sem, name, 0, flag);
	rt_mutex_init(&(barrier->mutex), name, flag);

	return RT_EOK;
}

int rt_barrier_destroy(rt_barrier_t barrier)
{
	rt_err_t result;

	if (!barrier)
		return -RT_ERROR;

	result = rt_sem_detach(&barrier->sem);
	if (result)
		return result;

	result = rt_mutex_detach(&barrier->mutex);

	return result;
}

static int rt_barrier_broadcast(rt_barrier_t barrier)
{
	rt_err_t result;

	if (barrier == RT_NULL)
		return -RT_ERROR;

	rt_enter_critical();
	while (1)
	{
		/* try to take condition semaphore */
		result = rt_sem_trytake(&(barrier->sem));
		if (result == -RT_ETIMEOUT)
		{
			/* it's timeout, release this semaphore */
			rt_sem_release(&(barrier->sem));
		}
		else if (result == RT_EOK)
		{
			/* has taken this semaphore, release it */
			rt_sem_release(&(barrier->sem));
			break;
		}
		else
		{
			rt_exit_critical();

			return result;
		}
	}
	rt_exit_critical();

	return RT_EOK;
}

int rt_barrier_wait(rt_barrier_t barrier, rt_int32_t time)
{
	rt_err_t result;
	if (!barrier)
		return -RT_ERROR;

	result = rt_mutex_take(&(barrier->mutex), time);
	if (result != RT_EOK)
		return result;

	if (barrier->count == 0)
		result = -RT_EBUSY;
	else
	{
		barrier->count--;
		if (barrier->count == 0) {/* broadcast condition */
			rt_barrier_broadcast(barrier);
		} else {
			/* The mutex was not owned by the current thread at the time of the call. */
			if (barrier->mutex.owner != rt_thread_self())
				return -RT_ERROR;
			/* unlock a mutex failed */
			if (rt_mutex_release(&barrier->mutex) != 0)
				return -RT_ERROR;

			result = rt_sem_take(&(barrier->sem), RT_WAITING_FOREVER);
			if (result != RT_EOK)
				return result;
			/* lock mutex again */
			result = rt_mutex_take(&(barrier->sem), time);
			if (result != RT_EOK)
				return result;
		}
	}

	rt_mutex_release(&(barrier->mutex));

	return result;
}

int rt_barrier_get_count(rt_barrier_t barrier)
{
	return barrier->count;
}


static struct rt_barrier _mavlink_dev_barrier;

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

	if (rt_barrier_get_count(&_mavlink_dev_barrier) == 1) {
		new_dev = rt_device_find(dev_name);
		if(new_dev == NULL) {
			Console.e(TAG, "err not find %s device\n", dev_name);
			return;
		} else {
			if (_mavlink_dev == new_dev)
				return;
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
	}

	rt_barrier_wait(&_mavlink_dev_barrier, RT_WAITING_FOREVER);

	return RT_EOK;
}

uint8_t mavlink_lowlevel_write(uint8_t* buff, uint16_t len)
{
	rt_err_t res;
	uint16_t s_bytes;
	rt_uint32_t recv_set = 0;

	if (mavlink_dev_need_switch) {
		mavlink_dev_switch();
		mavlink_dev_need_switch = 0;
	}
	if(_mavlink_dev) {
		s_bytes = rt_device_write(_mavlink_dev, 0, (void*)buff, len);
#ifdef MAV_PKG_RETRANSMIT
		uint8_t retry = 0;
		while(retry < MAX_RETRY_NUM && s_bytes != len) {
			rt_thread_delay(1);
			s_bytes = rt_device_write(_mavlink_dev, 0, (void*)buff, len);
			retry++;
		}

		if (mavlink_dev_need_switch) {
			return 1;
		}

		
#endif
		if (!usb_is_connected) {
			res = rt_event_recv(&event_mavlink_dev, EVENT_MAVLINK_DEV_TX, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, 
									MSEC_TO_TICK(30), &recv_set);
			if (res != RT_EOK) {
				Console.e(TAG, "mav tx err%d\n", res);
			}
		}

	}
	
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

	if (mavlink_dev_need_switch) {
		mavlink_dev_switch();
		mavlink_dev_need_switch = 0;
	}

	do {
		size = rt_device_read(_mavlink_dev, 0,  buff, len);
		if (size <= 0) {
			if (mavlink_dev_need_switch) {
				break;
			}
			res = rt_event_recv(&event_mavlink_dev, EVENT_MAVLINK_DEV_RX, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, 
								MAVLINK_DEV_TIMEOUT, &recv_set);
		}
	} while (size <= 0);

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
}

void mavproxy_lowlevel_init(void)
{
	rt_err_t res;

	rt_barrier_init(&_mavlink_dev_barrier, "mav_dev", 2, RT_IPC_FLAG_FIFO);

	/* create event */
	rt_event_init(&event_mavlink_dev, "mavlink_dev", RT_IPC_FLAG_FIFO);

	_mavlink_console_dev = rt_device_find(MAVLINK_CONSOLE_DEV_NAME);
	if(!_mavlink_console_dev)
		Console.e(TAG, "mavlink console device not found\n");

	if (!_mavlink_dev && !usb_is_connected) {
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

