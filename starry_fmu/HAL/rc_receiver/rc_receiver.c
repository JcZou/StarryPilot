/*
 * File      : rc_receiver.c
 *
 * Change Logs:
 * Date           Author       Notes
 * 2016-12-16     zoujiachi    first version.
 */

#include <stdio.h>
#include <rthw.h>
#include <rtdevice.h>
#include <rtthread.h>

static rt_device_t serial_device;

//this function will be callback on rt_hw_serial_isr()
static rt_err_t rc_serial_rx_ind(rt_device_t dev, rt_size_t size)
{
	rt_size_t bytes;
	uint8_t ch[RT_SERIAL_RB_BUFSZ];
	
	bytes = rt_device_read(serial_device , 0 , ch , size);
	
	if(bytes){
		for(uint32_t j = 0 ; j<bytes ; j++){
			printf("%c" , ch[j]);
		}
	}
	
    return RT_EOK;
}

rt_err_t rc_init(rt_device_t dev)
{
	rt_device_set_rx_indicate(serial_device, rc_serial_rx_ind);
	rt_device_open(serial_device, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX);
	
	return RT_EOK;
}

rt_err_t rt_rc_init(char* serial_device_name)
{
	rt_err_t res = RT_EOK;
	
	serial_device = rt_device_find(serial_device_name);
	
	if(serial_device == RT_NULL)
    {
        rt_kprintf("serial device %s not found!\r\n", serial_device_name);
        return RT_EEMPTY;
    }
	
	rt_device_set_rx_indicate(serial_device, rc_serial_rx_ind);
	rt_device_open(serial_device, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX);
	
	return res;
}
