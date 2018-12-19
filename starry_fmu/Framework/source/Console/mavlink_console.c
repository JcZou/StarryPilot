
#include <rtthread.h>
#include <rtdevice.h>
#include "mavproxy.h"

static struct rt_device mavlink_console_dev;

/* RT-Thread Device Interface */
/*
 * This function initializes mavlink_console device.
 */
static rt_err_t rt_mavlink_console_init(struct rt_device* dev)
{
	rt_err_t result = RT_EOK;
	RT_ASSERT(dev != RT_NULL);

	return result;
}

static rt_err_t rt_mavlink_console_open(struct rt_device* dev, rt_uint16_t oflag)
{
	RT_ASSERT(dev != RT_NULL);

	/* check device flag with the open flag */
	if((oflag & RT_DEVICE_FLAG_DMA_RX) && !(dev->flag & RT_DEVICE_FLAG_DMA_RX))
		return -RT_EIO;

	if((oflag & RT_DEVICE_FLAG_DMA_TX) && !(dev->flag & RT_DEVICE_FLAG_DMA_TX))
		return -RT_EIO;

	if((oflag & RT_DEVICE_FLAG_INT_RX) && !(dev->flag & RT_DEVICE_FLAG_INT_RX))
		return -RT_EIO;

	if((oflag & RT_DEVICE_FLAG_INT_TX) && !(dev->flag & RT_DEVICE_FLAG_INT_TX))
		return -RT_EIO;

	/* get open flags */
	dev->open_flag = oflag & 0xFFFF;

	/* initialize the Rx/Tx structure according to open flag */
	if(oflag & RT_DEVICE_FLAG_DMA_RX) {
		dev->open_flag |= RT_DEVICE_FLAG_DMA_RX;
	} else if(oflag & RT_DEVICE_FLAG_INT_RX) {
		dev->open_flag |= RT_DEVICE_FLAG_INT_RX;
	}

	if(oflag & RT_DEVICE_FLAG_DMA_TX) {
		dev->open_flag |= RT_DEVICE_FLAG_DMA_TX;
	} else if(oflag & RT_DEVICE_FLAG_INT_TX) {
		dev->open_flag |= RT_DEVICE_FLAG_INT_TX;
	}

	return RT_EOK;
}

static rt_err_t rt_mavlink_console_close(struct rt_device* dev)
{
	RT_ASSERT(dev != RT_NULL);

	/* this device has more reference count */
	if(dev->ref_count > 1) return RT_EOK;

	if(dev->open_flag & RT_DEVICE_FLAG_INT_RX) {
		dev->open_flag &= ~RT_DEVICE_FLAG_INT_RX;
	} else if(dev->open_flag & RT_DEVICE_FLAG_DMA_RX) {
		dev->open_flag &= ~RT_DEVICE_FLAG_DMA_RX;
	}

	if(dev->open_flag & RT_DEVICE_FLAG_INT_TX) {
		dev->open_flag &= ~RT_DEVICE_FLAG_INT_TX;
	} else if(dev->open_flag & RT_DEVICE_FLAG_DMA_TX) {
		dev->open_flag &= ~RT_DEVICE_FLAG_DMA_TX;
	}

	return RT_EOK;
}

static rt_size_t rt_mavlink_console_read(struct rt_device* dev,
        rt_off_t          pos,
        void*             buffer,
        rt_size_t         size)
{
	RT_ASSERT(dev != RT_NULL);

	if(size == 0) return 0;

	return mavproxy_msg_serial_control_read((uint8_t*)buffer, size);
}

static rt_size_t rt_mavlink_console_write(struct rt_device* dev,
        rt_off_t          pos,
        const void*       buffer,
        rt_size_t         size)
{
	rt_size_t len;
	rt_size_t left = size;
	uint8_t* ptr = buffer;
	RT_ASSERT(dev != RT_NULL);

	if(size == 0) return 0;

	while(left) {
		len = left < 70 ? left : 70;

		if(mavproxy_msg_serial_control_send((uint8_t*)buffer, len)) {
			break;
		}

		left -= len;
	}

	return size - left;
}

static rt_err_t rt_mavlink_console_control(struct rt_device* dev,
        rt_uint8_t        cmd,
        void*             args)
{
	RT_ASSERT(dev != RT_NULL);

	switch(cmd) {
		case RT_DEVICE_CTRL_SUSPEND:
			/* suspend device */
			dev->flag |= RT_DEVICE_FLAG_SUSPENDED;
			break;

		case RT_DEVICE_CTRL_RESUME:
			/* resume device */
			dev->flag &= ~RT_DEVICE_FLAG_SUSPENDED;
			break;

		case RT_DEVICE_CTRL_CONFIG:
			/* configure device */
			break;

		case RT_DEVICE_CTRL_GET_INT:
			break;

		default :
			break;
	}

	return RT_EOK;
}


/*
 * mavlink console register
 */
rt_err_t rt_hw_mavlink_console_init(void)
{
	struct rt_device* device;

	device = &mavlink_console_dev;

	device->type        = RT_Device_Class_Char;
	device->rx_indicate = RT_NULL;
	device->tx_complete = RT_NULL;

	device->init        = rt_mavlink_console_init;
	device->open        = rt_mavlink_console_open;
	device->close       = rt_mavlink_console_close;
	device->read        = rt_mavlink_console_read;
	device->write       = rt_mavlink_console_write;
	device->control     = rt_mavlink_console_control;
	device->user_data   = RT_NULL;

	/* register a character device */
	return rt_device_register(device, "mav", RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX);
}


