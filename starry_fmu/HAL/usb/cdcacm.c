/*
 * File      : cdcacm.c
 *
 *
 * Change Logs:
 * Date           Author       	Notes
 * 2016-03-21     zoujiachi   	the first version
 */
#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>
#include "cdcacm.h"
#include "ringbuffer.h"
#include "console.h"

#ifdef USB_OTG_HS_CORE
	#define RECEIVE_RINGBUFF_SIZE		2*RX_FIFO_HS_SIZE
#else 
	#define RECEIVE_RINGBUFF_SIZE		2*RX_FIFO_FS_SIZE
#endif

__ALIGN_BEGIN USB_OTG_CORE_HANDLE    USB_OTG_dev __ALIGN_END ;

static uint16_t cdc_data_tx(void);
static uint16_t cdc_data_rx(uint32_t Len);

/* add volatile, otherwise the compiler will optimize these value and the value can't be read correctly */	
static volatile uint8_t data_sent, data_receive;
#ifdef USB_OTG_HS_CORE
	static uint8_t receiv_pack[RX_FIFO_HS_SIZE];
#else 
	static uint8_t receiv_pack[RX_FIFO_FS_SIZE];
#endif

uint8_t rb_buffer[RECEIVE_RINGBUFF_SIZE];
ringbuffer* rb;

static struct rt_device usb_device;

CDC_IF_Prop_TypeDef VCP_fops = 
{
  cdc_data_tx,
  cdc_data_rx
};

extern void usbd_set_connect_callback(void (*callback)(int));
extern void usbd_is_connected(int connect);

/* This callback is called when the send status is finished */
static uint16_t cdc_data_tx(void)
{ 
	data_sent =1;

	/* invoke callback */
	if (usb_device.tx_complete != RT_NULL)
	{
		usb_device.tx_complete(&usb_device, RT_NULL);
	}
	
	return USBD_OK;
}

/* This callback is called when the receive status is finished */
static uint16_t cdc_data_rx(uint32_t Len)
{ 
	data_receive = 1;
	
	/* we need to receive data here, otherwise, the send will be suspended */
	DCD_EP_PrepareRx(&USB_OTG_dev, CDC_OUT_EP, receiv_pack, Len);
	
	/* push data to ring buffer */
	for(int i = 0 ; i < Len ; i++){
		ringbuffer_putc(rb, receiv_pack[i]);
	}

	/* invoke callback */
	if (usb_device.rx_indicate != RT_NULL)
	{
		rt_size_t rx_length;
		//rt_base_t level;
	
		/* get rx length */
		rx_length = ringbuffer_getlen(rb);

		usb_device.rx_indicate(&usb_device, rx_length);
	}

	return USBD_OK;
}

uint8_t cdc_check_sent(void)
{
	return	data_sent;
}

uint8_t cdc_check_receive(void)
{
	return data_receive;
}

uint32_t cdc_get_receive_cnt(void)
{
	return ringbuffer_getlen(rb);
}

void cdc_send_data(uint8_t* pbuf, uint32_t buf_len)
{
	/* clear send flag */
	data_sent =0; 
	
	DCD_EP_Tx (&USB_OTG_dev, CDC_IN_EP, pbuf , buf_len);
}

uint8_t cdc_receive_data(uint8_t *pbuf, uint32_t len)
{
	data_receive = 0;
	
	return ringbuffer_get(rb, pbuf, len);
}

rt_size_t usb_read(rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t size)
{
	return cdc_receive_data((uint8_t*)buffer, size);
}

rt_size_t usb_write(rt_device_t dev, rt_off_t pos, const void *buffer, rt_size_t size)
{	
	/* usb is busy now */
	if(!cdc_check_sent())
		return 0;

	cdc_send_data((uint8_t*)buffer, size);

	return size;
}

rt_err_t  usb_control(rt_device_t dev, rt_uint8_t cmd, void *args)
{
	switch(cmd)
	{
		case USB_CMD_RECEIVE_CNT:
		{
			*(uint32_t*)args = cdc_get_receive_cnt();
		}break;
		case USB_CMD_RECEIVE_FIN:
		{
			*(uint8_t*)args = cdc_check_receive();
		}break;
		case USB_CMD_SEND_FIN:
		{
			*(uint8_t*)args = cdc_check_sent();
		}break;
		default:
			break;
	}
	
	return RT_EOK;
}

uint8_t usb_cdc_init(void)
{
	rt_err_t res;
	data_sent = 1;
	data_receive = 0;
	
	rb = ringbuffer_static_create(rb_buffer, RECEIVE_RINGBUFF_SIZE);

	usbd_set_connect_callback(usbd_is_connected);
	USBD_Init(&USB_OTG_dev,
#ifdef USE_USB_OTG_HS 
            USB_OTG_HS_CORE_ID,
#else            
            USB_OTG_FS_CORE_ID,
#endif  
            &USR_desc, 
            &USBD_CDC_cb, 
            &USR_cb);
	
	/* set device type */
    usb_device.type    = RT_Device_Class_USBDevice;
    usb_device.init    = RT_NULL;
    usb_device.open    = RT_NULL;
    usb_device.close   = RT_NULL;
    usb_device.read    = usb_read;
    usb_device.write   = usb_write;
    usb_device.control = usb_control;
	
	/* register to device manager */
    res = rt_device_register(&usb_device , USB_DEVICE_NAME, RT_DEVICE_FLAG_RDWR);
	
	return res == RT_EOK ? 0 : 1;
}
