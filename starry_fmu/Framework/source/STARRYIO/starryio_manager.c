/*
 * File      : starryio_manager.c
 *
 * Change Logs:
 * Date           Author       Notes
 * 2017-02-13     zoujiachi   	the first version
 */
 
#include <rthw.h>
#include <rtdevice.h>
#include <rtthread.h>
#include "starryio_manager.h"
#include "starryio_protocol.h"
#include "console.h"
#include "uMCN.h"

//#define PX4IO_DEBUG

//static rt_device_t debug_dev;
struct rt_semaphore starryio_dbg_rx_sem;
struct rt_semaphore starryio_rx_pack_sem;
struct rt_semaphore starryio_tx_pack_sem;
static rt_device_t serial_dev;
#define MSEC_TO_TICKS(ms) ((ms) * RT_TICK_PER_SECOND / 1000)
#define STARRYIO_DTX_TIMEOUT MSEC_TO_TICKS(30)

//static ringbuffer* rb;
static char* TAG = "STARRYIO Manager";

uint8_t ppm_send_freq = 20;	/* sending frequemcy of ppm signal, HZ */

MCN_DECLARE(RC_STATUS);

rt_err_t starryio_serial_tx_done(rt_device_t dev, void * buffer)
{
	rt_sem_release(&starryio_tx_pack_sem);
}

static rt_err_t send_char(uint8_t c)
{
	rt_size_t bytes;
	
	bytes = rt_device_write(serial_dev, 0, (const void *)&c, 1);
	if(bytes != 1)
		return -RT_ERROR;

	if (rt_sem_take(&starryio_tx_pack_sem, STARRYIO_DTX_TIMEOUT) != RT_EOK) {
		Console.print("starryio tx timeout\n");
		return -RT_EIO;
	}
	
	return RT_EOK;
}

static rt_err_t send(uint8_t* buff, uint32_t size)
{
	rt_size_t bytes;
	
	bytes = rt_device_write(serial_dev, 0, (const void *)buff, size);
	if(bytes != size)
		return -RT_ERROR;

	if (rt_sem_take(&starryio_tx_pack_sem, STARRYIO_DTX_TIMEOUT) != RT_EOK) {
		Console.print("starryio tx timeout\n");
		return -RT_EIO;
	}
	
	return RT_EOK;
}

//this function will be callback on rt_hw_serial_isr()
static rt_err_t starryio_serial_rx_ind(rt_device_t dev, rt_size_t size)
{	
	rt_sem_release(&starryio_rx_pack_sem);

    return RT_EOK;
}

//static rt_err_t starryio_debug_rx_ind(rt_device_t dev, rt_size_t size)
//{
//	rt_sem_release(&starryio_dbg_rx_sem);

//    return RT_EOK;
//}

rt_device_t starryio_get_device(void)
{
	return serial_dev;
}

void px4io_reset_rx_ind(void)
{
	rt_device_set_rx_indicate(serial_dev, starryio_serial_rx_ind);
}

uint8_t send_package(uint8_t cmd, uint8_t* data, uint16_t len)
{
	Package_Def pack;
	SendPackage_Def send_pack;
	
	if(!make_package(data , cmd , len , &pack)){
		return 0;
	}
	
	package2sendpack(pack , &send_pack);
	send(send_pack.send_buff, send_pack.buff_size);
	
	free_pack(&pack);
	free_sendpack(&send_pack);
	
	return 1;
}

rt_err_t set_ppm_freq(uint8_t freq)
{
	rt_err_t ret;
	
	ret = send_package(CMD_CONFIG_CHANNEL, &freq, 1) ? RT_EOK : RT_ERROR;
	
	return ret;
}

rt_err_t request_reboot(void)
{
	rt_err_t ret;
	
	ret = send_package(CMD_REBOOT, NULL, 0) ? RT_EOK : RT_ERROR;
	
	return ret;
}

rt_err_t reply_sync(void)
{
	send_package(ACK_SYNC, NULL, 0);
	
	/* after sync, we should config starryio */
	send_package(CMD_CONFIG_CHANNEL, &ppm_send_freq, 1);
	
	return RT_EOK;
}

void starryio_entry(void *parameter)
{	
#ifdef PX4IO_DEBUG
	rt_size_t bytes;
	uint8_t dbg_buffer[51];
	debug_dev = rt_device_find("uart1");	/* usart1 for debug */
	
	if(debug_dev == RT_NULL)
    {
        printf("debug device uart1 not found!\r\n");
    }
	
	rt_sem_init(&starryio_dbg_rx_sem, "iodbg", 0, 0);
	
	rt_device_set_rx_indicate(debug_dev, starryio_debug_rx_ind);
	rt_device_open(debug_dev, RT_DEVICE_OFLAG_RDONLY | RT_DEVICE_FLAG_INT_RX);
#endif
	
	serial_dev = rt_device_find("uart6");
	
	if(serial_dev == RT_NULL)
    {
        Console.e(TAG, "serial device %s not found!\r\n", "uart6");
    }
	
	rt_sem_init(&starryio_rx_pack_sem, "rxpack", 0, 0);
	rt_sem_init(&starryio_tx_pack_sem, "txpack", 0, RT_IPC_FLAG_FIFO);
	
	rt_device_set_rx_indicate(serial_dev, starryio_serial_rx_ind);
	rt_device_set_tx_complete(serial_dev, starryio_serial_tx_done);
	rt_device_open(serial_dev, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_DMA_RX | RT_DEVICE_FLAG_DMA_TX);
	
	int mcn_res;
	mcn_res = mcn_advertise(MCN_ID(RC_STATUS));
	if(mcn_res != 0){
		Console.e(TAG, "err:%d, RC_STATUS advertise fail!\n", mcn_res);
	}
	
	while(1){
#ifdef PX4IO_DEBUG
		if (rt_sem_take(&starryio_dbg_rx_sem, 1) == RT_EOK){
			uint8_t ch;
			while(rt_device_read(debug_dev , 0 , &ch , 1)){
				fputc((int)ch, NULL);
			}
		}
#endif
	
		if (rt_sem_take(&starryio_rx_pack_sem, 20) == RT_EOK){
			uint8_t ch;
			while(rt_device_read(serial_dev , 0 , &ch , 1)){
				if(wait_complete_pack(ch))
					process_recv_pack();
			}
		}
	}
}
