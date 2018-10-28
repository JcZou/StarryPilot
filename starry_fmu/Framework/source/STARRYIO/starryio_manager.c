/*****************************************************************************
Copyright (c) 2018, StarryPilot Development Team. All rights reserved.

Author: Jiachi Zou, weety

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of StarryPilot nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/
 
#include <rthw.h>
#include <rtdevice.h>
#include <rtthread.h>
#include "starryio_manager.h"
#include "starryio_protocol.h"
#include "console.h"
#include "uMCN.h"

//#define PX4IO_DEBUG
#define MSEC_TO_TICKS(ms) ((ms) * RT_TICK_PER_SECOND / 1000)
#define STARRYIO_DTX_TIMEOUT MSEC_TO_TICKS(30)
#define MAX_IO_BUFFER_SIZE				10

//static rt_device_t debug_dev;
struct rt_semaphore starryio_dbg_rx_sem;
struct rt_semaphore starryio_rx_pack_sem;
struct rt_semaphore starryio_tx_pack_sem;
static rt_device_t serial_dev;
static bool _io_comm_suspend = false;
static Package_Def _io_pack_buffer[MAX_IO_BUFFER_SIZE];
static uint16_t _buffer_head = 0;
static uint16_t _buffer_tail = 0;
static uint8_t _send_pack_buff[MAX_PACKAGE_SIZE];

static char* TAG = "STARRYIO Manager";

uint8_t ppm_send_freq = 20;	/* sending frequemcy of ppm signal, HZ */

MCN_DECLARE(RC_STATUS);

rt_err_t _starryio_serial_tx_done(rt_device_t dev, void * buffer)
{
	return rt_sem_release(&starryio_tx_pack_sem);
}

static rt_err_t _send_char(uint8_t c)
{
	rt_size_t bytes;
	
	bytes = rt_device_write(serial_dev, 0, (const void *)&c, 1);
	if(bytes != 1){
		Console.print("starryio tx err\n");
		return -RT_ERROR;
	}
	
	return RT_EOK;
}

static rt_err_t _send(uint8_t* buff, uint32_t size)
{
	rt_size_t bytes;
		
	bytes = rt_device_write(serial_dev, 0, (const void *)buff, size);
	if(bytes != size){
		Console.print("starryio tx err\n");
		return -RT_ERROR;
	}
	
	return RT_EOK;
}

//this function will be callback on rt_hw_serial_isr()
static rt_err_t _starryio_serial_rx_ind(rt_device_t dev, rt_size_t size)
{	
	rt_sem_release(&starryio_rx_pack_sem);

    return RT_EOK;
}

uint8_t _sendout_buffer_package(void)
{
	SendPackage_Def send_pack;
	send_pack.send_buff = _send_pack_buff;
	
	package2sendpack_static(_io_pack_buffer[_buffer_tail] , &send_pack);
	_send(send_pack.send_buff, send_pack.buff_size);
	
	free_pack(&_io_pack_buffer[_buffer_tail]);
	OS_ENTER_CRITICAL;
	_buffer_tail = (_buffer_tail+1) % MAX_IO_BUFFER_SIZE;
	OS_EXIT_CRITICAL;
	
	return 1;
}

rt_device_t starryio_get_device(void)
{
	return serial_dev;
}

void starryio_suspend_comm(bool enable)
{
	_io_comm_suspend = enable;
}

void px4io_reset_rx_ind(void)
{
	rt_device_set_rx_indicate(serial_dev, _starryio_serial_rx_ind);
}

uint8_t send_package(uint8_t cmd, uint8_t* data, uint16_t len)
{
	Package_Def pack;
	SendPackage_Def send_pack;
	uint8_t res = 1;
	
	if(_io_comm_suspend){
		// suspend io package sending
		return 0;
	}
	
	if( (_buffer_head+1) % MAX_IO_BUFFER_SIZE == _buffer_tail ){
		Console.print("io package overflow\n");
		res = 0;
	}else{
		if(!make_package(data , cmd , len , &pack)){
			Console.print("make pack fail\n");
			return 0;
		}
		
		OS_ENTER_CRITICAL;
		_io_pack_buffer[_buffer_head] = pack;
		_buffer_head = (_buffer_head + 1) % MAX_IO_BUFFER_SIZE;
		OS_EXIT_CRITICAL;
	}
	
	return res;
}

rt_err_t request_reboot(void)
{
	rt_err_t ret;
	
	ret = send_package(CMD_REBOOT, NULL, 0) ? RT_EOK : RT_ERROR;
	
	return ret;
}

rt_err_t reply_io_sync_package(void)
{
	send_package(ACK_SYNC, NULL, 0);
	
	/* after sync, we should config starryio ppm frequency */
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
	rt_sem_init(&starryio_tx_pack_sem, "txpack", 1, RT_IPC_FLAG_FIFO);
	
	rt_device_set_rx_indicate(serial_dev, _starryio_serial_rx_ind);
	rt_device_set_tx_complete(serial_dev, _starryio_serial_tx_done);
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
		if(_buffer_head != _buffer_tail){
			if (rt_sem_take(&starryio_tx_pack_sem, 0) == RT_EOK){
				// DMA is accessible
				_sendout_buffer_package();
			}
		}
	
		if (rt_sem_take(&starryio_rx_pack_sem, 0) == RT_EOK){
			uint8_t ch;
			while(rt_device_read(serial_dev , 0 , &ch , 1)){
				if(wait_complete_pack(ch)){
					process_recv_pack();
				}
			}
		}
		
		rt_thread_delay(1);
	}
}
