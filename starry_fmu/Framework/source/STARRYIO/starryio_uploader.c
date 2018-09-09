/*
 * File      : starryio_uploader.c
 *
 * Change Logs:
 * Date           Author       Notes
 * 2017-02-04     zoujiachi   	the first version
 */
 
#include <rtdevice.h>
#include <rtthread.h>
#include "shell.h"
#include <stdlib.h>
#include "ringbuffer.h"
#include "console.h"
#include "starryio_uploader.h"
#include "delay.h"
#include "starryio_manager.h"
#include "ff.h"

static const uint32_t crc32_tab[] = {
	0x00000000, 0x77073096, 0xee0e612c, 0x990951ba, 0x076dc419, 0x706af48f, 0xe963a535, 0x9e6495a3,
	0x0edb8832, 0x79dcb8a4, 0xe0d5e91e, 0x97d2d988, 0x09b64c2b, 0x7eb17cbd, 0xe7b82d07, 0x90bf1d91,
	0x1db71064, 0x6ab020f2, 0xf3b97148, 0x84be41de, 0x1adad47d, 0x6ddde4eb, 0xf4d4b551, 0x83d385c7,
	0x136c9856, 0x646ba8c0, 0xfd62f97a, 0x8a65c9ec, 0x14015c4f, 0x63066cd9, 0xfa0f3d63, 0x8d080df5,
	0x3b6e20c8, 0x4c69105e, 0xd56041e4, 0xa2677172, 0x3c03e4d1, 0x4b04d447, 0xd20d85fd, 0xa50ab56b,
	0x35b5a8fa, 0x42b2986c, 0xdbbbc9d6, 0xacbcf940, 0x32d86ce3, 0x45df5c75, 0xdcd60dcf, 0xabd13d59,
	0x26d930ac, 0x51de003a, 0xc8d75180, 0xbfd06116, 0x21b4f4b5, 0x56b3c423, 0xcfba9599, 0xb8bda50f,
	0x2802b89e, 0x5f058808, 0xc60cd9b2, 0xb10be924, 0x2f6f7c87, 0x58684c11, 0xc1611dab, 0xb6662d3d,
	0x76dc4190, 0x01db7106, 0x98d220bc, 0xefd5102a, 0x71b18589, 0x06b6b51f, 0x9fbfe4a5, 0xe8b8d433,
	0x7807c9a2, 0x0f00f934, 0x9609a88e, 0xe10e9818, 0x7f6a0dbb, 0x086d3d2d, 0x91646c97, 0xe6635c01,
	0x6b6b51f4, 0x1c6c6162, 0x856530d8, 0xf262004e, 0x6c0695ed, 0x1b01a57b, 0x8208f4c1, 0xf50fc457,
	0x65b0d9c6, 0x12b7e950, 0x8bbeb8ea, 0xfcb9887c, 0x62dd1ddf, 0x15da2d49, 0x8cd37cf3, 0xfbd44c65,
	0x4db26158, 0x3ab551ce, 0xa3bc0074, 0xd4bb30e2, 0x4adfa541, 0x3dd895d7, 0xa4d1c46d, 0xd3d6f4fb,
	0x4369e96a, 0x346ed9fc, 0xad678846, 0xda60b8d0, 0x44042d73, 0x33031de5, 0xaa0a4c5f, 0xdd0d7cc9,
	0x5005713c, 0x270241aa, 0xbe0b1010, 0xc90c2086, 0x5768b525, 0x206f85b3, 0xb966d409, 0xce61e49f,
	0x5edef90e, 0x29d9c998, 0xb0d09822, 0xc7d7a8b4, 0x59b33d17, 0x2eb40d81, 0xb7bd5c3b, 0xc0ba6cad,
	0xedb88320, 0x9abfb3b6, 0x03b6e20c, 0x74b1d29a, 0xead54739, 0x9dd277af, 0x04db2615, 0x73dc1683,
	0xe3630b12, 0x94643b84, 0x0d6d6a3e, 0x7a6a5aa8, 0xe40ecf0b, 0x9309ff9d, 0x0a00ae27, 0x7d079eb1,
	0xf00f9344, 0x8708a3d2, 0x1e01f268, 0x6906c2fe, 0xf762575d, 0x806567cb, 0x196c3671, 0x6e6b06e7,
	0xfed41b76, 0x89d32be0, 0x10da7a5a, 0x67dd4acc, 0xf9b9df6f, 0x8ebeeff9, 0x17b7be43, 0x60b08ed5,
	0xd6d6a3e8, 0xa1d1937e, 0x38d8c2c4, 0x4fdff252, 0xd1bb67f1, 0xa6bc5767, 0x3fb506dd, 0x48b2364b,
	0xd80d2bda, 0xaf0a1b4c, 0x36034af6, 0x41047a60, 0xdf60efc3, 0xa867df55, 0x316e8eef, 0x4669be79,
	0xcb61b38c, 0xbc66831a, 0x256fd2a0, 0x5268e236, 0xcc0c7795, 0xbb0b4703, 0x220216b9, 0x5505262f,
	0xc5ba3bbe, 0xb2bd0b28, 0x2bb45a92, 0x5cb36a04, 0xc2d7ffa7, 0xb5d0cf31, 0x2cd99e8b, 0x5bdeae1d,
	0x9b64c2b0, 0xec63f226, 0x756aa39c, 0x026d930a, 0x9c0906a9, 0xeb0e363f, 0x72076785, 0x05005713,
	0x95bf4a82, 0xe2b87a14, 0x7bb12bae, 0x0cb61b38, 0x92d28e9b, 0xe5d5be0d, 0x7cdcefb7, 0x0bdbdf21,
	0x86d3d2d4, 0xf1d4e242, 0x68ddb3f8, 0x1fda836e, 0x81be16cd, 0xf6b9265b, 0x6fb077e1, 0x18b74777,
	0x88085ae6, 0xff0f6a70, 0x66063bca, 0x11010b5c, 0x8f659eff, 0xf862ae69, 0x616bffd3, 0x166ccf45,
	0xa00ae278, 0xd70dd2ee, 0x4e048354, 0x3903b3c2, 0xa7672661, 0xd06016f7, 0x4969474d, 0x3e6e77db,
	0xaed16a4a, 0xd9d65adc, 0x40df0b66, 0x37d83bf0, 0xa9bcae53, 0xdebb9ec5, 0x47b2cf7f, 0x30b5ffe9,
	0xbdbdf21c, 0xcabac28a, 0x53b39330, 0x24b4a3a6, 0xbad03605, 0xcdd70693, 0x54de5729, 0x23d967bf,
	0xb3667a2e, 0xc4614ab8, 0x5d681b02, 0x2a6f2b94, 0xb40bbe37, 0xc30c8ea1, 0x5a05df1b, 0x2d02ef8d
};

enum {

	PROTO_NOP				= 0x00,
	PROTO_OK				= 0x10,
	PROTO_FAILED			= 0x11,
	PROTO_INSYNC			= 0x12,
	PROTO_INVALID			= 0x13,
	PROTO_BAD_SILICON_REV  	= 0x14,
	PROTO_EOC				= 0x20,
	PROTO_GET_SYNC			= 0x21,
	PROTO_GET_DEVICE		= 0x22,
	PROTO_CHIP_ERASE		= 0x23,
	PROTO_CHIP_VERIFY		= 0x24,
	PROTO_PROG_MULTI		= 0x27,
	PROTO_READ_MULTI		= 0x28,
	PROTO_GET_CRC			= 0x29,
	PROTO_GET_OTP			= 0x2a,
	PROTO_GET_SN			= 0x2b,
	PROTO_GET_CHIP			= 0x2c,
	PROTO_SET_DELAY			= 0x2d,
	PROTO_GET_CHIP_DES		= 0x2e,
	PROTO_REBOOT			= 0x30,

	INFO_BL_REV			= 1,		/**< bootloader protocol revision */
	BL_REV				= 5,		/**< supported bootloader protocol  */
	INFO_BOARD_ID		= 2,		/**< board type */
	INFO_BOARD_REV		= 3,		/**< board revision */
	INFO_FLASH_SIZE		= 4,		/**< max firmware size in bytes */

	PROG_MULTI_MAX		= 248,		/**< protocol max is 255, must be multiple of 4 */

};

static rt_device_t _dev;
static ringbuffer* rb;

uint32_t crc32part(const uint8_t *src, size_t len, uint32_t crc32val)
{
	size_t i;

	for (i = 0;  i < len;  i++) {
		crc32val = crc32_tab[(crc32val ^ src[i]) & 0xff] ^ (crc32val >> 8);
	}

	return crc32val;
}

static rt_err_t recv_byte_with_timeout(uint8_t *c, unsigned timeout)
{
	uint32_t time = time_nowMs();
	
	while(time_nowMs()-time < timeout){
		if(ringbuffer_getlen(rb)){
			*c = ringbuffer_getc(rb);
			return RT_EOK;
		}
	}
	
	return RT_ETIMEOUT;
}

static rt_err_t recv_bytes(uint8_t *buff, unsigned count)
{
	int ret = RT_EOK;

	while (count--) {
		ret = recv_byte_with_timeout(buff++, 5000);

		if (ret != RT_EOK) {
			break;
		}
	}

	return ret;
}

static rt_err_t send_char(uint8_t c)
{
	rt_size_t bytes;
	
	bytes = rt_device_write(_dev, 0, (const void *)&c, 1);
	if(bytes != 1)
		return RT_ERROR;
	
	return RT_EOK;
}

static rt_err_t send(uint8_t* buff, uint32_t size)
{
	rt_size_t bytes;
	
	bytes = rt_device_write(_dev, 0, (const void *)buff, size);
	if(bytes != size)
		return RT_ERROR;
	
	return RT_EOK;
}

static rt_err_t get_sync(unsigned timeout)
{
	uint8_t c[2];
	int ret;

	ret = recv_byte_with_timeout(c, timeout);

	if (ret != RT_EOK) {
		return ret;
	}

	ret = recv_byte_with_timeout(c + 1, timeout);

	if (ret != RT_EOK) {
		return ret;
	}

	if ((c[0] != PROTO_INSYNC) || (c[1] != PROTO_OK)) {
		//Console.print("bad sync 0x%02x,0x%02x\r\n", c[0], c[1]);
		return RT_ERROR;
	}

	return RT_EOK;
}


static rt_err_t sync(void)
{
	/* complete any pending program operation */
//	for (unsigned i = 0; i < (PROG_MULTI_MAX + 6); i++) {
//		send_char(0);
//	}
	
	send_char(PROTO_GET_SYNC);
	send_char(PROTO_EOC);
	
	return get_sync(50);
}

static rt_err_t get_info(int param, uint8_t* val, uint32_t size)
{
	int ret;

	send_char(PROTO_GET_DEVICE);
	send_char(param);
	send_char(PROTO_EOC);

	ret = recv_bytes(val, size);

	if (ret != RT_EOK) {
		return ret;
	}

	return get_sync(100);
}

static rt_err_t erase(void)
{
	send_char(PROTO_CHIP_ERASE);
	send_char(PROTO_EOC);
	return get_sync(10000);		/* allow 10s timeout */
}

static rt_err_t program_fs(char* file_name)
{
	size_t count = 0;
	uint8_t* file_buf;
	rt_err_t ret;
	uint32_t prog_cnt, prog_offset;
	uint32_t sum = 0;
	uint32_t fw_size_remote;
	uint8_t fill_blank = 0xff;
	uint32_t crc = 0;
	
	FIL fp;
	FRESULT res = f_open(&fp, file_name, FA_OPEN_EXISTING | FA_READ);
	if(res != FR_OK){
		Console.print("can not open the file:%s err:%d\n", file_name, res);
		return RT_ERROR;
	}
	
	prog_cnt = fp.fsize/PROG_MULTI_MAX;
	prog_offset = fp.fsize%PROG_MULTI_MAX;
	
	file_buf = (uint8_t*)rt_malloc(PROG_MULTI_MAX);
	
	if(file_buf == NULL){
		Console.print("malloc fail\r\n");
		return RT_ERROR;
	}
	
	UINT br; 
	
	Console.print("erase...\r\n");
	ret = erase();
	Console.print("program...\r\n");
	
	for (uint32_t i = 0 ; i < prog_cnt ; i++) {

		res = f_read(&fp, file_buf, PROG_MULTI_MAX, &br);
		if(br != PROG_MULTI_MAX){
			Console.print("read fail err. size:%d br:%d\n", PROG_MULTI_MAX, br);
			rt_free(file_buf);
			return RT_ERROR;
		}
	
		/* calculate crc32 sum */
		sum = crc32part((uint8_t *)file_buf, PROG_MULTI_MAX, sum);
		
		send_char(PROTO_PROG_MULTI);
		send_char(PROG_MULTI_MAX);
		send(file_buf, PROG_MULTI_MAX);
		send_char(PROTO_EOC);

		ret = get_sync(1000);
		
		if (ret != RT_EOK) {
			Console.print("program fail %ld\r\n", ret);
			break;
		}
	}
	
	if (prog_offset) {
		res = f_read(&fp, file_buf, prog_offset, &br);
		if(br != prog_offset){
			Console.print("read fail err. size:%d br:%d\n", prog_offset, br);
			rt_free(file_buf);
			return RT_ERROR;
		}
		
		/* calculate crc32 sum */
		sum = crc32part((uint8_t *)file_buf, prog_offset, sum);
		
		send_char(PROTO_PROG_MULTI);
		send_char(prog_offset);
		send(file_buf, prog_offset);
		send_char(PROTO_EOC);

		ret = get_sync(1000);
		
		if (ret != RT_EOK) {
			Console.print("program fail %ld\r\n", ret);
		}
	}
	
	/* free file_buf */
	rt_free(file_buf);
	
	ret = get_info(INFO_FLASH_SIZE, (uint8_t*)&fw_size_remote, sizeof(fw_size_remote));
	send_char(PROTO_EOC);

	if (ret != RT_EOK) {
		Console.print("could not read firmware size\r\n");
		return ret;
	}
	
	/* fill the rest with 0xff */
	count = fp.fsize;
	while (count < fw_size_remote) {
		sum = crc32part(&fill_blank, sizeof(fill_blank), sum);
		count += sizeof(fill_blank);
	}
	
	/* request CRC from IO */
	send_char(PROTO_GET_CRC);
	send_char(PROTO_EOC);

	ret = recv_bytes((uint8_t *)(&crc), sizeof(crc));
	
	if (ret != RT_EOK) {
		Console.print("did not receive CRC checksum\r\n");
		return ret;
	}

	/* compare the CRC sum from the IO with the one calculated */
	if (sum != crc) {
		Console.print("CRC wrong: received: %x, expected: %x\r\n", crc, sum);
		return RT_ERROR;
	}else{
		Console.print("CRC check ok, received: %x, expected: %x\r\n", crc, sum);
	}
	
	return RT_EOK;
}

static rt_err_t program_serial(size_t fw_size)
{
	size_t count = 0;
	uint8_t* file_buf;
	rt_err_t ret;
	uint32_t prog_cnt, prog_offset;
	uint32_t sum = 0;
	uint32_t fw_size_remote;
	uint8_t fill_blank = 0xff;
	uint32_t crc = 0;
	
	struct finsh_shell* shell = finsh_get_shell();
	
	prog_cnt = fw_size/PROG_MULTI_MAX;
	prog_offset = fw_size%PROG_MULTI_MAX;
	
	/* read file from serial */
	file_buf = (uint8_t*)rt_malloc(fw_size);
	
	if(file_buf == NULL){
		Console.print("malloc fail\r\n");
		return RT_ERROR;
	}
	
	while(count < fw_size){
		count += rt_device_read(shell->device, 0, &file_buf[count], fw_size-count);
	}
	
	Console.print("program...\r\n");
	
	for (uint32_t i = 0 ; i < prog_cnt ; i++) {
		
//		Console.print("i:%d\r\n", i);
//		
//		while (count < PROG_MULTI_MAX) {
//			count += rt_device_read(shell->device, 0, &file_buf[count], PROG_MULTI_MAX-count);
//		}
		
		//Console.print("cnt:%d\r\n", count);
		
		/* calculate crc32 sum */
		sum = crc32part((uint8_t *)&file_buf[i*PROG_MULTI_MAX], PROG_MULTI_MAX, sum);
		
		send_char(PROTO_PROG_MULTI);
		send_char(PROG_MULTI_MAX);
		send(&file_buf[i*PROG_MULTI_MAX], PROG_MULTI_MAX);
		send_char(PROTO_EOC);

		ret = get_sync(1000);
		
		if (ret != RT_EOK) {
			Console.print("program fail %ld\r\n", ret);
			break;
		}
	}
	
	if (prog_offset) {
		
//		count = 0;
//		
//		while (count < prog_offset) {
//			count += rt_device_read(shell->device, 0, &file_buf[count], PROG_MULTI_MAX-count);
//		}
		
		/* calculate crc32 sum */
		sum = crc32part((uint8_t *)&file_buf[prog_cnt*PROG_MULTI_MAX], prog_offset, sum);
		
		send_char(PROTO_PROG_MULTI);
		send_char(prog_offset);
		send(&file_buf[prog_cnt*PROG_MULTI_MAX], prog_offset);
		send_char(PROTO_EOC);

		ret = get_sync(1000);
		
		if (ret != RT_EOK) {
			Console.print("program fail %ld\r\n", ret);
		}
	}
	
	/* free file_buf */
	rt_free(file_buf);
	
	ret = get_info(INFO_FLASH_SIZE, (uint8_t*)&fw_size_remote, sizeof(fw_size_remote));
	send_char(PROTO_EOC);

	if (ret != RT_EOK) {
		Console.print("could not read firmware size\r\n");
		return ret;
	}
	
	/* fill the rest with 0xff */
	count = fw_size;
	while (count < fw_size_remote) {
		sum = crc32part(&fill_blank, sizeof(fill_blank), sum);
		count += sizeof(fill_blank);
	}
	
	/* request CRC from IO */
	send_char(PROTO_GET_CRC);
	send_char(PROTO_EOC);

	ret = recv_bytes((uint8_t *)(&crc), sizeof(crc));
	
	if (ret != RT_EOK) {
		Console.print("did not receive CRC checksum\r\n");
		return ret;
	}

	/* compare the CRC sum from the IO with the one calculated */
	if (sum != crc) {
		Console.print("CRC wrong: received: %x, expected: %x\r\n", crc, sum);
		return RT_ERROR;
	}else{
		Console.print("CRC check ok, received: %x, expected: %x\r\n", crc, sum);
	}

	return RT_EOK;
}

static rt_err_t reboot()
{
	send_char(PROTO_REBOOT);
	time_waitUs(100 * 1000);
	send_char(PROTO_EOC);

	return RT_EOK;
}

//this function will be callback on rt_hw_serial_isr()
static rt_err_t uploader_serial_rx_ind(rt_device_t dev, rt_size_t size)
{
	rt_size_t bytes;
	uint8_t ch[RT_SERIAL_RB_BUFSZ];
	
	bytes = rt_device_read(_dev , 0 , ch , size);
	
	if(bytes){
		for(uint32_t i = 0 ; i<bytes ; i++){
			if(!ringbuffer_putc(rb, ch[i]))
				Console.print("ringbuffer full\r\n");
		}
	}else{
		Console.print("uploader listen err:%ld\r\n" , bytes);
	}

    return RT_EOK;
}

void starryio_upload(void)
{
	uint32_t bl_rev;
	rt_err_t ret;
	uint8_t file_size[32] = {0};
	uint32_t time = time_nowMs();
	
	struct finsh_shell* shell = finsh_get_shell();
	
	Console.print("starryio uploader, wait sync signal...\r\n");
	
	if(uploader_init() != RT_EOK){
		uploader_deinit();
		return ;
	}
	
	request_reboot();	/* reboot starryio to let device enter bootloader */
	
	ringbuffer_flush(rb);	/*flush ring buffer*/
	
	while(time_nowMs() - time < 10000) {
		if(sync() == RT_EOK){
			char ch;
			Console.print("sync success\r\n");
			get_info(INFO_BL_REV, (uint8_t*)&bl_rev, sizeof(bl_rev));
			Console.print("found bootloader revision: %d\r\n", bl_rev);
			
			Console.print("please choose download method:\n1:file system  2:serial  3:cancel\n");
			ch = shell_wait_ch();
			
			if(ch == '1'){
				ret = program_fs("starryio.bin");
			}else if(ch == '2'){
				Console.print(".bin file size:");
			
				for(uint32_t i = 0 ; ; i++){
					//rt_sem_take(&shell->rx_sem, RT_WAITING_FOREVER);
					//rt_device_read(shell->device, 0, &file_size[i], 1);
					file_size[i] = shell_wait_ch();
					Console.print("%c", file_size[i]);
					if(file_size[i] < '0' || file_size[i] > '9'){
						file_size[i] = '\0';
						break;
					}
				}
				
				size_t f_size = atoi((char*)file_size);
				Console.print("file size is %d, upload now? Y/N\r\n", f_size);
				
				ch = shell_wait_ch();
				if(ch == 'Y' || ch == 'y'){
					Console.print("erase...\r\n");
					ret = erase();
					ret = program_serial(f_size);
				}else{
					return ;
				}
			}else{
				return ;
			}
			
			if (ret != RT_EOK) {
				Console.print("program failed!\r\n");
				return ;
			}else{
				Console.print("program success!\r\n");
				
				ret = reboot();
			
				if (ret != RT_EOK) {
					Console.print("reboot failed\r\n");
					rt_device_close(_dev);
				}else{
					Console.print("update complete\r\n");
				}
			}
			
			uploader_deinit();
			return;
		}
	}
}

rt_err_t uploader_init(void)
{	
	_dev = starryio_get_device();
	
	if(_dev == RT_NULL)
    {
        Console.print("serial device usart6 not found!\r\n");
        return RT_EEMPTY;
    }
	
	rb = ringbuffer_create(256);
	rt_device_set_rx_indicate(_dev, uploader_serial_rx_ind);

	if(rb == NULL){
		Console.print("create ringbuffer err\r\n");
		return RT_ERROR;
	}
	
	return RT_EOK;
}

rt_err_t uploader_deinit(void)
{
	ringbuffer_delete(rb);
	px4io_reset_rx_ind();
	_dev = NULL;
	
	return RT_EOK;
}

