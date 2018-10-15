/*
 * File      : starryio_protocol.c
 *
 * Change Logs:
 * Date           Author       Notes
 * 2017-02-18     zoujiachi   	the first version
 */
 
//#include <rthw.h>
//#include <rtdevice.h>
#include <rtthread.h>
#include <stdlib.h>
#include "rc.h"
#include "starryio_protocol.h"
#include "starryio_manager.h"
#include "console.h"
#include "delay.h"
#include "sensor_manager.h"
#include "pwm_io.h"

static uint8_t pack_buff[MAX_PACKAGE_SIZE];
static uint8_t pack_usr_buff[MAX_PACKAGE_SIZE];
Package_Def Recv_Package;
static char* TAG = "px4io";

#define min(a, b) ((a) > (b) ? (b) : (a))

#ifdef PROTOCOL_SERVER
static uint8_t s_head = 0x5A;
static uint8_t r_head = 0x5B;
#else
static uint8_t s_head = 0x5B;
static uint8_t r_head = 0x5A;
#endif

void _fill_head(Package_Def* package)
{
	package->head[0] = 0xFA;
	package->head[1] = s_head;
}

void _fill_endflag(Package_Def* package)
{
	package->endflag = 0xFC;
}

uint16_t _calc_checksum(const Package_Def* package)
{
	uint16_t cheksum = 0;
	uint16_t i;
	
	cheksum = package->head[0]+package->head[1]+((package->len&0xFF)+((package->len>>8)&0xFF))+package->cmd;
	
	for(i=0 ; i<package->len ; i++){
		cheksum += (uint8_t)package->usr_data[i];
	}
	
	return cheksum;
}

void free_pack(Package_Def* package)
{
	if(package->len > 0 && package->usr_data != NULL){
		rt_free(package->usr_data);
	}
}

void free_sendpack(SendPackage_Def* send_pack)
{
	if(send_pack->send_buff != NULL){
		rt_free(send_pack->send_buff);
	}
}

//Noticed: need to adjust heap size to suit the package size
uint8_t make_package(uint8_t* data , char cmd , uint16_t len , Package_Def* package)
{
	uint16_t i;
	
	_fill_head(package);
	
	package->len = len;
	package->cmd = cmd;

	//copy data
	if(len>0){
		package->usr_data = rt_malloc(len);
		if(package->usr_data == NULL){
			Console.e(TAG, "Make_Package Heap is not enough:%d\n" , len);
			return 0;
		}
	}
	for(i=0 ; i<len ; i++){
		package->usr_data[i] = data[i];
	}
	
	//calculate checksum
	package->checksum = _calc_checksum(package);
	
	_fill_endflag(package);

	return 1;
}

uint8_t package2sendpack(const Package_Def package , SendPackage_Def* send_pack)
{
	int i;
	send_pack->buff_size = package.len+PACK_SIZE_EXCEPT_DATA;
	//When sending finish, should involke FreeSendPack() to free memory.
	send_pack->send_buff = rt_malloc(send_pack->buff_size);
	if(send_pack->send_buff == NULL){
		Console.e(TAG, "Package2SendPack Heap is not enough:%d\n" , send_pack->buff_size);
		return 0;
	}
	
	send_pack->send_buff[0] = package.head[0];
	send_pack->send_buff[1] = package.head[1];
	//little-endian 
	*((uint16_t*)&send_pack->send_buff[2]) = package.len;
	send_pack->send_buff[4] = package.cmd;
	for(i=0 ; i<package.len ; i++){
		send_pack->send_buff[5+i] = package.usr_data[i];
	}
	*((uint16_t*)&send_pack->send_buff[5+package.len]) = package.checksum;
	send_pack->send_buff[7+package.len] = package.endflag;
	
	return 1;
}

uint8_t parse_package(const uint8_t* data , Package_Def* package)
{
	static Package_Def tempPack;
	uint16_t correct_checksum;
	int i;
	
	if(data == NULL){
		Console.e(TAG, "ParsePackage data NULL\n");
		return 0x01;
	}
	
	tempPack.head[0] = data[0];
	tempPack.head[1] = data[1];
	tempPack.len = *((uint16_t*)&data[2]);
	tempPack.cmd = data[4];
	
	if(tempPack.head[0] != 0xFA || tempPack.head[1] != r_head){
		Console.e(TAG, "ParsePackage pack head illegal\n");
		return 0x02;
	}
	
	if(data[7+tempPack.len] != 0xFC){
		Console.e(TAG, "ParsePackage tail illegal\n");
		return 0x03;
	}
	
	if(tempPack.len > 0){
		tempPack.usr_data = pack_usr_buff;
		
		for(i = 0 ; i<min(sizeof(pack_usr_buff), tempPack.len) ; i++){
			tempPack.usr_data[i] = (uint8_t)data[5+i];
		}
	}

	tempPack.checksum = *((uint16_t*)&data[5+tempPack.len]);
	tempPack.endflag = data[7+tempPack.len];

	correct_checksum = _calc_checksum(&tempPack);
	if(tempPack.checksum != correct_checksum){
		Console.e(TAG, "checksum not correct %x\n" , correct_checksum);
		return 0x04;
	}
	
	if(tempPack.len > 0){
		package->usr_data = tempPack.usr_data;
	}

	package->head[0] = tempPack.head[0];
	package->head[1] = tempPack.head[1];
	package->len = tempPack.len;
	package->cmd = tempPack.cmd;

	package->checksum = tempPack.checksum;
	package->endflag = tempPack.endflag;
	
	return 0;
}

uint8_t wait_complete_pack(uint8_t c)
{
	#define WAIT_HEAD_1	0x00
	#define WAIT_HEAD_2	0x01
	#define WAIT_LEN_1	0x02
	#define WAIT_LEN_2	0x03
	#define WAIT_CMD	0x04
	#define WAIT_DATA	0x05
	#define WAIT_CS_1	0x06
	#define WAIT_CS_2	0x07
	#define WAIT_0xFC	0x08
	
	uint8_t res = 0;
	static uint8_t status = WAIT_HEAD_1;
	static uint16_t data_cnt = 0;
	
	switch(status)
	{
		case WAIT_HEAD_1:
		{
			if(c == 0xFA){
				pack_buff[0] = c;
				status = WAIT_HEAD_2;
			}
			
			res = 0;
		}break;
		case WAIT_HEAD_2:
		{
			if(c == r_head)
			{
				pack_buff[1] = c;
				status = WAIT_LEN_1;
			}
			else if(c != 0xFA)
			{
				status = WAIT_HEAD_1;
			}
			
			res = 0;
		}break;
		case WAIT_LEN_1:
		{
			pack_buff[2] = c;
			status = WAIT_LEN_2;
			
			res = 0;
		}break;
		case WAIT_LEN_2:
		{
			pack_buff[3] = c;
			if(*(uint16_t*)&pack_buff[2] > (MAX_PACKAGE_SIZE-PACK_SIZE_EXCEPT_DATA)){
				//Console.e(TAG, "err,pack len exceed max value:%d\n" , *(uint16_t*)&pack_buff[2]);
				status = WAIT_HEAD_1;
			}else{
				status = WAIT_CMD;
			}
			
			res = 0;
		}break;
		case WAIT_CMD:
		{
			pack_buff[4] = c;
			if(*(uint16_t*)&pack_buff[2] > 0){
				status = WAIT_DATA;
			}else{
				status = WAIT_CS_1;
			}
			data_cnt = 0;
			
			res = 0;
		}break;
		case WAIT_DATA:
		{
			pack_buff[5+data_cnt] = c;
			data_cnt++;
			
			if(data_cnt >= *(uint16_t*)&pack_buff[2]){
				status = WAIT_CS_1;
			}
			
			res = 0;
		}break;
		case WAIT_CS_1:
		{
			pack_buff[5+data_cnt] = c;
			status = WAIT_CS_2;
			
			res = 0;
		}break;
		case WAIT_CS_2:
		{
			pack_buff[6+data_cnt] = c;
			status = WAIT_0xFC;
			
			res = 0;
		}break;
		case WAIT_0xFC:
		{
			if(c == 0xFC)
			{
				pack_buff[7+data_cnt] = c;	
				status = WAIT_HEAD_1;
				res = 1;		
			}
			else if(c == 0xFA)
			{
				status = WAIT_HEAD_2;
				res = 0;
			}
			else
			{
				status = WAIT_HEAD_1;
				res = 0;
			}
			
			
		}break;
	}
	
	return res;
}

void lidar_lite_input(float distance);
void handle_package(const Package_Def package)
{
	switch(package.cmd){
		case CMD_SYNC:
		{
			//Console.w(TAG, "receive px4io sync\n");
			reply_sync();
		}break;
		case ACK_REBOOT:
		{
			Console.w(TAG, "ack reboot\n");
		}break;
		case CMD_CHANNEL_VAL:
		{
			float chan_val[CHAN_NUM];
			
			for(int i = 0 ; i < CHAN_NUM ; i++){
				chan_val[i] = rc_raw2chanval(((uint32_t*)package.usr_data)[i]);
			}
			rc_handle_ppm_signal(chan_val);
		}break;
		case ACK_CONFIG_CHANNEL:
		{
			//Console.w(TAG, "ack config channel\n");
		}break;
		case CMD_LIDAR_DIS:
		{
			OS_ENTER_CRITICAL;
			_lidar_dis = *((float*)package.usr_data);
			_lidar_recv_stamp = time_nowMs();
			OS_EXIT_CRITICAL;
		}break;
		case CMD_DEBUG:
		{
			char *str = (char*)package.usr_data;
			str[package.len] = '\0';
			Console.print("IO:%s", str);
		}break;
		case ACK_GET_PWM_CHANNEL:
		{
			float *pwm_dc = (float*)package.usr_data;
			Console.print("pwm get channel\n");
			for(uint8_t i = 0 ; i < MAX_PWM_MAIN_CHAN ; i++){
				_remote_pwm_duty_cycle[i] = pwm_dc[i];
			}

			//rt_sem_release(_sem_pwm_chan_recv);
		}break;
		case ACK_CONFIG_PWM_CHANNEL:
		{
			//TODO
		}break;
		default :
			Console.e(TAG, "unknow package:%d\n", package.cmd);
	}
}

void process_recv_pack(void)
{
	uint8_t ret;
	
	ret = parse_package(pack_buff , &Recv_Package);
	
	if(ret){
		Console.e(TAG, "Parse Pack Error:%d" , ret);
	}else{				
		handle_package(Recv_Package);
	}	
}
