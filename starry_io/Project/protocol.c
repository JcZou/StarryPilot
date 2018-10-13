/**
  ******************************************************************************
  * @file    protocol.c 
  * @author  J Zou
  * @version V1.0
  * @date    17-Feb-2017
  * @brief   Communication protocol with PX4FMU
  ******************************************************************************
*/  

#include "protocol.h"
#include "ppm_capture.h"
#include "usart.h"
#include "pwm.h"
#include <stdio.h>
#include <stdlib.h>

static uint8_t pack_buff[MAX_PACKAGE_SIZE];
static uint8_t pack_usr_buff[MAX_PACKAGE_SIZE];
static uint8_t sync_ack = 0;
Package_Def Recv_Package;

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
		free(package->usr_data);
	}
}

void free_sendpack(SendPackage_Def* send_pack)
{
	if(send_pack->send_buff != NULL){
		free(send_pack->send_buff);
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
		package->usr_data = malloc(len);
		if(package->usr_data == NULL){
			printf("Make_Package Heap is not enough:%d\n" , len);
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
	send_pack->send_buff = malloc(send_pack->buff_size);
	if(send_pack->send_buff == NULL){
		printf("Package2SendPack Heap is not enough:%d\n" , send_pack->buff_size);
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
		printf("ParsePackage data NULL\n");
		return 0x01;
	}
	
	tempPack.head[0] = data[0];
	tempPack.head[1] = data[1];
	tempPack.len = *((uint16_t*)&data[2]);
	tempPack.cmd = data[4];
	
	if(tempPack.head[0] != 0xFA || tempPack.head[1] != r_head){
		printf("ParsePackage pack head illegal\n");
		return 0x02;
	}
	
	if(data[7+tempPack.len] != 0xFC){
		printf("ParsePackage tail illegal\n");
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
		printf("checksum not correct %x\n" , correct_checksum);
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

uint8_t wait_complete_pack(uint8_t c, uint8_t* pack_buff)
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
				printf("err,pack len exceed max value:%d\n" , *(uint16_t*)&pack_buff[2]);
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

void handle_package(const Package_Def package)
{
	switch(package.cmd){
		case ACK_SYNC:
		{
			sync_ack = 1;
		}break;
		case CMD_REBOOT:
		{
			//send_package(ACK_REBOOT, NULL, 0);
			NVIC_SystemReset();
		}break;
		case CMD_CONFIG_CHANNEL:
		{
			if( config_ppm_send_freq(*package.usr_data) ){
				send_package(ACK_CONFIG_CHANNEL, package.usr_data, 1);
			}
		}break;
#ifdef USE_PWM_OUTPUT
		case CMD_SET_PWM_CHANNEL:
		{
			PWM_CHAN_MSG pwm_msg = *((PWM_CHAN_MSG*)package.usr_data);
			pwm_write(pwm_msg.duty_cyc, pwm_msg.chan_id);
		}break;
		case CMD_GET_PWM_CHANNEL:
		{
			float cur_dc[MAX_PWM_CHAN];

			pwm_read(cur_dc, PWM_CHAN_ALL);
			send_package(ACK_GET_PWM_CHANNEL, (uint8_t*)&cur_dc, sizeof(cur_dc));
		}break;
		case CMD_CONFIG_PWM_CHANNEL:
		{
			PWM_CONFIG_MSG pwm_conf_msg = *((PWM_CONFIG_MSG*)package.usr_data);
			if(pwm_configure(pwm_conf_msg.cmd, &pwm_conf_msg.val) == 0){
				send_package(ACK_CONFIG_PWM_CHANNEL, NULL, 0);
			}
		}break;
#endif
		default :
			printf("unknow package\n");
	}
}

uint8_t sync_finish(void)
{
	return sync_ack;
}

void protocol_input_byte(uint8_t ch)
{
	uint8_t ret;
	
	if(wait_complete_pack(ch, pack_buff))
	{
		ret = parse_package(pack_buff , &Recv_Package);
		
		if(ret){
			printf("parse packake error:%d" , ret);
		}else{				
			handle_package(Recv_Package);
		}	
	}
}

