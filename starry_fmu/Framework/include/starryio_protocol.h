/*
 * File      : starryio_protocol.h
 *
 * Change Logs:
 * Date           Author       Notes
 * 2017-02-18     zoujiachi   	the first version
 */
 
#ifndef __PX4IO_PROTOCOL_H__
#define __PX4IO_PROTOCOL_H__

#include "stm32f4xx.h"
#include <rtthread.h>

#define		PACK_SIZE_EXCEPT_DATA		8
#define		MAX_PACKAGE_SIZE			256

#define		PROTOCOL_SERVER
//#define		PROTOCOL_CLIENT

typedef struct
{
	uint8_t		head[2];
	uint16_t	len;
	uint8_t		cmd;
	uint8_t* 	usr_data;
	uint16_t 	checksum;
	uint8_t		endflag;
}Package_Def;

typedef struct
{
	uint8_t* 	send_buff;
	uint16_t 	buff_size;
}SendPackage_Def;

typedef enum
{
	ACK_SYNC = 0x01,
	CMD_REBOOT = 0x02,
	CMD_CONFIG_CHANNEL = 0x03,
	CMD_SET_PWM_CHANNEL = 0x04,
	CMD_GET_PWM_CHANNEL = 0x05,
	CMD_CONFIG_PWM_CHANNEL = 0x06,
}SERVER_CMD_Def;

typedef enum
{
	CMD_SYNC = 0x01,
	ACK_REBOOT = 0x02,
	ACK_CONFIG_CHANNEL = 0x03,
	CMD_CHANNEL_VAL = 0x04,
	CMD_LIDAR_DIS = 0x05,
	CMD_DEBUG = 0x06,
	ACK_GET_PWM_CHANNEL = 0x07,
	ACK_CONFIG_PWM_CHANNEL = 0x08,
}CLIENT_CMD_Def;

uint8_t make_package(uint8_t* data , char cmd , uint16_t len , Package_Def* package);
uint8_t package2sendpack(const Package_Def package , SendPackage_Def* send_pack);
void free_pack(Package_Def* package);
void free_sendpack(SendPackage_Def* send_pack);
uint8_t wait_complete_pack(uint8_t c);
void process_recv_pack(void);

#endif 
