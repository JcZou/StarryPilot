/*
 * File      : conversion.c
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-04-14     zoujiachi    first version.
 */
 
#include "conversion.h"

void Msb2Lsb(uint8_t* data , uint8_t bytes)
{
	uint8_t temp;
	
	if(!bytes)
		return;
	
	for(uint8_t i = 0 ; i<bytes/2 ; i++){
		temp = data[i];
		data[i] = data[bytes-1-i];
		data[bytes-1-i] = temp;
	}
}

int16_t int16_t_from_bytes(uint8_t bytes[])
{
	union {
		uint8_t    b[2];
		int16_t    w;
	} u;

	u.b[1] = bytes[0];
	u.b[0] = bytes[1];

	return u.w;
}
