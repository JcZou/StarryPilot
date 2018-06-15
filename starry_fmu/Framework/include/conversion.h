/*
 * File      : conversion.h
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-04-14     zoujiachi    first version.
 */
 
#ifndef __CONVERSION_H__
#define __CONVERSION_H__

#include "stdint.h"

void Msb2Lsb(uint8_t* data , uint8_t bytes);
int16_t int16_t_from_bytes(uint8_t bytes[]);

#endif
