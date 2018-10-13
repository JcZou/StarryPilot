/**
  ******************************************************************************
  * @file    debug.c 
  * @author  J Zou
  * @version V1.0
  * @date    11-Oct-2018
  * @brief   Debug Module
  ******************************************************************************
*/  

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include "debug.h"
#include "protocol.h"

#define DEBUG_BUFFER_SIZE			128

static char _dbg_buf[DEBUG_BUFFER_SIZE];

void debug(const char *fmt, ...)
{
	va_list args;
    int length;
	
	va_start(args, fmt);
	length = vsprintf(_dbg_buf, fmt, args);
	va_end(args);
	
	send_package(CMD_DEBUG, (uint8_t*)_dbg_buf, length>DEBUG_BUFFER_SIZE ? DEBUG_BUFFER_SIZE : length);
}
