/*
 * File      : console.h
 *
 *
 * Change Logs:
 * Date           Author       	Notes
 * 2016-03-22     zoujiachi   	the first version
 */
 
#ifndef __CONSOLE_H__
#define __CONSOLE_H__

#include <rtthread.h>
#include <rtdevice.h>

typedef enum
{
	CONSOLE_INTERFACE_SERIAL = 0,
	CONSOLE_INTERFACE_USB
}CONSOLE_INTERFACE_Typedef;

typedef struct
{
	void (*e)(char* tag, const char *fmt, ...);
	void (*w)(char* tag, const char *fmt, ...);
	void (*print)(const char *fmt, ...);
	void (*print2dev)(CONSOLE_INTERFACE_Typedef dev, const char *fmt, ...);
	void (*print_eachtime)(uint32_t *time_stamp, uint32_t time_ms, const char *fmt, ...);
	void (*write)(char* content, uint32_t len);
}CONSOLE_Typedef;


extern CONSOLE_Typedef Console;

int console_redirect_device(const char *name);

uint8_t console_init(CONSOLE_INTERFACE_Typedef console_if);

#endif
