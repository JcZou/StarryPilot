/*
 * File      : console.c
 *
 *
 * Change Logs:
 * Date           Author       	Notes
 * 2016-03-22     zoujiachi   	the first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <stdio.h>
#include <stdlib.h>
#include "console.h"
#include "delay.h"
#include "ringbuffer.h"

CONSOLE_Typedef Console;

static rt_device_t console_device;
static rt_device_t console_uart_device;
static rt_device_t console_usb_device;

#define CONSOLE_BUFF_SIZE			128
#define CONSOLE_SEND_BUFF_SIZE		1024
static char console_buf[CONSOLE_BUFF_SIZE];

/* redefine fputc(),for printf() function to call */
int fputc(int ch, FILE * file)
{
	if(console_device != NULL){
		rt_device_write(console_device, 0, (void*)&ch, 1);
	}

	return ch; 
}

void console_output(rt_device_t dev, char* content, uint32_t len)
{
	rt_device_write(dev, 0, (void*)content, len);
}

void console_error(char* tag, const char *fmt, ...)
{
	va_list args;
    int length;
	
	va_start(args, fmt);
	length = vsprintf(console_buf, fmt, args);
	va_end(args);
	
	console_output(console_device, console_buf, length);
}

void console_warning(char* tag, const char *fmt, ...)
{
	va_list args;
    int length;
	
	va_start(args, fmt);
	length = vsprintf(console_buf, fmt, args);
	va_end(args);
	
	console_output(console_device, console_buf, length);
}

void console_print(const char *fmt, ...)
{
	va_list args;
    int length;
	
	va_start(args, fmt);
	length = vsprintf(console_buf, fmt, args);
	va_end(args);
	
	console_output(console_device, console_buf, length);
}

void console_print2dev(CONSOLE_INTERFACE_Typedef dev, const char *fmt, ...)
{
	va_list args;
    int length;
	
	va_start(args, fmt);
	length = vsprintf(console_buf, fmt, args);
	va_end(args);
	
	if(dev == CONSOLE_INTERFACE_SERIAL){
		console_output(console_uart_device, console_buf, length);
	}else if(dev == CONSOLE_INTERFACE_USB){
		console_output(console_usb_device, console_buf, length);
	}
}

void console_print_eachtime(uint32_t *time_stamp, uint32_t time_ms, const char *fmt, ...)
{
	uint32_t now = time_nowMs();
	if(now - *time_stamp > time_ms){
		*time_stamp = now;
		
		va_list args;
		int length;
		
		va_start(args, fmt);
		length = vsprintf(console_buf, fmt, args);
		va_end(args);
		
		console_output(console_device, console_buf, length);
	}
}

void console_write(char* content, uint32_t len)
{
	console_output(console_device, content, len);
}

int console_redirect_device(const char *name)
{
	rt_device_t new_dev = rt_device_find(name);
	if (!new_dev) {
		return -1;
	}
	if (console_device) {
		rt_device_open(new_dev , RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX);
		rt_device_close(console_device);
		console_device = new_dev;
	}

	return 0;
}

uint8_t console_init(CONSOLE_INTERFACE_Typedef console_if)
{	
	Console.e = console_error;
	Console.w = console_warning;
	Console.print = console_print;
	Console.print2dev = console_print2dev;
	Console.print_eachtime = console_print_eachtime;
	Console.write = console_write;
	
	if(console_if == CONSOLE_INTERFACE_SERIAL)
		console_device = rt_device_find("uart3");
	else 
		console_device = rt_device_find("usb");
	
	console_uart_device = rt_device_find("uart3");
	console_usb_device = rt_device_find("usb");
	
	if(console_device){
		if(console_device == console_uart_device)
			rt_device_open(console_device , RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX);
		else
			rt_device_open(console_device , RT_DEVICE_OFLAG_RDWR);
	}
	
	if(console_uart_device && console_uart_device != console_device){
		rt_device_open(console_uart_device , RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX);
	}
	if(console_usb_device && console_usb_device != console_device){
		rt_device_open(console_usb_device , RT_DEVICE_OFLAG_RDWR);
	}
	
	return 0;
}
