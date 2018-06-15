/*
 * File      : printf.c
 *
 * Change Logs:
 * Date           Author       Notes
 * 2016-06-19     zoujiachi    first version.
 */
 
#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>
#include <stdio.h>

#if defined(RT_USING_DEVICE) && defined(RT_USING_CONSOLE)
extern rt_device_t _console_device;

//redefine fputc(),for printf() function to call
//int fputc(int ch, FILE * file)
//{
//    rt_uint16_t old_flag = _console_device->open_flag;

//	_console_device->open_flag |= RT_DEVICE_FLAG_STREAM;
//	rt_device_write(_console_device, 0, (void*)&ch, 1);
//	_console_device->open_flag = old_flag;

//	return ch; 
//}
#endif


