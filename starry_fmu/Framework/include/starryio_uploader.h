/*
 * File      : starryio_uploader.h
 *
 *
 * Change Logs:
 * Date           Author       	Notes
 * 2017-02-04     zoujiachi   	the first version
 */
 
#ifndef __PX4IO_UPLOADER_H__
#define __PX4IO_UPLOADER_H__

#include "stm32f4xx.h"
#include <rtthread.h>

rt_err_t uploader_init(void);
rt_err_t uploader_deinit(void);
void starryio_upload(void);

#endif
