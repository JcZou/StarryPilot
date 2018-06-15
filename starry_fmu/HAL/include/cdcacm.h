/*
 * File      : cdcacm.h
 *
 *
 * Change Logs:
 * Date           Author       	Notes
 * 2016-03-21     zoujiachi   	the first version
 */
 
#ifndef __CDCACM_H__
#define __CDCACM_H__

#include "stm32f4xx.h"
#include "usbd_cdc_core_loopback.h"
#include "usbd_usr.h"
#include "usbd_desc.h"
#include "usbd_cdc_vcp.h"

#define USB_DEVICE_NAME			"usb"
#define USB_CMD_RECEIVE_CNT		0
#define USB_CMD_RECEIVE_FIN		1
#define USB_CMD_SEND_FIN		2

extern USB_OTG_CORE_HANDLE           USB_OTG_dev;

uint8_t usb_cdc_init(void);
void cdc_send_data(uint8_t* pbuf, uint32_t buf_len);
uint8_t cdc_receive_data(uint8_t *pbuf, uint32_t len);
uint8_t cdc_check_sent(void);
uint8_t cdc_check_receive(void);
uint32_t cdc_get_receive_cnt(void);

#endif
