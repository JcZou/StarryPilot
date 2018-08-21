/*
* File      : mavlink_customer.h
*
*
* Change Logs:
* Date			Author			Notes
* 2016-06-23	zoujiachi		the first version
*/

#include <rtthread.h>
#include <stdint.h>
#include "quaternion.h"
#include "ringbuffer.h"

#pragma anon_unions
#include "../../Library/mavlink/v1.0/common/mavlink.h"
#include "mavlink_status.h"

#define MAX_PERIOD_MSG_QUEUE_SIZE	20
#define MAX_TEMP_MSG_QUEUE_SIZE		5

typedef struct
{
	uint8_t 	msgid;
	uint8_t		enable;
	uint16_t 	period;
	uint32_t	time_stamp;
	void (* msg_pack_cb)(mavlink_message_t *msg_t);
}MAV_PeriodMsg;

typedef struct
{
	MAV_PeriodMsg 		queue[MAX_PERIOD_MSG_QUEUE_SIZE];
	uint16_t 			size;
	uint16_t			index;
}MAV_PeriodMsg_Queue;

typedef struct
{
	mavlink_message_t 	queue[MAX_TEMP_MSG_QUEUE_SIZE];
	uint16_t 			head;
	uint16_t			tail;
}MAV_TempMsg_Queue;

extern ringbuffer* _mav_serial_rb;

rt_err_t device_mavproxy_init(void);
void mavproxy_rx_entry(void *param);
void mavproxy_entry(void *parameter);
uint8_t mavproxy_msg_serial_control_send(uint8_t *data, uint8_t count);
uint16_t mavproxy_msg_serial_control_read(uint8_t *data, uint16_t size);
void mavlink_send_status(mav_status_type status);
void mavlink_send_calibration_progress_msg(uint8_t progress);

