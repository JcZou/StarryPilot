/*
 * File      : uMCN.h
 *
 *
 * Change Logs:
 * Date           Author       	Notes
 * 2017-10-16     zoujiachi   	the first version
 */
 
#ifndef __UMCN_H__
#define __UMCN_H__

#include <stdlib.h>
#include <rtthread.h>
#include <rtdevice.h>
#include "global.h"

#define MCN_EVENT_HANDLE			rt_event_t	
#define MCN_SEND_EVENT(event_t)		rt_event_send(event_t, 1)
#define MCN_MALLOC(size)			rt_malloc(size)
#define MCN_FREE(ptr)				rt_free(ptr)
#define MCN_ENTER_CRITICAL			OS_ENTER_CRITICAL
#define MCN_EXIT_CRITICAL			OS_EXIT_CRITICAL

#define MCN_MAX_LINK_NUM		30

typedef struct mcn_node		McnNode;
typedef struct mcn_node*	McnNode_t;
struct mcn_node
{
	volatile uint8_t renewal;
	void (*cb)(void *parameter);
	McnNode_t next;
};

typedef struct mcn_hub		McnHub;
struct mcn_hub
{
	const char* obj_name;
	const uint32_t obj_size;
	void* pdata;
	McnNode_t link_head;
	McnNode_t link_tail;
	uint32_t link_num;
	uint8_t published;	// publish flag
};

#define MCN_ID(_name)				(&__mcn_##_name)

#define MCN_DECLARE(_name) 			extern McnHub __mcn_##_name
	
#define MCN_DEFINE(_name, _size)			\
	McnHub __mcn_##_name = {	        	\
		.obj_name = #_name,					\
		.obj_size = _size,					\
		.pdata = NULL,                      \
		.link_head = NULL,	                \
		.link_tail = NULL,	                \
		.link_num = 0,						\
		.published = 0						\
	}
	
int mcn_advertise(McnHub* hub);
McnNode_t mcn_subscribe(McnHub* hub, void (*cb)(void *parameter));
int mcn_publish(McnHub* hub, const void* data);
bool mcn_poll(McnNode_t node_t);
int mcn_copy(McnHub* hub, McnNode_t node_t, void* buffer);
int mcn_copy_from_hub(McnHub* hub, void* buffer);

#endif
	