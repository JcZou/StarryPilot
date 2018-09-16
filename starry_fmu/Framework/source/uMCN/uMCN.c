/*
 * File      : uMCN.c
 *
 *
 * Change Logs:
 * Date           Author       	Notes
 * 2017-10-16     zoujiachi   	the first version
 */

#include <string.h>
#include "uMCN.h"
#include "console.h"

static char* TAG = "uMCN";

int mcn_advertise(McnHub* hub)
{
	int res = 0;
	
	if(hub->pdata != NULL){
		// already advertised
		return 0;
	}
	
	MCN_ENTER_CRITICAL;
	hub->pdata = MCN_MALLOC(hub->obj_size);
	if(hub->pdata == NULL){
		res = -1;
	}
	MCN_EXIT_CRITICAL;
	
	return res;
}

McnNode_t mcn_subscribe(McnHub* hub, void (*cb)(void *parameter))
{
	if(hub->link_num >= MCN_MAX_LINK_NUM){
		Console.w(TAG, "mcn link num is already full!\n");
		return NULL;
	}
	
	McnNode_t node = (McnNode_t)MCN_MALLOC(sizeof(McnNode));
	if(node == NULL){
		Console.e(TAG, "mcn create node fail!\n");
		return NULL;
	}
	node->renewal = 0;
	node->cb = cb;
	node->next = NULL;
	
	MCN_ENTER_CRITICAL;
	/* no node link yet */
	if(hub->link_tail == NULL){
		hub->link_head = hub->link_tail = node;
	}else{	
		hub->link_tail->next = node;
		hub->link_tail = node;
	}
	hub->link_num++;
	MCN_EXIT_CRITICAL;
	
	return node;
}

int mcn_publish(McnHub* hub, const void* data)
{
	if(data == NULL){
		Console.w(TAG, "null data publish!\n");
		return -1;
	}
	
	if(hub->pdata == NULL){
		// hub is not advertised yet
		return -1;
	}
	
	MCN_ENTER_CRITICAL;
	/* copy data to hub */
	memcpy(hub->pdata, data, hub->obj_size);
	/* update each node's renewal flag */
	McnNode_t node = hub->link_head;
	while(node != NULL){
		node->renewal = 1;
		node = node->next;
	}
	hub->published = 1;
	MCN_EXIT_CRITICAL;
	
	/* invoke callback func */
	node = hub->link_head;
	while(node != NULL){
		if(node->cb != NULL){
			node->cb(hub->pdata);
		}
		node = node->next;
	}
	
	return 0;
}

bool mcn_poll(McnNode_t node_t)
{
	bool renewal;
	
	MCN_ENTER_CRITICAL;
	renewal = node_t->renewal;
	MCN_EXIT_CRITICAL;
	
	return renewal;
}

int mcn_copy(McnHub* hub, McnNode_t node_t, void* buffer)
{
	if(hub->pdata == NULL){
		// not advertised yet
		Console.e(TAG, "uMCN, copy from null hub:%s\n", hub->obj_name);
		return 1;
	}
	if(!hub->published){
		// copy before published
		return 2;
	}
	
	MCN_ENTER_CRITICAL;
	memcpy(buffer, hub->pdata, hub->obj_size);
	node_t->renewal = 0;
	MCN_EXIT_CRITICAL;
	
	return 0;
}

int mcn_copy_from_hub(McnHub* hub, void* buffer)
{
	if(hub->pdata == NULL){
		// not advertised yet
		Console.e(TAG, "uMCN, copy from null hub:%s\n", hub->obj_name);
		return 1;
	}
	if(!hub->published){
		// copy before published
		return 2;
	}
	
	MCN_ENTER_CRITICAL;
	memcpy(buffer, hub->pdata, hub->obj_size);
	MCN_EXIT_CRITICAL;
	
	return 0;
}

int handle_uMCN_cmd(int argc, char** argv)
{
	if(argc > 1){
		if(strcmp("echo", argv[1]) == 0){
			if(argc > 2){

			}
		}
	}
	
	return 0;
}