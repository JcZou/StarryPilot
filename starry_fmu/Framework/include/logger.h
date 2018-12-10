/*
 * File      : logger.h
 *
 *
 * Change Logs:
 * Date           Author       	Notes
 * 2018-03-29     zoujiachi   	the first version
 */
 
#ifndef __LOGGER_H__
#define __LOGGER_H__

#include <rtthread.h>
#include <rtdevice.h>
#include "global.h"


#define LOG_MAX_NAME_LEN            20
#define LOG_BUFFER_SIZE             16384
#define LOG_BLOCK_SIZE              4096        // write 8 sectors in each time to increase wrte speed       

// Macro to define packed structures
#ifdef __GNUC__
  #define LOGPACKED( __Declaration__ ) __Declaration__ __attribute__((packed))
#else
  #define LOGPACKED( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )
#endif

LOGPACKED(
typedef struct
{
    char name[LOG_MAX_NAME_LEN];
    uint16_t type;
    uint16_t number;
})log_elem_t;

LOGPACKED(
typedef struct
{
    char name[LOG_MAX_NAME_LEN];
    uint8_t msg_id;
    uint8_t num_elem;
    log_elem_t *elem_list;
})log_field_t;

LOGPACKED(
typedef struct
{
    uint8_t num_field;
    log_field_t *filed_list;
})log_header_t;

typedef struct
{
    uint8_t data[LOG_BUFFER_SIZE];
    uint32_t head;      // head point for sector
    uint32_t tail;      // tail point for sector
    uint32_t num_sector;
    uint32_t index;     // index in sector
}log_buffer_t;


void log_init(void);
uint8_t log_start(char *file_name);
void log_stop(void);
uint8_t log_write(void);
uint8_t log_push(uint8_t *payload, uint16_t len);
uint8_t log_push_msg(uint8_t *payload, uint8_t msg_id, uint16_t len);
uint8_t log_status(void);

void logger_entry(void *parameter);

#endif
