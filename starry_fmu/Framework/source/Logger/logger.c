/*
 * File      : logger.c
 *
 *
 * Change Logs:
 * Date           Author       	Notes
 * 2018-03-29     zoujiachi   	the first version
 */

#include <stdio.h>
#include <string.h>
#include "global.h"
#include "ff.h"
#include "file_manager.h"
#include "console.h"
#include "logger.h"
#include "msh_usr_cmd.h"

static char* TAG = "Logger";

enum {
	LOG_INT8 = 0,
	LOG_UINT8,
	LOG_INT16,
	LOG_UINT16,
	LOG_INT32,
	LOG_UINT32,
	LOG_FLOAT,
	LOG_DOUBLE,
	LOG_BOOLEAN,
};

#define LOG_ELEMENT(_name, _type) { \
	#_name, \
	_type, \
    1 \
}

#define LOG_ELEMENT_ARRAY(_name, _type, _num) { \
	#_name, \
	_type, \
    _num \
}

#define LOG_FIELD(_name, _id, _elem_list) { \
	#_name, \
	_id, \
	sizeof(_elem_list)/sizeof(log_elem_t), \
    _elem_list \
}

log_elem_t _IMU1_elem[] = {
	LOG_ELEMENT_ARRAY("gyr_radPs_B", LOG_FLOAT, 3),
	LOG_ELEMENT_ARRAY("acc_mPs2_B", LOG_FLOAT, 3),
	LOG_ELEMENT("timestamp_ms", LOG_UINT32)
};

log_elem_t _MAG_elem[] = {
	LOG_ELEMENT_ARRAY("mag_ga_B", LOG_FLOAT, 3),
	LOG_ELEMENT("timestamp_ms", LOG_UINT32)
};

log_elem_t _BARO_elem[] = {
	LOG_ELEMENT("pressure_Pa", LOG_INT32),
	LOG_ELEMENT("temperature_deg", LOG_FLOAT),
	LOG_ELEMENT("timestamp_ms", LOG_UINT32)
};

log_elem_t _GPS_uBlox_elem[] = {
	LOG_ELEMENT("iTOW", LOG_UINT32),
	LOG_ELEMENT("year", LOG_UINT16),
	LOG_ELEMENT("month", LOG_UINT8),
	LOG_ELEMENT("day", LOG_UINT8),
	LOG_ELEMENT("hour", LOG_UINT8),
	LOG_ELEMENT("min", LOG_UINT8),
	LOG_ELEMENT("sec", LOG_UINT8),
	LOG_ELEMENT("valid", LOG_UINT8),
	LOG_ELEMENT("tAcc", LOG_UINT32),
	LOG_ELEMENT("nano", LOG_INT32),
	LOG_ELEMENT("fixType", LOG_UINT8),
	LOG_ELEMENT("flags", LOG_UINT8),
	LOG_ELEMENT("reserved1", LOG_UINT8),
	LOG_ELEMENT("numSV", LOG_UINT8),
	LOG_ELEMENT("lon", LOG_INT32),
	LOG_ELEMENT("lat", LOG_INT32),
	LOG_ELEMENT("height", LOG_INT32),
	LOG_ELEMENT("hMSL", LOG_INT32),
	LOG_ELEMENT("hAcc", LOG_UINT32),
	LOG_ELEMENT("vAcc", LOG_UINT32),
	LOG_ELEMENT("velN", LOG_INT32),
	LOG_ELEMENT("velE", LOG_INT32),
	LOG_ELEMENT("velD", LOG_INT32),
	LOG_ELEMENT("gSpeed", LOG_INT32),
	LOG_ELEMENT("headMot", LOG_INT32),
	LOG_ELEMENT("sAcc", LOG_UINT32),
	LOG_ELEMENT("headAcc", LOG_UINT32),
	LOG_ELEMENT("pDOP", LOG_UINT16),
	LOG_ELEMENT("reserved2", LOG_UINT16),
	LOG_ELEMENT("timestamp_ms", LOG_UINT32)
};

log_elem_t _INS_Out_elem[] = {
	LOG_ELEMENT_ARRAY("quat", LOG_FLOAT, 4),
	LOG_ELEMENT_ARRAY("euler", LOG_FLOAT, 3),
	LOG_ELEMENT_ARRAY("rot_radPs_B", LOG_FLOAT, 3),
	LOG_ELEMENT_ARRAY("sfor_mPs2_B", LOG_FLOAT, 3),
	LOG_ELEMENT_ARRAY("qvel_cmPs_O", LOG_INT32, 3),
	LOG_ELEMENT("lon_1e7_deg", LOG_INT32),
	LOG_ELEMENT("lat_1e7_deg", LOG_INT32),
	LOG_ELEMENT("altitude_cm", LOG_INT32),
	LOG_ELEMENT("timestamp_ms", LOG_UINT32)
};

/////////////////////////////////////////////////////

log_field_t _log_field_list[] = {
	LOG_FIELD("IMU1", 0x01, _IMU1_elem),
	LOG_FIELD("MAG", 0x02, _MAG_elem),
	LOG_FIELD("BARO", 0x03, _BARO_elem),
	LOG_FIELD("GPS_uBlox", 0x04, _GPS_uBlox_elem),
	LOG_FIELD("INS_Out", 0x05, _INS_Out_elem),
};

typedef struct
{
	uint32_t total_msg;
	uint32_t lost_msg;
}log_filed_status;

struct log_status_t
{
	uint8_t 			file_name[20];
	uint8_t 			is_logging;
	log_filed_status	field_status[sizeof(_log_field_list)/sizeof(log_field_t)];
}_log_status;

static log_header_t _log_header = { sizeof(_log_field_list) / sizeof(log_field_t), _log_field_list };
static FIL _log_fid;
static log_buffer_t _log_buffer;
// static uint8_t _logging = 0;
// static uint8_t _log_filename[20];

static int32_t _find_filed_index(uint8_t msg_id)
{
	for(int i = 0 ; i < sizeof(_log_field_list)/sizeof(log_field_t) ; i++){
		if(_log_field_list[i].msg_id == msg_id) {
			return i;
		}
	}	

	return -1;
}

uint8_t log_push(uint8_t* payload, uint16_t len)
{
	// log file log is not open
	if(_log_fid.fs == NULL || !_log_status.is_logging) {
		return 1;
	}

	uint32_t free_space_in_sector = LOG_BLOCK_SIZE - _log_buffer.index;

	// check if buffer is full
	if(free_space_in_sector < 1 + len) {
		if((_log_buffer.head + 1) % _log_buffer.num_sector == _log_buffer.tail) {
			// log buffer is full
			//Console.print("log buffer full\r\n");

			return 2;
		}
	}

	if(free_space_in_sector < len) {
		memcpy(&_log_buffer.data[_log_buffer.head * LOG_BLOCK_SIZE + _log_buffer.index], payload, free_space_in_sector);

		// move head point to next sector
		_log_buffer.head = (_log_buffer.head + 1) % _log_buffer.num_sector;
		_log_buffer.index = 0;

		memcpy(&_log_buffer.data[_log_buffer.head * LOG_BLOCK_SIZE + _log_buffer.index], &payload[free_space_in_sector], len - free_space_in_sector);
		_log_buffer.index += len - free_space_in_sector;
	} else {
		memcpy(&_log_buffer.data[_log_buffer.head * LOG_BLOCK_SIZE + _log_buffer.index], payload, len);
		_log_buffer.index += len;
	}
}

uint8_t log_push_msg(uint8_t* payload, uint8_t msg_id, uint16_t len)
{
	uint8_t res = 0;

	// log file log is not open
	if(_log_fid.fs == NULL || !_log_status.is_logging) {
		return 1;
	}

	uint32_t free_space_in_sector = LOG_BLOCK_SIZE - _log_buffer.index;

	// check if buffer is full
	if(free_space_in_sector < 1 + len) {
		if((_log_buffer.head + 1) % _log_buffer.num_sector == _log_buffer.tail) {
			// log buffer is full, drop this msg
			int32_t index = _find_filed_index(msg_id);

			if(index >= 0)
				_log_status.field_status[index].lost_msg += 1;

			return 2;
		}
	}

	/* write msg_id */
	if(free_space_in_sector < 1) {
		// move head point to next sector
		_log_buffer.head = (_log_buffer.head + 1) % _log_buffer.num_sector;
		_log_buffer.index = 0;
	}

	_log_buffer.data[_log_buffer.head * LOG_BLOCK_SIZE + _log_buffer.index] = msg_id;
	_log_buffer.index += 1;

	/* write payload */
	free_space_in_sector = LOG_BLOCK_SIZE - _log_buffer.index;

	if(free_space_in_sector < len) {
		memcpy(&_log_buffer.data[_log_buffer.head * LOG_BLOCK_SIZE + _log_buffer.index], payload, free_space_in_sector);

		// move head point to next sector
		_log_buffer.head = (_log_buffer.head + 1) % _log_buffer.num_sector;
		_log_buffer.index = 0;

		memcpy(&_log_buffer.data[_log_buffer.head * LOG_BLOCK_SIZE + _log_buffer.index], &payload[free_space_in_sector], len - free_space_in_sector);
		_log_buffer.index += len - free_space_in_sector;
	} else {
		memcpy(&_log_buffer.data[_log_buffer.head * LOG_BLOCK_SIZE + _log_buffer.index], payload, len);
		_log_buffer.index += len;
	}

	int32_t index = _find_filed_index(msg_id);

	if(index >= 0)
		_log_status.field_status[index].total_msg += 1;

	return 0;
}

uint8_t log_write(void)
{
	UINT bw;
	uint32_t head_p, tail_p;
	uint8_t res = 0;

	// log file log is not open
	if(_log_fid.fs == NULL) {
		return 1;
	}

	OS_ENTER_CRITICAL;
	head_p = _log_buffer.head;
	tail_p = _log_buffer.tail;
	OS_EXIT_CRITICAL;

	if(!_log_status.is_logging) {
		if(_log_fid.fs != NULL && head_p == tail_p) { // no log data in buffer
			f_write(&_log_fid, &_log_buffer.data[tail_p * LOG_BLOCK_SIZE], _log_buffer.index, &bw);

			FRESULT res = f_close(&_log_fid);

			if(res == FR_OK) {
				Console.print("stop logging: %s\n", _log_status.file_name);
			} else {
				Console.print("log stop err:%d\n", res);
			}
		}
	}

	while(head_p != tail_p) {
		f_write(&_log_fid, &_log_buffer.data[tail_p * LOG_BLOCK_SIZE], LOG_BLOCK_SIZE, &bw);
		tail_p = (tail_p + 1) % _log_buffer.num_sector;
		OS_ENTER_CRITICAL;
		_log_buffer.tail = tail_p;
		OS_EXIT_CRITICAL;
	}

	return 0;
}

uint8_t log_start(char* file_name)
{
	uint8_t res = 0;
	UINT bw;

	/* create log file */
	FRESULT fres = f_open(&_log_fid, file_name, FA_OPEN_ALWAYS | FA_WRITE);

	if(fres == FR_OK) {

		// init log buffer
		_log_buffer.head = _log_buffer.tail = 0;
		_log_buffer.index = 0;

		/* write header */
		log_push(&_log_header.num_field, sizeof(_log_header.num_field));    // write num_filed

		// write log filed
		for(int n = 0 ; n < _log_header.num_field ; n++) {
			log_push(_log_header.filed_list[n].name, LOG_MAX_NAME_LEN);
			log_push(&_log_header.filed_list[n].msg_id, sizeof(_log_header.filed_list[n].msg_id));
			log_push(&_log_header.filed_list[n].num_elem, sizeof(_log_header.filed_list[n].num_elem));

			// write log element
			for(int k = 0 ; k < _log_header.filed_list[n].num_elem ; k++) {
				log_push(_log_header.filed_list[n].elem_list[k].name, LOG_MAX_NAME_LEN);
				log_push(&_log_header.filed_list[n].elem_list[k].type, sizeof(_log_header.filed_list[n].elem_list[k].type));
				log_push(&_log_header.filed_list[n].elem_list[k].number, sizeof(_log_header.filed_list[n].elem_list[k].number));
			}
		}

		/* set log status */
		strcpy(_log_status.file_name, file_name);
		_log_status.is_logging = 1;
		for(int i = 0 ; i < sizeof(_log_field_list)/sizeof(log_field_t) ; i++) {
			_log_status.field_status[i].total_msg = 0;
			_log_status.field_status[i].lost_msg = 0;
		}

		Console.print("start logging: %s\n", file_name);
	} else {
		Console.print("log file open fail:%d\n", fres);
		res = 1;
	}

	return res;
}

uint8_t log_auto_start(void)
{
	DIR dir;
	FILINFO fno;
	FRESULT fres;
	char filename_buffer[20];
	uint16_t file_id = 1;

	while(file_id < 100) {
		sprintf(filename_buffer, "LOG%d.BIN", file_id++);
		fres = f_open(&_log_fid, filename_buffer, FA_OPEN_EXISTING | FA_READ);

		if(fres == FR_NO_FILE) {
			f_close(&_log_fid);
			log_start(filename_buffer);
			return 1;
		} else {
			f_close(&_log_fid);
		}
	}

	return 0;
}

void log_stop(void)
{
	_log_status.is_logging = 0;
}

void log_init(void)
{
	_log_buffer.num_sector = LOG_BUFFER_SIZE / LOG_BLOCK_SIZE;
	_log_buffer.head = _log_buffer.tail = 0;
	_log_buffer.index = 0;
	_log_fid.fs = NULL;

	/* initialize log status */
	strcpy(_log_status.file_name, "");
	_log_status.is_logging = 0;
}

uint8_t log_is_logging(void)
{
	return _log_status.is_logging;
}

void log_dump_status(void)
{
	if(log_is_logging()) {
		Console.print("%s is logging.\n", _log_status.file_name);
		
		for(int i = 0 ; i < sizeof(_log_field_list)/sizeof(log_field_t) ; i++) {
			Console.print("%-20s msg id:%-3d total msg:%-10d lost msg:%-5d\n", _log_field_list[i].name, _log_field_list[i].msg_id,
				_log_status.field_status[i].total_msg, _log_status.field_status[i].lost_msg);
		}
	} else {
		Console.print("no file is logging.\n");
	}
}

int handle_log_shell_cmd(int argc, char** argv, int optc, sh_optv* optv)
{
	int res = 0;

	if(argc > 1) {
		if(strcmp(argv[1], "start") == 0) {
			if(argc > 2)
				res = log_start(argv[2]);	// default period
		}

		if(strcmp(argv[1], "stop") == 0) {
			log_stop();
		}

		if(strcmp(argv[1], "status") == 0) {
			log_dump_status();
		}
	}

	return res;
}

void logger_entry(void* parameter)
{

	while(1) {
		log_write();
		rt_thread_delay(1);
	}
}

