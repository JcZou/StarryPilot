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

log_elem_t _Sensor_Param_elem[] = {
	LOG_ELEMENT_ARRAY("gyr_rotM", LOG_FLOAT, 9),
	LOG_ELEMENT("gyr_bias", LOG_FLOAT),
	LOG_ELEMENT_ARRAY("acc_rotM", LOG_FLOAT, 9),
	LOG_ELEMENT("acc_bias", LOG_FLOAT),
	LOG_ELEMENT_ARRAY("mag_rotM", LOG_FLOAT, 9),
	LOG_ELEMENT("mag_bias", LOG_FLOAT),
};

/////////////////////////////////////////////////////

log_field_t _log_field_list[] = {
	LOG_FIELD("IMU1", 0x01, _IMU1_elem),
	LOG_FIELD("MAG", 0x02, _MAG_elem),
	LOG_FIELD("BARO", 0x03, _BARO_elem),
	LOG_FIELD("GPS_uBlox", 0x04, _GPS_uBlox_elem),
	LOG_FIELD("INS_Out", 0x05, _INS_Out_elem),
	//LOG_FIELD("Sensor_Param", 0x06, _Sensor_Param_elem),
};

static log_header_t _log_header = { sizeof(_log_field_list) / sizeof(log_field_t), _log_field_list };
static FIL _log_fid;
static log_buffer_t _log_buffer;
static uint8_t _logging = 0;

uint8_t log_push(uint8_t* payload, uint16_t len)
{
	// log file log is not open
	if(_log_fid.fs == NULL || !_logging) {
		return 1;
	}

	uint32_t free_space_in_sector = LOG_BLOCK_SIZE - _log_buffer.index;

	// check if buffer is full
	if(free_space_in_sector < 1 + len) {
		if((_log_buffer.head + 1) % _log_buffer.num_sector == _log_buffer.tail) {
			// log buffer is full
			Console.print("log buffer full\r\n");
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
	if(_log_fid.fs == NULL || !_logging) {
		return 1;
	}

	uint32_t free_space_in_sector = LOG_BLOCK_SIZE - _log_buffer.index;

	// check if buffer is full
	if(free_space_in_sector < 1 + len) {
		if((_log_buffer.head + 1) % _log_buffer.num_sector == _log_buffer.tail) {
			// log buffer is full
			Console.print("log buffer full\r\n");
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

	if(!_logging) {
		if(_log_fid.fs != NULL && head_p == tail_p) { // no log data in buffer
			f_write(&_log_fid, &_log_buffer.data[tail_p * LOG_BLOCK_SIZE], _log_buffer.index, &bw);

			FRESULT res = f_close(&_log_fid);

			if(res == FR_OK) {
				Console.print("log stop!\r\n");
			} else {
				Console.print("log stop err:%d\r\n", res);
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

void log_test(char* file_name, uint32_t sec_size)
{
	// char test_data[4096];
	// memset(test_data, 'a', sizeof(test_data));
	// FIL fid;
	// UINT bw;
	// FRESULT fres = f_open(&fid, file_name, FA_OPEN_ALWAYS | FA_WRITE);
	// f_lseek(&fid, 4096);

	// Console.print("start log test, sec_size:%d\r\n", sec_size);
	// float max = 0;
	// float min = 10000000;
	// float total_time = 0;
	// for(int i = 0 ; i < 1024 ; i++)
	// {

	// 	timediff_us();
	// 	uint32_t start_ms = Uptime_ms();

	// 	f_write(&fid, test_data, sec_size, &bw);

	//     uint32_t time_us = timediff_us();
	// 	uint32_t time_ms = Uptime_ms() - start_ms;

	//     float elapse_time = (float)time_ms + (float)time_us/1000;
	//     total_time += elapse_time;
	//     if(elapse_time > max){
	//         max = elapse_time;
	//     }
	//     if(elapse_time < min){
	//         min = elapse_time;
	//     }
	// }

	// Console.print("total:%f avg:%f min:%f max:%f\r\n", total_time, total_time/1024, min, max);
	// f_close(&fid);
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
		// set log flag
		_logging = 1;

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

		Console.print("log start: %s\r\n", file_name);
	} else {
		Console.print("log file open fail:%d\r\n", fres);
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
	_logging = 0;
}

void log_init(void)
{
	_log_buffer.num_sector = LOG_BUFFER_SIZE / LOG_BLOCK_SIZE;
	_log_buffer.head = _log_buffer.tail = 0;
	_log_buffer.index = 0;
	_log_fid.fs = NULL;
}

uint8_t log_status(void)
{
	return _logging;
}

int handle_log_shell_cmd(int argc, char** argv)
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

