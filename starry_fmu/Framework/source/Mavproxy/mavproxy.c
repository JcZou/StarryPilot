/*
* File      : mavproxy.c
*
*
* Change Logs:
* Date			Author			Notes
* 2016-06-23	zoujiachi		the first version
*/

#include <rthw.h>
#include <rtdevice.h>
#include <rtthread.h>
#include <stdio.h>
#include <string.h>
#include "console.h"
#include "delay.h"
#include "quaternion.h"
#include "pos_estimator.h"
#include "att_estimator.h"
#include "mavproxy.h"
#include "uMCN.h"
#include "sensor_manager.h"
#include "gps.h"
#include "statistic.h"
#include "mavlink_param.h"
#include "mavlink_status.h"
#include "calibration.h"
#include "shell.h"

#define EVENT_MAVPROXY_UPDATE		(1<<0)
#define EVENT_MAVPROXY_SEND_ALL_PARAM		(1<<1)

#define MAV_SERIAL_BUFFER_SIZE		128

uint8_t mav_tx_buff[1024];
uint8_t mav_tx_buff2[256];
mavlink_system_t mavlink_system;
/* disable mavlink sending */
uint8_t _mav_disable = 0;

ringbuffer* _mav_serial_rb;
uint8_t _mav_serial_buffer[MAV_SERIAL_BUFFER_SIZE];

static char *TAG = "MAV_Proxy";

static struct rt_timer timer_mavproxy;
static struct rt_event event_mavproxy;

static mavlink_status_t mav_status;
static MAV_PeriodMsg_Queue _period_msg_queue;
static MAV_TempMsg_Queue _temp_msg_queue;
static McnNode_t _gps_status_node_t;

static char thread_mavlink_rx_stack[2048];
struct rt_thread thread_mavlink_rx_handle;

MCN_DEFINE(HIL_STATE_Q, sizeof(mavlink_hil_state_quaternion_t));
MCN_DEFINE(HIL_SENSOR, sizeof(mavlink_hil_sensor_t));
MCN_DEFINE(HIL_GPS, sizeof(mavlink_hil_gps_t));

MCN_DECLARE(SENSOR_FILTER_GYR);
MCN_DECLARE(SENSOR_FILTER_ACC);
MCN_DECLARE(SENSOR_FILTER_MAG);
MCN_DECLARE(SENSOR_BARO);
MCN_DECLARE(ALT_INFO);
MCN_DECLARE(SENSOR_LIDAR);
MCN_DECLARE(CORRECT_LIDAR);
MCN_DECLARE(ATT_EULER);
MCN_DECLARE(GPS_POSITION);
MCN_DECLARE(GPS_STATUS);

extern uint8_t mavlink_lowlevel_write(uint8_t* buff, uint16_t len);
extern int mavlink_lowlevel_read(uint8_t* buff, uint16_t len);
extern void mavproxy_lowlevel_init(void);
extern int mavproxy_console_proc(int count);
uint8_t mavproxy_temp_msg_push(mavlink_message_t *msg);

uint8_t mavlink_msg_transfer(uint8_t chan, uint8_t* msg_buff, uint16_t len)
{
	return mavlink_lowlevel_write(msg_buff, len);
}

rt_err_t device_mavproxy_init(void)
{
	mavlink_system.sysid = 1;        
	mavlink_system.compid = 1;    

	return 0;
}

void mavproxy_msg_heartbeat_pack(mavlink_message_t *msg_t)
{
    mavlink_heartbeat_t heartbeat;
	uint16_t len;
	
	heartbeat.type          = MAV_TYPE_QUADROTOR;
    heartbeat.autopilot     = MAV_AUTOPILOT_PX4;
	//TODO, fill base_mode and custom_mode
    heartbeat.base_mode     = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
	heartbeat.custom_mode   = 0;
	heartbeat.system_status = MAV_STATE_STANDBY;
	
	mavlink_msg_heartbeat_encode(mavlink_system.sysid, mavlink_system.compid, msg_t, &heartbeat);
}

void mavproxy_msg_sys_status_pack(mavlink_message_t *msg_t)
{
    mavlink_sys_status_t sys_status;
	uint16_t len;
	
	sys_status.onboard_control_sensors_present = 1;
	sys_status.onboard_control_sensors_enabled = 1;
	sys_status.onboard_control_sensors_health = 1;
	sys_status.load = (uint16_t)(get_cpu_usage()*1e3);
	sys_status.voltage_battery = 11000;
	sys_status.current_battery = -1;
	sys_status.battery_remaining = -1;
	
	mavlink_msg_sys_status_encode(mavlink_system.sysid, mavlink_system.compid, msg_t, &sys_status);
}

void mavproxy_msg_scaled_imu_pack(mavlink_message_t *msg_t)
{
	mavlink_scaled_imu_t scaled_imu;
	uint16_t len;

	float gyr[3], acc[3], mag[3];
	mcn_copy_from_hub(MCN_ID(SENSOR_FILTER_GYR), gyr);
	mcn_copy_from_hub(MCN_ID(SENSOR_FILTER_ACC), acc);
	mcn_copy_from_hub(MCN_ID(SENSOR_FILTER_MAG), mag);
	
	scaled_imu.xacc = (int16_t)(acc[0]*1000.0f);
	scaled_imu.yacc = (int16_t)(acc[1]*1000.0f);
	scaled_imu.zacc = (int16_t)(acc[2]*1000.0f);
	scaled_imu.xgyro = (int16_t)(gyr[0]*1000.0f);
	scaled_imu.ygyro = (int16_t)(gyr[1]*1000.0f);
	scaled_imu.zgyro = (int16_t)(gyr[2]*1000.0f);
	scaled_imu.xmag = (int16_t)(mag[0]*1000.0f);
	scaled_imu.ymag = (int16_t)(mag[1]*1000.0f);
	scaled_imu.zmag = (int16_t)(mag[2]*1000.0f);
	
	mavlink_msg_scaled_imu_encode(mavlink_system.sysid, mavlink_system.compid, msg_t, &scaled_imu);
}

void mavproxy_msg_attitude_pack(mavlink_message_t *msg_t)
{
	mavlink_attitude_t attitude;
	uint16_t len;

	Euler e;
	mcn_copy_from_hub(MCN_ID(ATT_EULER), &e);
	float gyr[3];
	mcn_copy_from_hub(MCN_ID(SENSOR_FILTER_GYR), gyr);
	
	attitude.roll = e.roll;
	attitude.pitch = e.pitch;
	attitude.yaw = e.yaw;
	attitude.rollspeed = gyr[0];
	attitude.pitchspeed = gyr[1];
	attitude.yawspeed = gyr[2];
	
	mavlink_msg_attitude_encode(mavlink_system.sysid, mavlink_system.compid, msg_t, &attitude);
}

void mavproxy_msg_altitude_pack(mavlink_message_t *msg_t)
{
	mavlink_altitude_t altitude;
	uint16_t len;

	Altitude_Info alt_info;
	mcn_copy_from_hub(MCN_ID(ALT_INFO), &alt_info);
	
	altitude.altitude_monotonic = 0;
	altitude.altitude_amsl = alt_info.alt;
	altitude.altitude_local = 0;
	altitude.altitude_relative = alt_info.relative_alt;
	altitude.altitude_terrain = 0;
	
	mavlink_msg_altitude_encode(mavlink_system.sysid, mavlink_system.compid, msg_t, &altitude);
}

void mavproxy_msg_global_position_pack(mavlink_message_t *msg_t)
{
	mavlink_global_position_int_t global_position;
	uint16_t len;

	struct vehicle_gps_position_s gps_pos_t;
	mcn_copy_from_hub(MCN_ID(GPS_POSITION), &gps_pos_t);
	Altitude_Info alt_info;
	mcn_copy_from_hub(MCN_ID(ALT_INFO), &alt_info);
	
	global_position.lat 			= gps_pos_t.lat;
	global_position.lon 			= gps_pos_t.lon;
	global_position.alt 			= alt_info.alt*1e3;
	global_position.relative_alt 	= alt_info.relative_alt*1e3;
	global_position.vx				= gps_pos_t.vel_n_m_s*1e2;
	global_position.vy 				= gps_pos_t.vel_e_m_s*1e2;
	global_position.vz				= alt_info.vz*1e2;
	
	mavlink_msg_global_position_int_encode(mavlink_system.sysid, mavlink_system.compid, msg_t, &global_position);
}

void mavproxy_msg_gps_raw_int_pack(mavlink_message_t *msg_t)
{
	mavlink_gps_raw_int_t gps_raw_int;
	uint16_t len;

	struct vehicle_gps_position_s gps_pos_t;
	int res = mcn_copy_from_hub(MCN_ID(GPS_POSITION), &gps_pos_t);
	
	if(res == 0){
		gps_raw_int.lat = gps_pos_t.lat;
		gps_raw_int.lon = gps_pos_t.lon;
		gps_raw_int.alt = gps_pos_t.alt;
		gps_raw_int.eph = gps_pos_t.eph*1e3;
		gps_raw_int.epv = gps_pos_t.epv*1e3;
		gps_raw_int.vel = gps_pos_t.vel_m_s*1e2;
		gps_raw_int.cog = Rad2Deg(gps_pos_t.cog_rad)*1e2;
		gps_raw_int.fix_type = gps_pos_t.fix_type;
		gps_raw_int.satellites_visible = gps_pos_t.satellites_used;
	}else{
		gps_raw_int.lat = 0;
		gps_raw_int.lon = 0;
		gps_raw_int.alt = 0;
		gps_raw_int.eph = 9999999;
		gps_raw_int.epv = 9999999;
		gps_raw_int.vel = 0;
		gps_raw_int.cog = 0;
		gps_raw_int.fix_type = 0;
		gps_raw_int.satellites_visible = 0;
	}
	
	mavlink_msg_gps_raw_int_encode(mavlink_system.sysid, mavlink_system.compid, msg_t, &gps_raw_int);
}

void mavproxy_msg_param_pack(mavlink_message_t *msg_t, param_info_t *param)
{
	mavlink_param_value_t param_value;
	uint16_t len = strlen(param->name);
	
	param_value.param_count = mavlink_param_get_info_count() + param_get_info_count();
	param_value.param_index = mavlink_param_get_info_count() + param_get_info_index(param->name);
	memset(param_value.param_id, 0, 16);
	memcpy(param_value.param_id, param->name, len < 16 ? len : 16);
	switch (param->type) {
		case PARAM_TYPE_FLOAT:
			param_value.param_type = MAVLINK_TYPE_FLOAT;
			param_value.param_value = param->val.f;
			break;
		case PARAM_TYPE_INT32:
			param_value.param_type = MAVLINK_TYPE_INT32_T;
			memcpy(&(param_value.param_value), &(param->val.i), sizeof(param->val.i));
			break;
		case PARAM_TYPE_UINT32:
			param_value.param_type = MAVLINK_TYPE_UINT32_T;
			memcpy(&(param_value.param_value), &(param->val.u), sizeof(param->val.u));
			break;
		default:
			param_value.param_type = MAVLINK_TYPE_FLOAT;
			param_value.param_value = param->val.f;
			break;
	}
	
	mavlink_msg_param_value_encode(mavlink_system.sysid, mavlink_system.compid, msg_t, &param_value);
}

void mavproxy_msg_mavlink_param_pack(mavlink_message_t *msg_t, param_t *param)
{
	mavlink_param_value_t param_value;
	uint16_t len = strlen(param->name);
	
	param_value.param_count = mavlink_param_get_info_count() + param_get_info_count();
	param_value.param_index = mavlink_param_get_info_index(param);
	memset(param_value.param_id, 0, 16);
	memcpy(param_value.param_id, param->name, len < 16 ? len : 16);
	if (param->param) {
		switch (param->param->type) {
			case PARAM_TYPE_FLOAT:
				param_value.param_type = MAVLINK_TYPE_FLOAT;
				param_value.param_value = param->value;
				break;
			case PARAM_TYPE_INT32:
				param_value.param_type = MAVLINK_TYPE_INT32_T;
				memcpy(&(param_value.param_value), &(param->value), sizeof(param->value));
				break;
			case PARAM_TYPE_UINT32:
				param_value.param_type = MAVLINK_TYPE_UINT32_T;
				memcpy(&(param_value.param_value), &(param->value), sizeof(param->value));
				break;
			default:
				param_value.param_type = MAVLINK_TYPE_FLOAT;
				param_value.param_value = param->value;
				break;
		}
	}else {
		param_value.param_type = MAVLINK_TYPE_FLOAT;
		param_value.param_value = param->value;
	}
	
	mavlink_msg_param_value_encode(mavlink_system.sysid, mavlink_system.compid, msg_t, &param_value);
}

//uint8_t mavlink_send_msg_rc_channels_raw(uint32_t channel[8])
//{
//	mavlink_message_t msg;
//	uint16_t len;
//	
//	if(_mav_disable)
//		return 0;
//	
//	mavlink_msg_rc_channels_raw_pack(mavlink_system.sysid, mavlink_system.compid, &msg,
//						       time_nowUs(), 1, (uint16_t)1000*channel[0], (uint16_t)1000*channel[1], 
//								(uint16_t)1000*channel[2], (uint16_t)1000*channel[3], (uint16_t)1000*channel[4], (uint16_t)1000*channel[5], 
//								(uint16_t)1000*channel[6], (uint16_t)1000*channel[7], 70);
//	
//	len = mavlink_msg_to_send_buffer(mav_tx_buff, &msg);
//	mavlink_msg_transfer(0, mav_tx_buff, len);	
//	
//	return 1;
//}

uint8_t mavlink_send_hil_actuator_control(float control[16], int motor_num)
{
	mavlink_message_t msg;
	uint16_t len;
	
	if(_mav_disable)
		return 0;
	
	mavlink_msg_hil_actuator_controls_pack(mavlink_system.sysid, mavlink_system.compid, &msg,
                               time_nowUs(), control, 0, motor_num);
	
	len = mavlink_msg_to_send_buffer(mav_tx_buff, &msg);
	return mavlink_msg_transfer(0, mav_tx_buff, len);
}

int mavlink_send_single_param(const char *name, mavlink_message_t *msg)
{
	param_info_t *param = NULL;
	param_t *mav_param = NULL;
	mav_param = mavlink_param_get_by_name(name);
	if (mav_param) {
		mavproxy_msg_mavlink_param_pack(msg, mav_param);
		mavproxy_temp_msg_push(msg);
		if (mav_param->param) {
			mavproxy_msg_param_pack(msg, mav_param->param);
			mavproxy_temp_msg_push(msg);
		}
	} else {
		param = param_get_by_name(name);
		if (param) {
			mavproxy_msg_param_pack(msg, param);
			mavproxy_temp_msg_push(msg);
			mav_param = mavlink_param_get_by_info(param);
			if (mav_param) {
				mavproxy_msg_mavlink_param_pack(msg, mav_param);
				mavproxy_temp_msg_push(msg);
			}
		}
	}

	return 0;
}

void mavproxy_send_statustext_msg(mav_status_type status, mavlink_message_t *msg)
{
	mavlink_statustext_t statustext;
	memset(statustext.text, 0, 50);
	statustext.severity = mavlink_get_status_content(status).severity;
	memcpy(statustext.text, mavlink_get_status_content(status).string, strlen(mavlink_get_status_content(status).string));
	mavlink_msg_statustext_encode(mavlink_system.sysid, mavlink_system.compid, msg, &statustext);
	mavproxy_temp_msg_push(msg);
}

void mavlink_send_status(mav_status_type status)
{
	mavlink_message_t msg;

	mavproxy_send_statustext_msg(status, &msg);
}

void mavlink_send_calibration_progress_msg(uint8_t progress)
{
	switch(progress)
	{
		case 0:
			mavlink_send_status(CAL_PROGRESS_0);
			break;
		case 1:
			mavlink_send_status(CAL_PROGRESS_10);
			break;
		case 2:
			mavlink_send_status(CAL_PROGRESS_20);
			break;
		case 3:
			mavlink_send_status(CAL_PROGRESS_30);
			break;
		case 4:
			mavlink_send_status(CAL_PROGRESS_40);
			break;  
		case 5:
			mavlink_send_status(CAL_PROGRESS_50);
			break;
		case 6:
			mavlink_send_status(CAL_PROGRESS_60);
			break;
		case 7:
			mavlink_send_status(CAL_PROGRESS_70);
			break;
		case 8:
			mavlink_send_status(CAL_PROGRESS_80);
			break;
		case 9:
			mavlink_send_status(CAL_PROGRESS_90);
			break;
		default:
			break;
	}
}


static void mavlink_send_command_ack(mavlink_command_ack_t *command_ack, mavlink_message_t *msg)
{
	mavlink_msg_command_ack_encode(mavlink_system.sysid, mavlink_system.compid, msg, command_ack);
	mavproxy_temp_msg_push(msg);
}

static void mavproxy_proc_command(mavlink_command_long_t *command, mavlink_message_t *msg)
{
	switch(command->command) {
		case MAV_CMD_PREFLIGHT_CALIBRATION:
		{
			mavlink_command_ack_t command_ack;
			
			if(command->param1 == 1) {          //calibration gyr
				mavproxy_send_statustext_msg(CAL_START_GYRO, msg);
				gyr_mavlink_calibration_start();
			} else if(command->param2 == 1) {    //calibration mag
				mavproxy_send_statustext_msg(CAL_START_MAG, msg);
				mag_mavlink_calibration_start();
			} else if(command->param5 == 1) {    //calibration acc
				mavproxy_send_statustext_msg(CAL_START_ACC, msg);
				acc_mavlink_calibration_start();
			} else if(command->param5 == 2) {    //calibration level
				//mavproxy_send_statustext_msg(CAL_START_LEVEL, msg);
			}

			command_ack.command = MAV_CMD_PREFLIGHT_CALIBRATION;
			command_ack.result  = MAV_CMD_ACK_OK | MAV_CMD_ACK_ENUM_END;
			mavlink_send_command_ack(&command_ack, msg);
			break;
		}
		default:
			break;
	}
}

static void timer_mavproxy_update(void* parameter)
{
	rt_event_send(&event_mavproxy, EVENT_MAVPROXY_UPDATE);
}

static void mavproxy_send_all_param(void)
{
	rt_event_send(&event_mavproxy, EVENT_MAVPROXY_SEND_ALL_PARAM);
}

void mavproxy_rx_entry(void *param)
{
	mavlink_message_t msg;
	int chan = 0;
	char byte;
	rt_err_t res = RT_EOK;

	while (1) {
		rt_size_t rb = mavlink_lowlevel_read(&byte, 1);
		if(rb <= 0) {
			continue;
		}
		if(mavlink_parse_char(chan, byte, &msg, &mav_status)){
			//Console.print("mav msg:%d\n", msg.msgid);
			/* decode mavlink package */
			switch(msg.msgid){
				case MAVLINK_MSG_ID_HEARTBEAT:
					break;
				case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
				{
					if(mavlink_system.sysid == mavlink_msg_param_request_read_get_target_system(&msg)) {
						mavlink_param_request_read_t request_read;
						mavlink_msg_param_request_read_decode(&msg, &request_read);
						mavlink_send_single_param(request_read.param_id, &msg);
					}
					break;
				}
				case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
				{
					if(mavlink_system.sysid == mavlink_msg_param_request_list_get_target_system(&msg)) {
						mavproxy_send_all_param();
					}
					break;
				}
				case MAVLINK_MSG_ID_PARAM_SET:
				{
					if(mavlink_system.sysid == mavlink_msg_param_set_get_target_system(&msg)) {
						param_info_t* param = NULL;
						mavlink_param_set_t param_set;
						mavlink_msg_param_set_decode(&msg, &param_set);

						mavlink_param_set_value(param_set.param_id, param_set.param_value);
						mavlink_send_single_param(param_set.param_id, &msg);
					}
					break;
				}
				case MAVLINK_MSG_ID_COMMAND_LONG:
				{
					if(mavlink_system.sysid == mavlink_msg_command_long_get_target_system(&msg)) {
						mavlink_command_long_t command;
						mavlink_msg_command_long_decode(&msg, &command);

						mavproxy_proc_command(&command, &msg);
					}
					break;
				}
				case MAVLINK_MSG_ID_SERIAL_CONTROL:
				{
					mavlink_serial_control_t serial_control;
					mavlink_msg_serial_control_decode(&msg, &serial_control);
					
					// the last byte for data is '\0', change to '\r'
					//serial_control.data[serial_control.count] = '\r';	

					for(uint8_t i = 0 ; i < serial_control.count ; i++){
						if(!ringbuffer_putc(_mav_serial_rb, serial_control.data[i])) break;
					}

					mavproxy_console_proc(serial_control.count);
					break;
				}
				case MAVLINK_MSG_ID_HIL_SENSOR:
				{
					mavlink_hil_sensor_t hil_sensor;
					mavlink_msg_hil_sensor_decode(&msg, &hil_sensor);
					/* publish */
					mcn_publish(MCN_ID(HIL_SENSOR), &hil_sensor);
				}break;
				case MAVLINK_MSG_ID_HIL_GPS:
				{
					mavlink_hil_gps_t hil_gps;
					mavlink_msg_hil_gps_decode(&msg, &hil_gps);
					//Console.print("lat:%f, vn:%f eph:%f\n", (double)hil_gps.lat*1e-7, (float)hil_gps.vn*1e-2, (float)hil_gps.eph*1e-2);
					
					struct vehicle_gps_position_s gps_position;
					gps_position.lat = hil_gps.lat;
					gps_position.lon = hil_gps.lon;
					gps_position.alt = hil_gps.alt;
					gps_position.eph = (float)hil_gps.eph*1e-2;
					gps_position.epv = (float)hil_gps.epv*1e-2;
					gps_position.vel_m_s = (float)hil_gps.vel*1e-2;
					gps_position.vel_n_m_s = (float)hil_gps.vn*1e-2;
					gps_position.vel_e_m_s = (float)hil_gps.ve*1e-2;
					gps_position.vel_d_m_s = (float)hil_gps.vd*1e-2;
					gps_position.fix_type = hil_gps.fix_type;
					gps_position.satellites_used = hil_gps.satellites_visible;
					uint32_t now = time_nowMs();
					gps_position.timestamp_position = gps_position.timestamp_velocity = now;
					
					mcn_publish(MCN_ID(GPS_POSITION), &gps_position);
				}break;
				case MAVLINK_MSG_ID_HIL_STATE_QUATERNION:
				{
					mavlink_hil_state_quaternion_t	hil_state_q;
					mavlink_msg_hil_state_quaternion_decode(&msg, &hil_state_q);
					/* publish */
					mcn_publish(MCN_ID(HIL_STATE_Q), &hil_state_q);
				}break;
				default :
				{
					//Console.print("mav unknown msg:%d\n", msg.msgid);
				}break;
			}
		}
	}

}


int handle_mavproxy_shell_cmd(int argc, char** argv)
{
	if(argc > 1){
		if(strcmp(argv[1], "send") == 0){
			if(argc < 3 )
				return 1;
			if(strcmp(argv[2], "hil_actuator_control") == 0){
				float cocntrol[16] = {0.0f};
				int motor_num = argc - 3;
				Console.print("hil_actuator_control motor num:%d throttle:", motor_num);
				for(int i = 0 ; i < motor_num ; i++){
					cocntrol[i] = atof(argv[3+i]);
					Console.print("%f ", cocntrol[i]);
				}
				Console.print("\n");
				mavlink_send_hil_actuator_control(cocntrol, motor_num);
			}
		}
	}
	
	return 0;
}

uint8_t mavproxy_period_msg_register(uint8_t msgid, uint16_t period_ms, void (* msg_pack_cb)(mavlink_message_t *msg_t), uint8_t enable)
{
	MAV_PeriodMsg msg;
	
	msg.msgid = msgid;
	msg.enable = enable;
	msg.period = period_ms;
	msg.msg_pack_cb = msg_pack_cb;
	msg.time_stamp = 0;
	
	if(_period_msg_queue.size < MAX_PERIOD_MSG_QUEUE_SIZE){
		
		_period_msg_queue.queue[_period_msg_queue.size++] = msg;
		return 1;
	}else{
		
		Console.print("mavproxy period msg queue is full\n");
		return 0;
	}
}

uint8_t mavproxy_temp_msg_push(mavlink_message_t *msg)
{
	if(_mav_disable)
		return 0;
	
	if( (_temp_msg_queue.head+1) % MAX_TEMP_MSG_QUEUE_SIZE == _temp_msg_queue.tail ){

		return 0;
	}
	
	_temp_msg_queue.queue[_temp_msg_queue.head] = *msg;
	_temp_msg_queue.head = (_temp_msg_queue.head+1) % MAX_TEMP_MSG_QUEUE_SIZE;
	
	//printf("temp push msg\n");
	
	return 1;
}

uint8_t mavproxy_send_out_msg(mavlink_message_t msg)
{
	if(_mav_disable)
		return 0;
	
	//uint8_t tx_buff[sizeof(mavlink_message_t)];
	uint16_t len;

	len = mavlink_msg_to_send_buffer(mav_tx_buff2, &msg);
	
	return mavlink_msg_transfer(0, mav_tx_buff2, len);
}

static void param_send_all(param_info_t* param)
{
	uint16_t len;
	mavlink_message_t msg;
	/* Console.print("param send:%s\n", param->name); */

	mavproxy_msg_param_pack(&msg, param);

	len = mavlink_msg_to_send_buffer(mav_tx_buff, &msg);
	mavlink_msg_transfer(0, mav_tx_buff, len);
}

static void mavlink_param_send_all(void)
{
	uint16_t len;
	param_t *param;
	mavlink_message_t msg;
	/* Console.print("param send:%s\n", param->name); */

	for (uint32_t cnt = 0; cnt < mavlink_param_get_info_count(); cnt++) {
		param = mavlink_param_get_info_by_index(cnt);
		if (!param) {
			break;
		}
		mavproxy_msg_mavlink_param_pack(&msg, param);

		len = mavlink_msg_to_send_buffer(mav_tx_buff, &msg);
		mavlink_msg_transfer(0, mav_tx_buff, len);
	}
}

uint8_t mavproxy_try_send_param_msg(void)
{
	mavlink_param_send_all();
	param_traverse(param_send_all);

	return 1;
}

uint8_t mavproxy_try_send_temp_msg(void)
{
	if(_temp_msg_queue.head == _temp_msg_queue.tail){
		//queue is empty
		return 0;
	}
	
	uint16_t len;

	len = mavlink_msg_to_send_buffer(mav_tx_buff, &_temp_msg_queue.queue[_temp_msg_queue.tail]);
	_temp_msg_queue.tail = (_temp_msg_queue.tail+1) % MAX_TEMP_MSG_QUEUE_SIZE;
	
	mavlink_msg_transfer(0, mav_tx_buff, len);

	return 1;
}

uint8_t mavproxy_try_send_period_msg(void)
{
	if(_mav_disable)
		return 0;
	
	for(uint16_t i = 0 ; i < _period_msg_queue.size ; i++){
		
		uint32_t now = time_nowMs();
		MAV_PeriodMsg *msg_t = &_period_msg_queue.queue[_period_msg_queue.index];
		_period_msg_queue.index = (_period_msg_queue.index+1) % _period_msg_queue.size;
		
		// find next msg to be sent
		if(now - msg_t->time_stamp >= msg_t->period && msg_t->enable){
			uint16_t len;
			msg_t->time_stamp = now;
			// pack msg
			mavlink_message_t msg;
			msg_t->msg_pack_cb(&msg);
			// send out msg
			len = mavlink_msg_to_send_buffer(mav_tx_buff, &msg);
			mavlink_msg_transfer(0, mav_tx_buff, len);
			
			return 1;
		}
	}
	
	return 0;
}

uint8_t mavproxy_msg_serial_control_send(uint8_t *data, uint8_t count)
{
	mavlink_serial_control_t serial_control;
	mavlink_message_t msg;
	
	serial_control.baudrate = 0;
	serial_control.count = count<=70 ? count : 70;
	serial_control.device = SERIAL_CONTROL_DEV_SHELL;
	serial_control.flags = SERIAL_CONTROL_FLAG_REPLY;

	memcpy(serial_control.data, data, serial_control.count);

	mavlink_msg_serial_control_encode(mavlink_system.sysid, mavlink_system.compid, &msg, &serial_control);

	return mavproxy_send_out_msg(msg);
}

uint16_t mavproxy_msg_serial_control_read(uint8_t *data, uint16_t size)
{
	uint16_t len;
	len = ringbuffer_getlen(_mav_serial_rb);
	if (!len)
		return 0;
	size = size > len ? len : size;
	return ringbuffer_get(_mav_serial_rb, data, size);
}

void mavproxy_msg_queue_init(void)
{
	_period_msg_queue.size = 0;
	_period_msg_queue.index = 0;
	
	_temp_msg_queue.head = 0;
	_temp_msg_queue.tail = 0;
}

void mavproxy_gps_status_cb(void *parameter)
{
	GPS_Status gps_status = *((GPS_Status*)parameter);
	if(gps_status.status == GPS_UNDETECTED){
		//TODO, deregister
	}else{
		mavproxy_period_msg_register(MAVLINK_MSG_ID_GPS_RAW_INT, 100, mavproxy_msg_gps_raw_int_pack, 1);
	}
}

void mavproxy_entry(void *parameter)
{
	rt_err_t res;
	rt_uint32_t recv_set = 0;
	rt_uint32_t wait_set = EVENT_MAVPROXY_UPDATE | EVENT_MAVPROXY_SEND_ALL_PARAM;

	mavlink_param_init();
	mavproxy_lowlevel_init();
	_mav_serial_rb = ringbuffer_static_create(_mav_serial_buffer, MAV_SERIAL_BUFFER_SIZE);

	/* create event */
	res = rt_event_init(&event_mavproxy, "mavproxy", RT_IPC_FLAG_FIFO);

	res = rt_thread_init(&thread_mavlink_rx_handle,
						   "mavproxy_rx",
						   mavproxy_rx_entry,
						   RT_NULL,
						   &thread_mavlink_rx_stack[0],
						   sizeof(thread_mavlink_rx_stack), MAVLINK_RX_THREAD_PRIORITY, 5);
	if (res == RT_EOK)
		rt_thread_startup(&thread_mavlink_rx_handle);
	
	/* register timer event */
	rt_timer_init(&timer_mavproxy, "mavproxy",
					timer_mavproxy_update,
					RT_NULL,
					10,
					RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
	rt_timer_start(&timer_mavproxy);
	
	/* advertise HIL publisher */
	int mcn_res;
	mcn_res = mcn_advertise(MCN_ID(HIL_STATE_Q));
	if(mcn_res != 0){
		Console.e(TAG, "err:%d, HIL_STATE_Q advertise fail!\n", mcn_res);
	}
	mcn_res = mcn_advertise(MCN_ID(HIL_SENSOR));
	if(mcn_res != 0){
		Console.e(TAG, "err:%d, HIL_SENSOR advertise fail!\n", mcn_res);
	}
	mcn_res = mcn_advertise(MCN_ID(HIL_GPS));
	if(mcn_res != 0){
		Console.e(TAG, "err:%d, HIL_GPS advertise fail!\n", mcn_res);
	}
	
	mavproxy_msg_queue_init();
	// register periodical mavlink msg
	mavproxy_period_msg_register(MAVLINK_MSG_ID_HEARTBEAT, 1000, mavproxy_msg_heartbeat_pack, 1);
	mavproxy_period_msg_register(MAVLINK_MSG_ID_SYS_STATUS, 1000, mavproxy_msg_sys_status_pack, 1);
	mavproxy_period_msg_register(MAVLINK_MSG_ID_SCALED_IMU, 50, mavproxy_msg_scaled_imu_pack, 1);
	mavproxy_period_msg_register(MAVLINK_MSG_ID_ATTITUDE, 100, mavproxy_msg_attitude_pack, 1);
	mavproxy_period_msg_register(MAVLINK_MSG_ID_ALTITUDE, 100, mavproxy_msg_altitude_pack, 1);
	//mavproxy_period_msg_register(MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 100, mavproxy_msg_global_position_pack, 1);

	_gps_status_node_t = mcn_subscribe(MCN_ID(GPS_STATUS), mavproxy_gps_status_cb);
	
	while(1)
	{
		/* wait event occur */
		res = rt_event_recv(&event_mavproxy, wait_set, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, 
								RT_WAITING_FOREVER, &recv_set);

		if(res == RT_EOK)
		{
			if (recv_set & EVENT_MAVPROXY_SEND_ALL_PARAM) {
				mavproxy_try_send_param_msg();
			}
			if (recv_set & EVENT_MAVPROXY_UPDATE) {
				// try to send out temporary msg first
				mavproxy_try_send_temp_msg();
				// try to send out periodical msg
				mavproxy_try_send_period_msg();
			}
		}
		else
		{
			//some err happen
			Console.e(TAG, "mavproxy loop, err:%d\r\n" , res);
		}
	}
}

