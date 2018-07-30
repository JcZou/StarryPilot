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

#define MAVLINK_USE_USB

#define EVENT_MAV_1HZ_UPDATE		(1<<0)
#define EVENT_MAV_3HZ_UPDATE		(1<<1)

#define MAV_PKG_RETRANSMIT
#define MAX_RETRY_NUM				5

static rt_device_t usb_device = NULL;
uint8_t mav_tx_buff[1024];
mavlink_system_t mavlink_system;
/* disable mavlink sending */
uint8_t mav_disenable = 0;

static char *TAG = "Mavproxy";

static struct rt_timer timer_1HZ;
static struct rt_timer timer_3HZ;

static struct rt_event event_mavlink;

extern rt_device_t _console_device;
mavlink_status_t mav_status;

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

static McnNode_t gyr_node_t;
static McnNode_t acc_node_t;
static McnNode_t mag_node_t;
static McnNode_t baro_node_t;
static McnNode_t alt_node_t;
static McnNode_t lidar_node_t;
static McnNode_t cor_lidar_node_t;

uint8_t mavlink_msg_transfer(uint8_t chan, uint8_t* msg_buff, uint16_t len)
{
	uint16_t s_bytes;
	
	if(usb_device){
		s_bytes = rt_device_write(usb_device, 0, (void*)msg_buff, len);
#ifdef MAV_PKG_RETRANSMIT
		uint8_t retry = 0;
		while(retry < MAX_RETRY_NUM && s_bytes != len){
			rt_thread_delay(1);
			s_bytes = rt_device_write(usb_device, 0, (void*)msg_buff, len);
			retry++;
		}
#endif
	}else{
		s_bytes = rt_device_write(_console_device, 0, msg_buff, len);
	}
	
	if(s_bytes == len)
		return 0;
	else
		return 1;
}

rt_err_t device_mavproxy_init(void)
{
	mavlink_system.sysid = 20;                   
	mavlink_system.compid = MAV_COMP_ID_IMU;     

	return 0;
}

uint8_t mavlink_send_msg_heartbeat(uint8_t system_status)
{
	mavlink_message_t msg;
	uint16_t len;
	
	if(mav_disenable)
		return 0;
	
	mavlink_msg_heartbeat_pack(mavlink_system.sysid, mavlink_system.compid, &msg,
						       MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC, 0xFE, 0, system_status);
	
	len = mavlink_msg_to_send_buffer(mav_tx_buff, &msg);
	mavlink_msg_transfer(0, mav_tx_buff, len);
	
	return 1;
}

uint8_t mavlink_send_msg_sys_status(uint8_t system_status)
{
	mavlink_message_t msg;
	uint16_t len;
	
	if(mav_disenable)
		return 0;

	mavlink_msg_sys_status_pack(mavlink_system.sysid, mavlink_system.compid, &msg,
						       1, 1, 1, get_cpu_usage(), 1100, 1100, 0, 0, 0, 0, 0, 0, 0);
	
	len = mavlink_msg_to_send_buffer(mav_tx_buff, &msg);
	mavlink_msg_transfer(0, mav_tx_buff, len);
	
	return 1;
}

uint8_t mavlink_send_msg_attitude_quaternion(uint8_t system_status, quaternion attitude)
{
	mavlink_message_t msg;
	uint16_t len;
	
	if(mav_disenable)
		return 0;
	
	mavlink_msg_attitude_quaternion_pack(mavlink_system.sysid, mavlink_system.compid, &msg,
						       0, attitude.w, attitude.x, attitude.y, attitude.z,0,0,0);
	
	len = mavlink_msg_to_send_buffer(mav_tx_buff, &msg);
	mavlink_msg_transfer(0, mav_tx_buff, len);	
	
	return 1;
}

uint8_t mavlink_send_msg_attitude(uint8_t system_status)
{
	mavlink_message_t msg;
	uint16_t len;
	
	if(mav_disenable)
		return 0;

	Euler att;
	mcn_copy_from_hub(MCN_ID(ATT_EULER), &att);
	
	float gyr[3];
	mcn_copy(MCN_ID(SENSOR_FILTER_GYR), gyr_node_t, gyr);
	
	mavlink_msg_attitude_pack(mavlink_system.sysid, mavlink_system.compid, &msg,
						       0, att.roll, att.pitch, att.yaw, gyr[0], gyr[1], gyr[2]);
	
	len = mavlink_msg_to_send_buffer(mav_tx_buff, &msg);
	mavlink_msg_transfer(0, mav_tx_buff, len);	
	
	return 1;
}

uint8_t mavlink_send_msg_global_position(uint8_t system_status)
{
	mavlink_message_t msg;
	uint16_t len;
	Position_Info pos_info;
	AltInfo alt_info;
	
	if(mav_disenable)
		return 0;
	
	pos_info = get_pos_info();
	mcn_copy(MCN_ID(ALT_INFO), alt_node_t, &alt_info);
	
	mavlink_msg_global_position_int_pack(mavlink_system.sysid, mavlink_system.compid, &msg,
						       0, 0, 0, (int32_t)(alt_info.alt*1000.0f), (int32_t)(alt_info.relative_alt*1000.0f), 
								0, 0, (int16_t)(alt_info.vz*100.0f), UINT16_MAX);
	
	len = mavlink_msg_to_send_buffer(mav_tx_buff, &msg);
	mavlink_msg_transfer(0, mav_tx_buff, len);	
	
	return 1;
}

uint8_t mavlink_send_altitude(void)
{
	mavlink_message_t msg;
	uint16_t len;
	mavlink_altitude_t alt;
	
	if(mav_disenable)
		return 0;

	AltInfo alt_info;
	mcn_copy(MCN_ID(ALT_INFO), alt_node_t, &alt_info);
	float lidar_dis;
	mcn_copy(MCN_ID(SENSOR_LIDAR), lidar_node_t, &lidar_dis);
	float cor_lidar_dis;
	mcn_copy(MCN_ID(CORRECT_LIDAR), cor_lidar_node_t, &cor_lidar_dis);
	
	alt.time_usec = 0;
	alt.altitude_monotonic = 0.0f;
	alt.altitude_amsl = cor_lidar_dis;;
	alt.altitude_local = alt_info.alt;
	alt.altitude_relative = alt_info.relative_alt;
	alt.altitude_terrain = lidar_dis; 
	alt.bottom_clearance = 0.0f; 

	mavlink_msg_altitude_encode(mavlink_system.sysid, mavlink_system.compid, &msg, &alt);
	
	len = mavlink_msg_to_send_buffer(mav_tx_buff, &msg);
	mavlink_msg_transfer(0, mav_tx_buff, len);	
	
	return 1;
}

uint8_t mavlink_send_hil_imu(void)
{
	mavlink_message_t msg;
	uint16_t len;
	mavlink_hil_sensor_t hil_imu;
	
	if(mav_disenable)
		return 0;
	
	float gyr[3], acc[3], mag[3];
	mcn_copy(MCN_ID(SENSOR_FILTER_GYR), gyr_node_t, gyr);
	mcn_copy(MCN_ID(SENSOR_FILTER_ACC), acc_node_t, acc);
	mcn_copy(MCN_ID(SENSOR_FILTER_MAG), mag_node_t, mag);
	MS5611_REPORT_Def baro_report;
	mcn_copy(MCN_ID(SENSOR_BARO), baro_node_t, &baro_report);

	hil_imu.time_usec = 0;
	hil_imu.xacc = acc[0]; 
	hil_imu.yacc = acc[1]; 
	hil_imu.zacc = acc[2]; 
	hil_imu.xgyro = gyr[0];
	hil_imu.ygyro = gyr[1];
	hil_imu.zgyro = gyr[2];
	hil_imu.xmag = mag[0]; 
	hil_imu.ymag = mag[1]; 
	hil_imu.zmag = mag[2]; 
	hil_imu.abs_pressure = baro_report.pressure; 
	hil_imu.diff_pressure = baro_report.pressure;
	hil_imu.pressure_alt = baro_report.altitude; 
	hil_imu.temperature = baro_report.temperature;
	
	mavlink_msg_hil_sensor_encode(mavlink_system.sysid, mavlink_system.compid, &msg, &hil_imu);
	
	len = mavlink_msg_to_send_buffer(mav_tx_buff, &msg);
	mavlink_msg_transfer(0, mav_tx_buff, len);	
	
	return 1;
}

uint8_t mavlink_send_imu(void)
{
	mavlink_message_t msg;
	uint16_t len;
	mavlink_scaled_imu_t imu;
	
	if(mav_disenable)
		return 0;
	
	float gyr[3], acc[3], mag[3];
	mcn_copy(MCN_ID(SENSOR_FILTER_GYR), gyr_node_t, gyr);
	mcn_copy(MCN_ID(SENSOR_FILTER_ACC), acc_node_t, acc);
	mcn_copy(MCN_ID(SENSOR_FILTER_MAG), mag_node_t, mag);
	
	imu.xacc = (int16_t)(acc[0]*1000.0f);
	imu.yacc = (int16_t)(acc[1]*1000.0f);
	imu.zacc = (int16_t)(acc[2]*1000.0f);
	
	imu.xgyro = (int16_t)(gyr[0]*1000.0f);
	imu.ygyro = (int16_t)(gyr[1]*1000.0f);
	imu.zgyro = (int16_t)(gyr[2]*1000.0f);
	
	imu.xmag = (int16_t)(mag[0]*1000.0f);
	imu.ymag = (int16_t)(mag[1]*1000.0f);
	imu.zmag = (int16_t)(mag[2]*1000.0f);
	
	mavlink_msg_scaled_imu_encode(mavlink_system.sysid, mavlink_system.compid, &msg, &imu);
	
	len = mavlink_msg_to_send_buffer(mav_tx_buff, &msg);
	mavlink_msg_transfer(0, mav_tx_buff, len);	
	
	return 1;
}

uint8_t mavlink_send_msg_rc_channels_raw(uint32_t channel[8])
{
	mavlink_message_t msg;
	uint16_t len;
	
	if(mav_disenable)
		return 0;
	
	mavlink_msg_rc_channels_raw_pack(mavlink_system.sysid, mavlink_system.compid, &msg,
						       time_nowUs(), 1, (uint16_t)1000*channel[0], (uint16_t)1000*channel[1], 
								(uint16_t)1000*channel[2], (uint16_t)1000*channel[3], (uint16_t)1000*channel[4], (uint16_t)1000*channel[5], 
								(uint16_t)1000*channel[6], (uint16_t)1000*channel[7], 70);
	
	len = mavlink_msg_to_send_buffer(mav_tx_buff, &msg);
	mavlink_msg_transfer(0, mav_tx_buff, len);	
	
	return 1;
}

uint8_t mavlink_send_hil_actuator_control(float control[16], int motor_num)
{
	mavlink_message_t msg;
	uint16_t len;
	
	if(mav_disenable)
		return 0;
	
	mavlink_msg_hil_actuator_controls_pack(mavlink_system.sysid, mavlink_system.compid, &msg,
                               time_nowUs(), control, 0, motor_num);
	
	len = mavlink_msg_to_send_buffer(mav_tx_buff, &msg);
	return mavlink_msg_transfer(0, mav_tx_buff, len);
}

static void timer_mavlink_1HZ_update(void* parameter)
{
	rt_event_send(&event_mavlink, EVENT_MAV_1HZ_UPDATE);
}

static void timer_mavlink_3HZ_update(void* parameter)
{
	rt_event_send(&event_mavlink, EVENT_MAV_3HZ_UPDATE);
}

rt_err_t mavproxy_recv_ind(rt_device_t dev, rt_size_t size)
{
	mavlink_message_t msg;
	int chan = 0;
	char byte;
	rt_err_t res = RT_EOK;
	
	for(int i = 0 ; i < size ; i++){
		rt_size_t rb = rt_device_read(dev, 0, &byte, 1);
		if(!rb){
			Console.e(TAG, "mavlink read byte err\n");
			res = RT_ERROR;
			break;
		}
		if(mavlink_parse_char(chan, byte, &msg, &mav_status)){
			//Console.print("%d\n", msg.seq);
			/* decode mavlink package */
			switch(msg.msgid){
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
					break;
			}
		}
	}
	
	return res;
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

void mavproxy_entry(void *parameter)
{
	rt_err_t res;
	rt_uint32_t recv_set = 0;
	rt_uint32_t wait_set = EVENT_MAV_1HZ_UPDATE | EVENT_MAV_3HZ_UPDATE;

	/* create event */
	res = rt_event_init(&event_mavlink, "mavlink_event", RT_IPC_FLAG_FIFO);

#ifdef MAVLINK_USE_USB	
	usb_device = rt_device_find("usb");
	if(usb_device == NULL)
		Console.e(TAG, "err not find usb device\n");
	else{
		rt_device_open(usb_device , RT_DEVICE_OFLAG_RDWR);
		/* set receive indicate function */
		rt_err_t err = rt_device_set_rx_indicate(usb_device, mavproxy_recv_ind);
		if(err != RT_EOK)
			Console.e(TAG, "set mavlink receive indicate err:%d\n", err);
	}
#endif
	
	/* register timer event */
	rt_timer_init(&timer_1HZ, "timer_1HZ",
					timer_mavlink_1HZ_update,
					RT_NULL,
					1000,
					RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
	rt_timer_start(&timer_1HZ);
	
	rt_timer_init(&timer_3HZ, "timer_3HZ",
					timer_mavlink_3HZ_update,
					RT_NULL,
					100,
					RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
	rt_timer_start(&timer_3HZ);
	
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
	
	gyr_node_t = mcn_subscribe(MCN_ID(SENSOR_FILTER_GYR), NULL);
	if(gyr_node_t == NULL)
		Console.e(TAG, "gyr subscribe err\n");
	acc_node_t = mcn_subscribe(MCN_ID(SENSOR_FILTER_ACC), NULL);
	if(acc_node_t == NULL)
		Console.e(TAG, "acc subscribe err\n");
	mag_node_t = mcn_subscribe(MCN_ID(SENSOR_FILTER_MAG), NULL);
	if(mag_node_t == NULL)
		Console.e(TAG, "mag subscribe err\n");
	baro_node_t = mcn_subscribe(MCN_ID(SENSOR_BARO), NULL);
	if(baro_node_t == NULL)
		Console.e(TAG, "baro subscribe err\n");
	alt_node_t = mcn_subscribe(MCN_ID(ALT_INFO), NULL);
	if(alt_node_t == NULL)
		Console.e(TAG, "alt info subscribe err\n");
	lidar_node_t = mcn_subscribe(MCN_ID(SENSOR_LIDAR), NULL);
	if(lidar_node_t == NULL)
		Console.e(TAG, "lidar subscribe err\n");
	cor_lidar_node_t = mcn_subscribe(MCN_ID(CORRECT_LIDAR), NULL);
	if(cor_lidar_node_t == NULL)
		Console.e(TAG, "correct lidar subscribe err\n");
	
	while(1)
	{
		/* wait event occur */
		res = rt_event_recv(&event_mavlink, wait_set, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, 
								RT_WAITING_FOREVER, &recv_set);

		if(res == RT_EOK)
		{
#ifdef HIL_SIMULATION
//			if(recv_set & EVENT_MAV_3HZ_UPDATE)
//			{
//				mavlink_send_msg_attitude(MAV_STATE_STANDBY);
//				mavlink_send_imu();
//				mavlink_send_msg_global_position(MAV_STATE_STANDBY);
//				mavlink_send_altitude();
//			}
			
			if(recv_set & EVENT_MAV_1HZ_UPDATE)
			{
				mavlink_send_msg_heartbeat(MAV_STATE_UNINIT);	
				mavlink_send_msg_sys_status(MAV_STATE_UNINIT);
			}
			
			if(recv_set & EVENT_MAV_3HZ_UPDATE)
			{
				//mavlink_send_msg_attitude_quaternion(MAV_STATE_STANDBY, attitude_est_get_quaternion());
				mavlink_send_msg_attitude(MAV_STATE_STANDBY);
				//mavlink_send_imu();
				mavlink_send_hil_imu();
				mavlink_send_msg_global_position(MAV_STATE_STANDBY);
				mavlink_send_altitude();
			}
#else
			if(recv_set & EVENT_MAV_1HZ_UPDATE)
			{
				mavlink_send_msg_heartbeat(MAV_STATE_STANDBY);	
			}
			
			if(recv_set & EVENT_MAV_3HZ_UPDATE)
			{
				//mavlink_send_msg_attitude_quaternion(MAV_STATE_STANDBY, attitude_est_get_quaternion());
				mavlink_send_msg_attitude(MAV_STATE_STANDBY);
				mavlink_send_imu();
				mavlink_send_msg_global_position(MAV_STATE_STANDBY);
				mavlink_send_altitude();
			}
#endif
		}
		else
		{
			//some err happen
			Console.e(TAG, "mavlink loop, err:%d\r\n" , res);
		}
	}
}

