/*****************************************************************************
Copyright (c) 2018, StarryPilot Development Team. All rights reserved.

Author: Jiachi Zou

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of StarryPilot nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

#include "fast_loop.h"
#include "sensor_manager.h"
#include "filter.h"
#include "hil_interface.h"
#include "console.h"
#include "systime.h"
#include "uMcn.h"
#include "INS.h"
#include "logger.h"
#include "param.h"
#include "quaternion.h"
#include "file_manager.h"
#include "logger.h"

#define EVENT_FAST_LOOP				(1<<0)
#define INS_STEP_PERIOD				2
#define CONTROL_STEP_PERIOD			2
#define MAG_READ_PERIOD				10
#define BARO_READ_PERIOD			10
#define GPS_READ_PERIOD				100

static struct rt_timer timer_fastloop;
static struct rt_event event_fastloop;

static uint16_t _ins_cnt = INS_STEP_PERIOD;
static uint16_t _mag_cnt = MAG_READ_PERIOD;
static uint16_t _baro_cnt = BARO_READ_PERIOD;
static uint16_t _gps_cnt = GPS_READ_PERIOD;

static McnNode_t gps_node_t;

MCN_DEFINE(ATT_EULER, 3 * sizeof(float));
MCN_DEFINE(ROT_RAD, 3 * sizeof(float));
MCN_DEFINE(POSITION, sizeof(position_int_t));
MCN_DEFINE(VELOCITY, sizeof(velocity_int_t));

MCN_DECLARE(uBLOX_PVT);

static char* TAG = "Main Loop";

static void timer_fastloop_update(void* parameter)
{
	rt_event_send(&event_fastloop, EVENT_FAST_LOOP);
}

void fast_loop(void)
{
	if(_mag_cnt >= MAG_READ_PERIOD) {
		_mag_cnt = 0;

		if(sensor_mag_measure(INS_U.Mag.mag_ga_B) == RT_EOK) {
			INS_U.Mag.timestamp_ms = time_nowMs();
		} else {
			Console.print("fail to get mag data\n");
		}

		log_push_msg((uint8_t*)&INS_U.Mag, 0x02, sizeof(INS_U.Mag));
	}

	if(_baro_cnt >= BARO_READ_PERIOD) {
		_baro_cnt = 0;

		if(sensor_baro_update()) {
			MS5611_REPORT_Def* baro_report = sensor_baro_get_report();

			INS_U.Baro.pressure_Pa = baro_report->pressure;
			INS_U.Baro.temperature_deg = baro_report->temperature;
			INS_U.Baro.timestamp_ms = time_nowMs();

			log_push_msg((uint8_t*)&INS_U.Baro, 0x03, sizeof(INS_U.Baro));
		}
	}

	if(_gps_cnt >= GPS_READ_PERIOD) {
		_gps_cnt = 0;

		if(mcn_poll(gps_node_t)) {
			mcn_copy(MCN_ID(uBLOX_PVT), gps_node_t, &INS_U.GPS_uBlox);
			INS_U.GPS_uBlox.timestamp_ms = time_nowMs();
		}

		log_push_msg((uint8_t*)&INS_U.GPS_uBlox, 0x04, sizeof(INS_U.GPS_uBlox));
	}


	if(_ins_cnt >= INS_STEP_PERIOD) {
		_ins_cnt = 0;

		if(sensor_gyr_measure(INS_U.IMU1.gyr_radPs_B) == RT_EOK && sensor_acc_measure(INS_U.IMU1.acc_mPs2_B) == RT_EOK) {
			INS_U.IMU1.timestamp_ms = time_nowMs();
		} else {
			Console.print("fail to get imu data\n");
		}

		INS_step();

		mcn_publish(MCN_ID(ATT_EULER), INS_Y.INS_Out.euler);
		mcn_publish(MCN_ID(ROT_RAD), INS_Y.INS_Out.rot_radPs_B);
		mcn_publish(MCN_ID(POSITION), (position_int_t*)&INS_Y.INS_Out.lon_1e7_deg);
		mcn_publish(MCN_ID(VELOCITY), (velocity_int_t*)INS_Y.INS_Out.vel_cmPs_O);

		log_push_msg((uint8_t*)&INS_U.IMU1, 0x01, sizeof(INS_U.IMU1));

		INS_Y.INS_Out.timestamp_ms = time_nowMs();
		log_push_msg((uint8_t*)&INS_Y.INS_Out, 0x05, sizeof(INS_Y.INS_Out));
	}

	_ins_cnt++;
	_mag_cnt++;
	_baro_cnt++;
	_gps_cnt++;
}

void fastloop_entry(void* parameter)
{
	rt_err_t res;
	rt_uint32_t recv_set = 0;
	rt_uint32_t wait_set = EVENT_FAST_LOOP;

	/* create event */
	res = rt_event_init(&event_fastloop, "fastloop", RT_IPC_FLAG_FIFO);

	/* register timer event */
	rt_timer_init(&timer_fastloop, "timer_fast",
	              timer_fastloop_update,
	              RT_NULL,
	              1,
	              RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
	rt_timer_start(&timer_fastloop);

	int mcn_res;
	mcn_res = mcn_advertise(MCN_ID(ATT_EULER));

	if(mcn_res != 0) {
		Console.e(TAG, "err:%d, ATT_EULER advertise fail!\n", mcn_res);
	}

	mcn_res = mcn_advertise(MCN_ID(ROT_RAD));

	if(mcn_res != 0) {
		Console.e(TAG, "err:%d, ROT_RAD advertise fail!\n", mcn_res);
	}

	mcn_res = mcn_advertise(MCN_ID(POSITION));

	if(mcn_res != 0) {
		Console.e(TAG, "err:%d, POSITION advertise fail!\n", mcn_res);
	}

	mcn_res = mcn_advertise(MCN_ID(VELOCITY));

	if(mcn_res != 0) {
		Console.e(TAG, "err:%d, VELOCITY advertise fail!\n", mcn_res);
	}

	gps_node_t = mcn_subscribe(MCN_ID(uBLOX_PVT), NULL);

	if(gps_node_t == NULL)
		Console.print("gps_node_t subscribe err\n");

	INS_U.Sensor_Param_j.gyr_rotM[0] = 1;
	INS_U.Sensor_Param_j.gyr_rotM[1] = 0;
	INS_U.Sensor_Param_j.gyr_rotM[2] = 0;
	INS_U.Sensor_Param_j.gyr_rotM[3] = 0;
	INS_U.Sensor_Param_j.gyr_rotM[4] = 1;
	INS_U.Sensor_Param_j.gyr_rotM[5] = 0;
	INS_U.Sensor_Param_j.gyr_rotM[6] = 0;
	INS_U.Sensor_Param_j.gyr_rotM[7] = 0;
	INS_U.Sensor_Param_j.gyr_rotM[8] = 1;

	INS_U.Sensor_Param_j.gyr_bias[0] = PARAM_GET_FLOAT(CALIBRATION, GYR_BIAS_X);
	INS_U.Sensor_Param_j.gyr_bias[1] = PARAM_GET_FLOAT(CALIBRATION, GYR_BIAS_Y);
	INS_U.Sensor_Param_j.gyr_bias[2] = PARAM_GET_FLOAT(CALIBRATION, GYR_BIAS_Z);

	INS_U.Sensor_Param_j.acc_rotM[0] = PARAM_GET_FLOAT(CALIBRATION, ACC_ROT_MAT_1);
	INS_U.Sensor_Param_j.acc_rotM[1] = PARAM_GET_FLOAT(CALIBRATION, ACC_ROT_MAT_2);
	INS_U.Sensor_Param_j.acc_rotM[2] = PARAM_GET_FLOAT(CALIBRATION, ACC_ROT_MAT_3);
	INS_U.Sensor_Param_j.acc_rotM[3] = PARAM_GET_FLOAT(CALIBRATION, ACC_ROT_MAT_4);
	INS_U.Sensor_Param_j.acc_rotM[4] = PARAM_GET_FLOAT(CALIBRATION, ACC_ROT_MAT_5);
	INS_U.Sensor_Param_j.acc_rotM[5] = PARAM_GET_FLOAT(CALIBRATION, ACC_ROT_MAT_6);
	INS_U.Sensor_Param_j.acc_rotM[6] = PARAM_GET_FLOAT(CALIBRATION, ACC_ROT_MAT_7);
	INS_U.Sensor_Param_j.acc_rotM[7] = PARAM_GET_FLOAT(CALIBRATION, ACC_ROT_MAT_8);
	INS_U.Sensor_Param_j.acc_rotM[8] = PARAM_GET_FLOAT(CALIBRATION, ACC_ROT_MAT_9);

	INS_U.Sensor_Param_j.acc_bias[0] = PARAM_GET_FLOAT(CALIBRATION, ACC_BIAS_X);
	INS_U.Sensor_Param_j.acc_bias[1] = PARAM_GET_FLOAT(CALIBRATION, ACC_BIAS_Y);
	INS_U.Sensor_Param_j.acc_bias[2] = PARAM_GET_FLOAT(CALIBRATION, ACC_BIAS_Z);

	INS_U.Sensor_Param_j.mag_rotM[0] = PARAM_GET_FLOAT(CALIBRATION, MAG_ROT_MAT_1);
	INS_U.Sensor_Param_j.mag_rotM[1] = PARAM_GET_FLOAT(CALIBRATION, MAG_ROT_MAT_2);
	INS_U.Sensor_Param_j.mag_rotM[2] = PARAM_GET_FLOAT(CALIBRATION, MAG_ROT_MAT_3);
	INS_U.Sensor_Param_j.mag_rotM[3] = PARAM_GET_FLOAT(CALIBRATION, MAG_ROT_MAT_4);
	INS_U.Sensor_Param_j.mag_rotM[4] = PARAM_GET_FLOAT(CALIBRATION, MAG_ROT_MAT_5);
	INS_U.Sensor_Param_j.mag_rotM[5] = PARAM_GET_FLOAT(CALIBRATION, MAG_ROT_MAT_6);
	INS_U.Sensor_Param_j.mag_rotM[6] = PARAM_GET_FLOAT(CALIBRATION, MAG_ROT_MAT_7);
	INS_U.Sensor_Param_j.mag_rotM[7] = PARAM_GET_FLOAT(CALIBRATION, MAG_ROT_MAT_8);
	INS_U.Sensor_Param_j.mag_rotM[8] = PARAM_GET_FLOAT(CALIBRATION, MAG_ROT_MAT_9);

	INS_U.Sensor_Param_j.mag_bias[0] = PARAM_GET_FLOAT(CALIBRATION, MAG_BIAS_X);
	INS_U.Sensor_Param_j.mag_bias[1] = PARAM_GET_FLOAT(CALIBRATION, MAG_BIAS_Y);
	INS_U.Sensor_Param_j.mag_bias[2] = PARAM_GET_FLOAT(CALIBRATION, MAG_BIAS_Z);

	/* start logging */
	if(filemanager_status() == fm_init_ok && PARAM_GET_FLOAT(SYSTEM, LOG_AUTO_START)) {
		log_auto_start();
	}

	while(1) {
		res = rt_event_recv(&event_fastloop, wait_set, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
		                    RT_WAITING_FOREVER, &recv_set);

		if(res == RT_EOK) {
			if(recv_set & EVENT_FAST_LOOP) {
				fast_loop();
			}
		}
	}
}
