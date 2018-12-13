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
//#include "control_main.h"
#include "console.h"
#include "systime.h"
#include "uMcn.h"

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

MCN_DECLARE(uBLOX_PVT);

SensorIMU imu1;
SensorMag mag_field;
SensorBaro barometer;
SensorGPS_PVT gps_pvt;

static void timer_fastloop_update(void* parameter)
{
	rt_event_send(&event_fastloop, EVENT_FAST_LOOP);
}

void fast_loop(void)
{
	if(_mag_cnt >= MAG_READ_PERIOD){
		_mag_cnt = 0;

		if(sensor_mag_measure(mag_field.mag_ga) == RT_EOK){
			mag_field.timestamp_ms = time_nowMs();

			log_push_msg((uint8_t *)&mag_field, 0x02, sizeof(mag_field));
		}else{
			Console.print("fail to get mag data\n");
		}
	}

	if(_baro_cnt >= BARO_READ_PERIOD){
		_baro_cnt = 0;

		if(sensor_baro_update()){
			MS5611_REPORT_Def* baro_report = sensor_baro_get_report();

			barometer.pressure_Pa = baro_report->pressure;
			barometer.temperature_deg = baro_report->temperature;
			barometer.timestamp_ms = time_nowMs();
			
			log_push_msg((uint8_t *)&barometer, 0x03, sizeof(barometer));
		}
	}

	if(_gps_cnt >= GPS_READ_PERIOD){
		_gps_cnt = 0;

		// if(mcn_poll(gps_node_t)){

		// 	mcn_copy(MCN_ID(uBLOX_PVT), gps_node_t, &gps_pvt);	

		// 	log_push_msg((uint8_t *)&gps_pvt, 0x04, sizeof(gps_pvt));
		// }

		mcn_copy_from_hub(MCN_ID(uBLOX_PVT), &gps_pvt);	
		gps_pvt.timestamp_ms = time_nowMs();
		log_push_msg((uint8_t *)&gps_pvt, 0x04, sizeof(gps_pvt));
	}


	if(_ins_cnt >= INS_STEP_PERIOD){
		_ins_cnt = 0;

		if(sensor_gyr_measure(imu1.gyr_dps) == RT_EOK && sensor_acc_measure(imu1.acc_mps2) == RT_EOK){
			imu1.timestamp_ms = time_nowMs();

			log_push_msg((uint8_t *)&imu1, 0x01, sizeof(imu1));
		}else{
			Console.print("fail to get imu data\n");
		}
	
	}

	_ins_cnt++;
	_mag_cnt++;
	_baro_cnt++;
	_gps_cnt++;
}

void fastloop_entry(void *parameter)
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

	gps_node_t = mcn_subscribe(MCN_ID(uBLOX_PVT), NULL);
	if(gps_node_t == NULL)
		Console.print("gps_node_t subscribe err\n");
	
	while(1)
	{
		res = rt_event_recv(&event_fastloop, wait_set, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, 
								RT_WAITING_FOREVER, &recv_set);
		
		if(res == RT_EOK){
			if(recv_set & EVENT_FAST_LOOP){
				fast_loop();
			}
		}
	}
}
