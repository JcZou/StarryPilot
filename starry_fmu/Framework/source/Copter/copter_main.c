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
 
#include <rtthread.h>
#include "copter_main.h"
#include "att_estimator.h"
#include "console.h"
#include "delay.h"
#include "sensor_manager.h"
#include "control_main.h"
#include "filter.h"
#include "pos_estimator.h"
#include "hil_interface.h"
#include "param.h"
#include "gps.h"
#include "state_est.h"
#include "OnBoard.h"
#include "INS.h"                 /* Model's header file */
#include "rtwtypes.h"

#define EVENT_COPTER_MAIN_LOOP		(1<<0)

static struct rt_timer timer_copter;
static struct rt_event event_copter;
uint32_t _att_est_period, _pos_est_period, _control_period;
uint32_t _ekf_est_period;

static char* TAG = "Copter_Main";

void copter_main_loop(uint32_t att_est_period, uint32_t pos_est_period, uint32_t control_period)
{
	static uint32_t ahrs_time = 0;
	static uint32_t pos_est_time = 0;
	static uint32_t ctrl_time = 0;
	static uint32_t state_est_time = 0;
	
	uint32_t now = time_nowMs();

#ifdef AHRS_USE_EKF	
	if(TIME_GAP(state_est_time, now) >= 4){
		state_est_time = now;
		state_est_update();
	}
#else	
	if(TIME_GAP(ahrs_time, now) >= att_est_period){
		ahrs_time = now;
		attitude_est_run(0.001f*att_est_period);
	}
	
	if(TIME_GAP(pos_est_time, now) >= pos_est_period){
		pos_est_time = now;
		pos_est_update(0.001f*pos_est_period);
	}
#endif
	
	if(TIME_GAP(ctrl_time, now) >= control_period){
		ctrl_time = now;
#ifdef	VEHICLE_BALANCE_CAR
		control_balance_car(0.001f*control_period);
#else
		control_vehicle(0.001f*control_period);
#endif
	}
}

static void timer_copter_update(void* parameter)
{
	rt_event_send(&event_copter, EVENT_COPTER_MAIN_LOOP);
}

uint32_t copter_get_event_period(Copter_Period event)
{
	uint32_t period = 0;

	switch(event)
	{
		case AHRS_Period:
		{
			period = _att_est_period;
		}break;
		case Control_Period:
		{
			period = _control_period;
		}break;
		case Pos_Period:
		{	
			period = _pos_est_period;
		}break;
		default:
			break;
	}
	
	return period;
}

void copter_entry(void *parameter)
{
	rt_err_t res;
	rt_uint32_t recv_set = 0;
	rt_uint32_t wait_set = EVENT_COPTER_MAIN_LOOP;
	
#ifdef HIL_SIMULATION
	_att_est_period = PARAM_GET_UINT32(HIL_SIM, HIL_ATT_EST_PRD);
	_pos_est_period = PARAM_GET_UINT32(HIL_SIM, HIL_POS_EST_PRD);
	_control_period = PARAM_GET_UINT32(HIL_SIM, HIL_CONTROL_PRD);
#else
	_att_est_period = AHRS_PERIOD;
	_pos_est_period = POS_EST_PERIOD;
	_control_period = CONTROL_PERIOD;
#endif	
	_ekf_est_period = EKF_PERIOD;
	
	filter_init();
	attitude_est_init();
	control_init();
	state_est_init(1e-3*_ekf_est_period);
#ifdef HIL_SIMULATION
	hil_interface_init(HIL_SENSOR_LEVEL);
	Console.print("HIL Simulation...\n");
#endif
	sensor_manager_init();
	pos_est_init(1e-3f*_pos_est_period);
	
	/* Initialize simulink model */
	INS_initialize();
#ifdef	VEHICLE_BALANCE_CAR
	OnBoard_initialize();
#endif
	
	/* create event */
	res = rt_event_init(&event_copter, "copter_event", RT_IPC_FLAG_FIFO);

	/* register timer event */
	rt_timer_init(&timer_copter, "timer_copter",
					timer_copter_update,
					RT_NULL,
					1,
					RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
	rt_timer_start(&timer_copter);

	while(1)
	{
		res = rt_event_recv(&event_copter, wait_set, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, 
								RT_WAITING_FOREVER, &recv_set);
		
		if(res == RT_EOK){
			if(recv_set & EVENT_COPTER_MAIN_LOOP){
				copter_main_loop(_att_est_period, _pos_est_period, _control_period);
			}
		}
	}
}
