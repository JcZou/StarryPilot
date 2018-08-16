/*****************************************************************************
Copyright (c) 2018, StarryPilot Development Team. All rights reserved.

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
#include "control_main.h"

#define EVENT_FAST_LOOP		(1<<0)

static struct rt_timer timer_fastloop;
static struct rt_event event_fastloop;

static void timer_fastloop_update(void* parameter)
{
	rt_event_send(&event_fastloop, EVENT_FAST_LOOP);
}

void fast_loop(void)
{
	
#ifdef HIL_SIMULATION
	hil_sensor_collect();
#else
	sensor_collect();
#endif
	
	ctrl_att_adrc_update();
	
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
