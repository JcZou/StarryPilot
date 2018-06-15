/*
 * File      : test.c
 *
 *
 * Change Logs:
 * Date           Author       	Notes
 * 2017-09-07     zoujiachi   	the first version
 */

#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "test.h"
#include "att_estimator.h"
#include "quaternion.h"
#include "console.h"
#include "global.h"
#include "led.h"
#include "sdio.h"
#include <stm32f4xx.h>
#include "ff.h"
#include "ap_math.h"
#include "param.h"
#include "uMCN.h"
#include "ms5611.h"
#include "control_main.h"
#include "shell.h"
#include "delay.h"

MCN_DECLARE(SENSOR_BARO);
MCN_DECLARE(SENSOR_GYR);

extern FIL my_fp;
extern int start;
int handle_test_shell_cmd(int argc, char** argv)
{
	if(argc > 1){
		float bth = atof(argv[1]);
		if(bth > 0){
			control_request(true);
			control_set("base_throttle", bth);
		}else{
			control_set("base_throttle", 0.0);
			control_request(false);
		}
	}else{
		control_set("base_throttle", 0.0);
		control_set("roll_sp", 0.0);
		control_set("pitch_sp", 0.0);
		Console.print("request control\n");
		control_request(true);
		Console.print("set base_throttle=0.5\n");
		control_set("base_throttle", 0.5);
		rt_thread_delay(3000);
		Console.print("set pitch=20deg\n");
		control_set("pitch_sp", 20.0);
		rt_thread_delay(3000);
		Console.print("set pitch=-20deg\n");
		control_set("pitch_sp", -20.0);
		rt_thread_delay(3000);
		Console.print("set pitch=0deg\n");
		control_set("pitch_sp", 0.0);
		rt_thread_delay(3000);
		control_set("base_throttle", 0.0);
		control_request(false);
		Console.print("finish\n");
	}
	
//	float gyr[3];
//	Euler e = {0,0,0};
//	float h = 0.002;
//	static uint32_t time = 0;
//	uint32_t last = 0;
//	while(!shell_is_end()){
//		uint32_t now = time_nowMs();
//		if(now - last >= 2){
//			last = now;
//			mcn_copy_from_hub(MCN_ID(SENSOR_GYR), gyr);
//			e.roll += gyr[0]*h;
//			e.pitch += gyr[1]*h;
//			e.yaw += gyr[2]*h;
//			Console.print_eachtime(&time, 200, "%.2f %.2f %.2f\n", Rad2Deg(e.roll),Rad2Deg(e.pitch),Rad2Deg(e.yaw));
//		}
//	}
	
//	if(argc > 1){
//		if(strcmp(argv[1], "start") == 0){
//			FRESULT fres = f_open(&my_fp, "log", FA_OPEN_ALWAYS | FA_WRITE);
//			start = 1;
//			Console.print("test start\n");
//		}
//		if(strcmp(argv[1], "stop") == 0){
//			start = 0;
//			f_close(&my_fp);
//			Console.print("test stop\n");
//		}
//	}
	
	return 0;
}
