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
#include "declination.h"
#include "light_matrix.h"
#include "sensor_manager.h"

MCN_DECLARE(SENSOR_BARO);
MCN_DECLARE(SENSOR_GYR);

#define Equal(x, y)					( fabs(x-y) < 1e-7 )

int calibrate_mag_run(struct finsh_shell *shell);

extern FIL my_fp;
extern int start;
int handle_test_shell_cmd(int argc, char** argv)
{
//	Console.print("am mag dec:%f %f\n", compass_get_declination(52.3702, 4.8952), compass_get_declination(-52.3702, -4.8952));
//	Console.print("sh mag dec:%f\n", compass_get_declination(31.2304, 121.4737));
//	Console.print("roma mag dec:%f\n", compass_get_declination(41.9028, 12.4964));
//	if(argc > 1){
//		float bth = atof(argv[1]);
//		if(bth > 0){
//			control_request(true);
//			control_set("base_throttle", bth);
//		}else{
//			control_set("base_throttle", 0.0);
//			control_request(false);
//		}
//	}else{
//		control_set("base_throttle", 0.0);
//		control_set("roll_sp", 0.0);
//		control_set("pitch_sp", 0.0);
//		Console.print("request control\n");
//		control_request(true);
//		Console.print("set base_throttle=0.5\n");
//		control_set("base_throttle", 0.5);
//		rt_thread_delay(3000);
//		Console.print("set pitch=20deg\n");
//		control_set("pitch_sp", 20.0);
//		rt_thread_delay(3000);
//		Console.print("set pitch=-20deg\n");
//		control_set("pitch_sp", -20.0);
//		rt_thread_delay(3000);
//		Console.print("set pitch=0deg\n");
//		control_set("pitch_sp", 0.0);
//		rt_thread_delay(3000);
//		control_set("base_throttle", 0.0);
//		control_request(false);
//		Console.print("finish\n");
//	}
	
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

	//calibrate_mag_run(NULL);
	
//	Mat A;
//	MatCreate(&A, 5, 5);
//	LIGHT_MATRIX_TYPE val[25] = {
//		10, 1, 2, 3, 4,
//		1, 9, -1, 2, -3,
//		2, -1, 7, 3, -5,
//		3, 2, 3, 12, -1,
//		4, -3, -5, -1, 15
//	};
//	MatSetVal(&A, val);
//	MatDump(&A);
//	
//	LIGHT_MATRIX_TYPE eig_val[5];
//	Mat eig_vec;
//	MatCreate(&eig_vec, 5, 5);
//	MatEig(&A, eig_val, &eig_vec, 1e-6, 100);
//	
//	Console.print("eig val: %lf %lf\n %lf %lf %lf\n\n", eig_val[0],eig_val[1],eig_val[2],eig_val[3],eig_val[4]);
//	MatDump(&eig_vec);

	float acc[3], gyr[3], mag[3];

	sensor_get_acc(acc);
	sensor_get_mag(mag);
	sensor_get_gyr(gyr);
	
	Console.print("acc:%f %f %f\n", acc[0], acc[1], acc[2]);
	Console.print("mag:%f %f %f\n", mag[0], mag[1], mag[2]);
	if(acc[0] == 0.0f && acc[1]==0.0f  && acc[2] == 0.0f){
		Console.print("acc null\n");
	}
	if(mag[0] == 0.0f && mag[1] == 0.0f && mag[2] == 0.0f){
		Console.print("mag null\n");
	}
	
	return 0;
}
