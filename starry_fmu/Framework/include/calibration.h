/*
 * File      : calibration.h
 *
 *
 * Change Logs:
 * Date           Author       	Notes
 * 2016-6-20      zoujiachi   	the first version
 */
 
#ifndef __CALIBRATION_H__
#define __CALIBRATION_H__

//#include <rtthread.h>
//#include <rtdevice.h>
#include "shell.h"
#include "light_matrix.h"

#define CALI_THREAD_SLEEP_MS  5 /* cali thread run loop 5ms */
#define GYR_CALIBRATE_COUNT   500
#define ACC_POS_DETECT_COUNT  100
#define ACC_SAMPLE_COUNT      500

typedef struct
{
	double V[9];
	double D[9];
	double P[9][9];
	double R;
	
	double OFS[3];
	double GAIN[3];
	Mat	  EigVec;
	Mat	  RotM;
}Cali_Obj;
void gyr_mavlink_calibration(void);
void gyr_mavlink_calibration_start(void);
void acc_mavlink_calibration(void);
void acc_mavlink_calibration_start(void);
void mag_mavlink_calibration(void);
void mag_mavlink_calibration_start(void);

int calibrate_acc_run(void);
int calibrate_mag_run(void);
int calibrate_gyr_run(void);

void rt_cali_thread_entry(void* parameter);

#endif
