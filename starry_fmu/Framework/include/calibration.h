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

int calibrate_acc_run(struct finsh_shell *shell);
int calibrate_mag_run(struct finsh_shell *shell);
void calibrate_gyr_run(struct finsh_shell *shell);

#endif
