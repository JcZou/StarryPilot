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

#include <rtthread.h>
#include <rtdevice.h>
#include "shell.h"

#define ACC_STANDARD_VALUE		9.8f
#define MAG_STANDARD_VALUE		1.0f

#define CALI_METHOD_1
//#define CALI_METHOD_2

//#ifdef CALI_METHOD_1
//void ResetMatrix(void);
//void CalcData_Input(double x , double y , double z);
//double* calibrate_process(double radius);
//void cali_input_acc_data(uint16_t p_num);
//void cali_input_mag_data(uint16_t p_num);
//rt_err_t calibrate_gyr(uint16_t p_num);
//#elif defined CALI_METHOD_2
//void Calc_Process(void);
//void Reset_Cali(void);
//void cali_input_mag_data(uint16_t p_num);
//#endif

int calibrate_acc_run(struct finsh_shell *shell);
int calibrate_mag_run(struct finsh_shell *shell);
void calibrate_gyr_run(struct finsh_shell *shell);

#endif
