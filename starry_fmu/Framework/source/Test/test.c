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
//#include "att_estimator.h"
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
//#include "control_main.h"
#include "shell.h"
#include "systime.h"
#include "declination.h"
#include "light_matrix.h"
#include "sensor_manager.h"
#include "incapture.h"

MCN_DECLARE(SENSOR_BARO);
MCN_DECLARE(SENSOR_GYR);

#define Equal(x, y)					( fabs(x-y) < 1e-7 )

int calibrate_mag_run(struct finsh_shell* shell);

extern FIL my_fp;
extern int start;
int handle_test_shell_cmd(int argc, char** argv)
{

	return 0;
}
