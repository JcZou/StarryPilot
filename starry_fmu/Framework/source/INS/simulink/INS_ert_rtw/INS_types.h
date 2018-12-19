/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: INS_types.h
 *
 * Code generated for Simulink model 'INS'.
 *
 * Model version                  : 1.9
 * Simulink Coder version         : 9.0 (R2018b) 24-May-2018
 * C/C++ source code generated on : Sun Dec 16 13:15:16 2018
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_INS_types_h_
#define RTW_HEADER_INS_types_h_
#include "rtwtypes.h"
#ifndef DEFINED_TYPEDEF_FOR_IMU1_Bus_
#define DEFINED_TYPEDEF_FOR_IMU1_Bus_

typedef struct {
	real32_T gyr_radPs_B[3];
	real32_T acc_mPs2_B[3];
	uint32_T timestamp_ms;
} IMU1_Bus;

#endif

#ifndef DEFINED_TYPEDEF_FOR_Mag_Bus_
#define DEFINED_TYPEDEF_FOR_Mag_Bus_

typedef struct {
	real32_T mag_ga_B[3];
	uint32_T timestamp_ms;
} Mag_Bus;

#endif

#ifndef DEFINED_TYPEDEF_FOR_GPS_uBlox_Bus_
#define DEFINED_TYPEDEF_FOR_GPS_uBlox_Bus_

/* NAV-PVT Message of u-Blox GPS */
typedef struct {
	uint32_T iTOW;
	uint16_T year;
	uint8_T month;
	uint8_T day;
	uint8_T hour;
	uint8_T min;
	uint8_T sec;
	uint8_T valid;
	uint32_T tAcc;
	int32_T nano;
	uint8_T fixType;
	uint8_T flags;
	uint8_T reserved1;
	uint8_T numSV;
	int32_T lon;
	int32_T lat;
	int32_T height;
	int32_T hMSL;
	uint32_T hAcc;
	uint32_T vAcc;
	int32_T velN;
	int32_T velE;
	int32_T velD;
	int32_T gSpeed;
	int32_T headMot;
	uint32_T sAcc;
	uint32_T headAcc;
	uint16_T pDOP;
	uint16_T reserved2;
	uint32_T timestamp_ms;
} GPS_uBlox_Bus;

#endif

#ifndef DEFINED_TYPEDEF_FOR_Baro_Bus_
#define DEFINED_TYPEDEF_FOR_Baro_Bus_

/* MS5611 (barometer) sensor data */
typedef struct {
	int32_T pressure_Pa;
	real32_T temperature_deg;
	uint32_T timestamp_ms;
} Baro_Bus;

#endif

#ifndef DEFINED_TYPEDEF_FOR_Sensor_Param_
#define DEFINED_TYPEDEF_FOR_Sensor_Param_

typedef struct {
	real32_T gyr_rotM[9];
	real32_T gyr_bias[3];
	real32_T acc_rotM[9];
	real32_T acc_bias[3];
	real32_T mag_rotM[9];
	real32_T mag_bias[3];
} Sensor_Param;

#endif

#ifndef DEFINED_TYPEDEF_FOR_IMU_Data_
#define DEFINED_TYPEDEF_FOR_IMU_Data_

typedef struct {
	real32_T rot_radPs_B[3];
	real32_T sfor_mPs2_B[3];
	boolean_T valid;
} IMU_Data;

#endif

#ifndef DEFINED_TYPEDEF_FOR_Mag_Data_
#define DEFINED_TYPEDEF_FOR_Mag_Data_

typedef struct {
	real32_T mag_ga_B[3];
	boolean_T valid;
} Mag_Data;

#endif

#ifndef DEFINED_TYPEDEF_FOR_GPS_Data_
#define DEFINED_TYPEDEF_FOR_GPS_Data_

typedef struct {
	real32_T pos_quality;
	real32_T vel_quality;
	real32_T status;
	real_T lon_rad;

	/* need double to ensure accurancy */
	real_T lat_rad;
	real32_T height_m;
	real32_T velN_mPs;
	real32_T velE_mPs;
	real32_T velD_mPs;
	uint16_T numSV;
} GPS_Data;

#endif

#ifndef DEFINED_TYPEDEF_FOR_Baro_Data_
#define DEFINED_TYPEDEF_FOR_Baro_Data_

typedef struct {
	boolean_T valid;
	real32_T altitude_M;
	real32_T relative_alt_m;
} Baro_Data;

#endif

#ifndef DEFINED_TYPEDEF_FOR_Sensor_Data_
#define DEFINED_TYPEDEF_FOR_Sensor_Data_

typedef struct {
	IMU_Data IMU_Data;
	Mag_Data Mag_Data;
	GPS_Data GPS_Data;
	Baro_Data Baro_Data;
} Sensor_Data;

#endif

#ifndef DEFINED_TYPEDEF_FOR_INS_Out_Bus_
#define DEFINED_TYPEDEF_FOR_INS_Out_Bus_

/* Experimental FCC actuator commands */
typedef struct {
	real32_T quat[4];
	real32_T euler[3];
	real32_T rot_radPs_B[3];
	real32_T acc_mPs2_O[3];
	int32_T vel_cmPs_O[3];
	int32_T lon_1e7_deg;
	int32_T lat_1e7_deg;
	int32_T altitude_cm;
	uint32_T timestamp_ms;
} INS_Out_Bus;

#endif

/* Forward declaration for rtModel */
typedef struct tag_RTM_INS_T RT_MODEL_INS_T;

#endif                                 /* RTW_HEADER_INS_types_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
