/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: INS.h
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

#ifndef RTW_HEADER_INS_h_
#define RTW_HEADER_INS_h_
#include <string.h>
#include <stddef.h>
#ifndef INS_COMMON_INCLUDES_
	#define INS_COMMON_INCLUDES_
	#include "rtwtypes.h"
#endif                                 /* INS_COMMON_INCLUDES_ */

#include "INS_types.h"

/* Shared type includes */
#include "model_reference_types.h"

/* Child system includes */
#define SensorProcessing_MDLREF_HIDE_CHILD_
#include "SensorProcessing.h"
#define CF_INS_MDLREF_HIDE_CHILD_
#include "CF_INS.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
	#define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
	#define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetErrorStatusPointer
	#define rtmGetErrorStatusPointer(rtm) ((const char_T **)(&((rtm)->errorStatus)))
#endif

/* Block signals (default storage) */
typedef struct {
	real_T lon_rad;                      /* '<Root>/Sensor_Processing' */
	real_T lat_rad;                      /* '<Root>/Sensor_Processing' */
	real32_T rot_radPs_B[3];             /* '<Root>/Sensor_Processing' */
	real32_T sfor_mPs2_B[3];             /* '<Root>/Sensor_Processing' */
	real32_T mag_ga_B[3];                /* '<Root>/Sensor_Processing' */
	real32_T pos_quality;                /* '<Root>/Sensor_Processing' */
	real32_T vel_quality;                /* '<Root>/Sensor_Processing' */
	real32_T status;                     /* '<Root>/Sensor_Processing' */
	real32_T height_m;                   /* '<Root>/Sensor_Processing' */
	real32_T velN_mPs;                   /* '<Root>/Sensor_Processing' */
	real32_T velE_mPs;                   /* '<Root>/Sensor_Processing' */
	real32_T velD_mPs;                   /* '<Root>/Sensor_Processing' */
	real32_T altitude_M;                 /* '<Root>/Sensor_Processing' */
	real32_T relative_alt_m;             /* '<Root>/Sensor_Processing' */
	uint16_T numSV;                      /* '<Root>/Sensor_Processing' */
	boolean_T valid;                     /* '<Root>/Sensor_Processing' */
	boolean_T valid_n;                   /* '<Root>/Sensor_Processing' */
	boolean_T valid_d;                   /* '<Root>/Sensor_Processing' */
} B_INS_T;

/* External inputs (root inport signals with default storage) */
typedef struct {
	IMU1_Bus IMU1;                       /* '<Root>/IMU1' */
	Mag_Bus Mag;                         /* '<Root>/Mag' */
	GPS_uBlox_Bus GPS_uBlox;             /* '<Root>/GPS_uBlox' */
	Baro_Bus Baro;                       /* '<Root>/Baro' */
	Sensor_Param Sensor_Param_j;         /* '<Root>/Sensor_Param' */
} ExtU_INS_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
	INS_Out_Bus INS_Out;                 /* '<Root>/INS_Out' */
} ExtY_INS_T;

/* Real-time Model Data Structure */
struct tag_RTM_INS_T {
	const char_T* volatile errorStatus;

	/*
	 * Timing:
	 * The following substructure contains information regarding
	 * the timing information for the model.
	 */
	struct {
		struct {
			uint32_T TID[2];
		} TaskCounters;
	} Timing;
};

/* Block signals (default storage) */
extern B_INS_T INS_B;

/* External inputs (root inport signals with default storage) */
extern ExtU_INS_T INS_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_INS_T INS_Y;

/* External data declarations for dependent source files */
extern const INS_Out_Bus INS_rtZINS_Out_Bus;/* INS_Out_Bus ground */

/* Model entry point functions */
extern void INS_initialize(void);
extern void INS_step(void);
extern void INS_terminate(void);

/* Real-time Model object */
extern RT_MODEL_INS_T* const INS_M;

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'INS'
 */
#endif                                 /* RTW_HEADER_INS_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
