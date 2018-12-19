/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: INS.c
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

#include "INS.h"
#include "INS_private.h"

rtTimingBridge INS_TimingBrdg;
const INS_Out_Bus INS_rtZINS_Out_Bus = {
	{
		0.0F, 0.0F, 0.0F, 0.0F
	}
	,                                    /* quat */

	{
		0.0F, 0.0F, 0.0F
	}
	,                                    /* euler */

	{
		0.0F, 0.0F, 0.0F
	}
	,                                    /* rot_radPs_B */

	{
		0.0F, 0.0F, 0.0F
	}
	,                                    /* acc_mPs2_O */

	{
		0, 0, 0
	}
	,                                    /* vel_cmPs_O */
	0,                                   /* lon_1e7_deg */
	0,                                   /* lat_1e7_deg */
	0,                                   /* altitude_cm */
	0U                                   /* timestamp_ms */
} ;                                    /* INS_Out_Bus ground */

/* Block signals (default storage) */
B_INS_T INS_B;

/* External inputs (root inport signals with default storage) */
ExtU_INS_T INS_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_INS_T INS_Y;

/* Real-time model */
RT_MODEL_INS_T INS_M_;
RT_MODEL_INS_T* const INS_M = &INS_M_;
static void rate_scheduler(void);

/*
 *   This function updates active task flag for each subrate.
 * The function is called at model base rate, hence the
 * generated code self-manages all its subrates.
 */
static void rate_scheduler(void)
{
	/* Compute which subrates run during the next base time step.  Subrates
	 * are an integer multiple of the base rate counter.  Therefore, the subtask
	 * counter is reset when it reaches its limit (zero means run).
	 */
	(INS_M->Timing.TaskCounters.TID[1])++;

	if((INS_M->Timing.TaskCounters.TID[1]) > 999) { /* Sample time: [2.0s, 0.0s] */
		INS_M->Timing.TaskCounters.TID[1] = 0;
	}
}

/* Model step function */
void INS_step(void)
{
	/* ModelReference: '<Root>/Sensor_Processing' incorporates:
	 *  Inport: '<Root>/Baro'
	 *  Inport: '<Root>/GPS_uBlox'
	 *  Inport: '<Root>/IMU1'
	 *  Inport: '<Root>/Mag'
	 *  Inport: '<Root>/Sensor_Param'
	 */
	SensorProcessing(&INS_U.IMU1.gyr_radPs_B[0], &INS_U.IMU1.acc_mPs2_B[0],
	                 &INS_U.IMU1.timestamp_ms, &INS_U.Mag.mag_ga_B[0],
	                 &INS_U.Mag.timestamp_ms, &INS_U.GPS_uBlox.fixType,
	                 &INS_U.GPS_uBlox.numSV, &INS_U.GPS_uBlox.lon,
	                 &INS_U.GPS_uBlox.lat, &INS_U.GPS_uBlox.height,
	                 &INS_U.GPS_uBlox.hAcc, &INS_U.GPS_uBlox.velN,
	                 &INS_U.GPS_uBlox.velE, &INS_U.GPS_uBlox.velD,
	                 &INS_U.GPS_uBlox.sAcc, &INS_U.GPS_uBlox.timestamp_ms,
	                 &INS_U.Baro.pressure_Pa, &INS_U.Baro.temperature_deg,
	                 &INS_U.Baro.timestamp_ms, &INS_U.Sensor_Param_j.gyr_rotM[0],
	                 &INS_U.Sensor_Param_j.gyr_bias[0],
	                 &INS_U.Sensor_Param_j.acc_rotM[0],
	                 &INS_U.Sensor_Param_j.acc_bias[0],
	                 &INS_U.Sensor_Param_j.mag_rotM[0],
	                 &INS_U.Sensor_Param_j.mag_bias[0], &INS_B.rot_radPs_B[0],
	                 &INS_B.sfor_mPs2_B[0], &INS_B.valid, &INS_B.mag_ga_B[0],
	                 &INS_B.valid_n, &INS_B.pos_quality, &INS_B.vel_quality,
	                 &INS_B.status, &INS_B.lon_rad, &INS_B.lat_rad,
	                 &INS_B.height_m, &INS_B.velN_mPs, &INS_B.velE_mPs,
	                 &INS_B.velD_mPs, &INS_B.numSV, &INS_B.valid_d,
	                 &INS_B.altitude_M, &INS_B.relative_alt_m);

	/* ModelReference: '<Root>/CF_INS' incorporates:
	 *  Outport: '<Root>/INS_Out'
	 */
	CF_INS(&INS_B.rot_radPs_B[0], &INS_B.sfor_mPs2_B[0], &INS_B.mag_ga_B[0],
	       &INS_B.pos_quality, &INS_B.vel_quality, &INS_B.height_m,
	       &INS_B.velD_mPs, &INS_B.valid_d, &INS_B.relative_alt_m, &INS_Y.INS_Out);
	rate_scheduler();
}

/* Model initialize function */
void INS_initialize(void)
{
	/* Registration code */

	/* initialize real-time model */
	(void) memset((void*)INS_M, 0,
	              sizeof(RT_MODEL_INS_T));

	/* block I/O */
	(void) memset(((void*) &INS_B), 0,
	              sizeof(B_INS_T));

	/* external inputs */
	(void)memset(&INS_U, 0, sizeof(ExtU_INS_T));

	/* external outputs */
	INS_Y.INS_Out = INS_rtZINS_Out_Bus;

	{
		static uint32_T* taskCounterPtrs;
		INS_TimingBrdg.nTasks = 2;
		INS_TimingBrdg.clockTick = (NULL);
		INS_TimingBrdg.clockTickH = (NULL);
		taskCounterPtrs = &(INS_M->Timing.TaskCounters.TID[0]);
		INS_TimingBrdg.taskCounter = taskCounterPtrs;
	}

	/* Model Initialize function for ModelReference Block: '<Root>/CF_INS' */
	CF_INS_initialize(rtmGetErrorStatusPointer(INS_M));

	/* Model Initialize function for ModelReference Block: '<Root>/Sensor_Processing' */
	SensorProcessing_initialize(rtmGetErrorStatusPointer(INS_M), &INS_TimingBrdg,
	                            0, 1);

	/* SystemInitialize for ModelReference: '<Root>/Sensor_Processing' incorporates:
	 *  Inport: '<Root>/Baro'
	 *  Inport: '<Root>/GPS_uBlox'
	 *  Inport: '<Root>/IMU1'
	 *  Inport: '<Root>/Mag'
	 *  Inport: '<Root>/Sensor_Param'
	 */
	SensorProcessing_Init();

	/* SystemInitialize for ModelReference: '<Root>/CF_INS' incorporates:
	 *  Outport: '<Root>/INS_Out'
	 */
	CF_INS_Init();
}

/* Model terminate function */
void INS_terminate(void)
{
	/* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
