/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: SensorProcessing.h
 *
 * Code generated for Simulink model 'SensorProcessing'.
 *
 * Model version                  : 1.175
 * Simulink Coder version         : 9.0 (R2018b) 24-May-2018
 * C/C++ source code generated on : Tue Dec 25 18:22:39 2018
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_SensorProcessing_h_
#define RTW_HEADER_SensorProcessing_h_
#include <math.h>
#include <string.h>
#ifndef SensorProcessing_COMMON_INCLUDES_
# define SensorProcessing_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* SensorProcessing_COMMON_INCLUDES_ */

#include "SensorProcessing_types.h"

/* Shared type includes */
#include "model_reference_types.h"
#include "rtGetInf.h"
#include "rt_nonfinite.h"

/* Block signals for model 'SensorProcessing' */
#ifndef SensorProcessing_MDLREF_HIDE_CHILD_

typedef struct {
  real32_T RoundingFunction;           /* '<S18>/Rounding Function' */
  real32_T DataTypeConversion;         /* '<S6>/Data Type Conversion' */
  real32_T Subtract1;                  /* '<S11>/Subtract1' */
  int32_T UnitDelay;                   /* '<S18>/Unit Delay' */
} B_SensorProcessing_c_T;

#endif                                 /*SensorProcessing_MDLREF_HIDE_CHILD_*/

/* Block states (default storage) for model 'SensorProcessing' */
#ifndef SensorProcessing_MDLREF_HIDE_CHILD_

typedef struct {
  int32_T UnitDelay_DSTATE;            /* '<S18>/Unit Delay' */
  uint32_T DelayInput1_DSTATE;         /* '<S25>/Delay Input1' */
  uint32_T DiscreteTimeIntegrator_DSTATE;/* '<S23>/Discrete-Time Integrator' */
  uint32_T DelayInput1_DSTATE_b;       /* '<S33>/Delay Input1' */
  uint32_T DiscreteTimeIntegrator_DSTATE_m;/* '<S31>/Discrete-Time Integrator' */
  uint32_T DelayInput1_DSTATE_g;       /* '<S16>/Delay Input1' */
  uint32_T DiscreteTimeIntegrator_DSTATE_e;/* '<S12>/Discrete-Time Integrator' */
  uint32_T DelayInput1_DSTATE_c;       /* '<S17>/Delay Input1' */
  uint32_T DiscreteTimeIntegrator1_DSTATE;/* '<S12>/Discrete-Time Integrator1' */
  uint32_T DelayInput1_DSTATE_c0;      /* '<S8>/Delay Input1' */
  uint32_T DiscreteTimeIntegrator_DSTATE_c;/* '<S5>/Discrete-Time Integrator' */
  real32_T ref_alt_PreviousInput;      /* '<S11>/ref_alt' */
  boolean_T Memory3_PreviousInput;     /* '<S11>/Memory3' */
} DW_SensorProcessing_f_T;

#endif                                 /*SensorProcessing_MDLREF_HIDE_CHILD_*/

/* Invariant block signals for model 'SensorProcessing' */
#ifndef SensorProcessing_MDLREF_HIDE_CHILD_

typedef struct {
  const real_T Multiply;               /* '<S6>/Multiply' */
  const real_T Gain1;                  /* '<S6>/Gain1' */
  const real_T Divide1;                /* '<S6>/Divide1' */
  const boolean_T LowerTest;           /* '<S27>/Lower Test' */
  const boolean_T UpperTest;           /* '<S27>/Upper Test' */
  const boolean_T AND;                 /* '<S27>/AND' */
  const boolean_T LowerTest_m;         /* '<S34>/Lower Test' */
  const boolean_T UpperTest_m;         /* '<S34>/Upper Test' */
  const boolean_T AND_m;               /* '<S34>/AND' */
} ConstB_SensorProcessing_h_T;

#endif                                 /*SensorProcessing_MDLREF_HIDE_CHILD_*/

#ifndef SensorProcessing_MDLREF_HIDE_CHILD_

/* Real-time Model Data Structure */
struct tag_RTM_SensorProcessing_T {
  const char_T **errorStatus;
};

#endif                                 /*SensorProcessing_MDLREF_HIDE_CHILD_*/

#ifndef SensorProcessing_MDLREF_HIDE_CHILD_

typedef struct {
  RT_MODEL_SensorProcessing_T rtm;
} MdlrefDW_SensorProcessing_T;

#endif                                 /*SensorProcessing_MDLREF_HIDE_CHILD_*/

extern void SensorProcessing_Init(void);
extern void SensorProcessing(const real32_T rtu_IMU1_gyr_radPs_B[3], const
  real32_T rtu_IMU1_acc_mPs2_B[3], const uint32_T *rtu_IMU1_timestamp_ms, const
  real32_T rtu_Mag_mag_ga_B[3], const uint32_T *rtu_Mag_timestamp_ms, const
  uint8_T *rtu_GPS_uBlox_fixType, const uint8_T *rtu_GPS_uBlox_numSV, const
  int32_T *rtu_GPS_uBlox_lon, const int32_T *rtu_GPS_uBlox_lat, const int32_T
  *rtu_GPS_uBlox_height, const uint32_T *rtu_GPS_uBlox_hAcc, const int32_T
  *rtu_GPS_uBlox_velN, const int32_T *rtu_GPS_uBlox_velE, const int32_T
  *rtu_GPS_uBlox_velD, const uint32_T *rtu_GPS_uBlox_sAcc, const uint32_T
  *rtu_GPS_uBlox_timestamp_ms, const int32_T *rtu_Baro_pressure_Pa, const
  real32_T *rtu_Baro_temperature_deg, const uint32_T *rtu_Baro_timestamp_ms,
  const real32_T rtu_Sensor_Param_gyr_rotM[9], const real32_T
  rtu_Sensor_Param_gyr_bias[3], const real32_T rtu_Sensor_Param_acc_rotM[9],
  const real32_T rtu_Sensor_Param_acc_bias[3], const real32_T
  rtu_Sensor_Param_mag_rotM[9], const real32_T rtu_Sensor_Param_mag_bias[3],
  real32_T rty_Sensor_Data_IMU_Data_rot_ra[3], real32_T
  rty_Sensor_Data_IMU_Data_sfor_m[3], boolean_T *rty_Sensor_Data_IMU_Data_valid,
  real32_T rty_Sensor_Data_Mag_Data_mag_ga[3], boolean_T
  *rty_Sensor_Data_Mag_Data_valid, real32_T *rty_Sensor_Data_GPS_Data_pos_qu,
  real32_T *rty_Sensor_Data_GPS_Data_vel_qu, real32_T
  *rty_Sensor_Data_GPS_Data_status, real_T *rty_Sensor_Data_GPS_Data_lon_ra,
  real_T *rty_Sensor_Data_GPS_Data_lat_ra, real32_T
  *rty_Sensor_Data_GPS_Data_height, real32_T *rty_Sensor_Data_GPS_Data_velN_m,
  real32_T *rty_Sensor_Data_GPS_Data_velE_m, real32_T
  *rty_Sensor_Data_GPS_Data_velD_m, uint16_T *rty_Sensor_Data_GPS_Data_numSV,
  boolean_T *rty_Sensor_Data_Baro_Data_valid, real32_T
  *rty_Sensor_Data_Baro_Data_altit, real32_T *rty_Sensor_Data_Baro_Data_relat);

/* Model reference registration function */
extern void SensorProcessing_initialize(const char_T **rt_errorStatus, const
  rtTimingBridge *timingBridge, int_T mdlref_TID0, int_T mdlref_TID1);

#ifndef SensorProcessing_MDLREF_HIDE_CHILD_

extern MdlrefDW_SensorProcessing_T SensorProcessing_MdlrefDW;

#endif                                 /*SensorProcessing_MDLREF_HIDE_CHILD_*/

#ifndef SensorProcessing_MDLREF_HIDE_CHILD_

/* Block signals (default storage) */
extern B_SensorProcessing_c_T SensorProcessing_B;

/* Block states (default storage) */
extern DW_SensorProcessing_f_T SensorProcessing_DW;

#endif                                 /*SensorProcessing_MDLREF_HIDE_CHILD_*/

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S9>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S10>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S26>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S27>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S28>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S34>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S35>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S5>/Logical Operator1' : Eliminated due to no operation
 * Block '<S3>/Reshape' : Reshape block reduction
 * Block '<S3>/Reshape1' : Reshape block reduction
 */

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
 * '<Root>' : 'SensorProcessing'
 * '<S1>'   : 'SensorProcessing/Baro_Preprocessing'
 * '<S2>'   : 'SensorProcessing/GPS_Preprocessing'
 * '<S3>'   : 'SensorProcessing/IMU_Preprocessing'
 * '<S4>'   : 'SensorProcessing/Mag_Preprocessing'
 * '<S5>'   : 'SensorProcessing/Baro_Preprocessing/Baro_detection'
 * '<S6>'   : 'SensorProcessing/Baro_Preprocessing/Pressure_Altitude'
 * '<S7>'   : 'SensorProcessing/Baro_Preprocessing/Baro_detection/Compare To Constant4'
 * '<S8>'   : 'SensorProcessing/Baro_Preprocessing/Baro_detection/Detect Change'
 * '<S9>'   : 'SensorProcessing/Baro_Preprocessing/Baro_detection/Interval Test2'
 * '<S10>'  : 'SensorProcessing/Baro_Preprocessing/Baro_detection/Interval Test3'
 * '<S11>'  : 'SensorProcessing/Baro_Preprocessing/Pressure_Altitude/Relative_Altitude'
 * '<S12>'  : 'SensorProcessing/GPS_Preprocessing/GPS_Status_Check'
 * '<S13>'  : 'SensorProcessing/GPS_Preprocessing/GPS_Status_Check/Compare To Constant1'
 * '<S14>'  : 'SensorProcessing/GPS_Preprocessing/GPS_Status_Check/Compare To Constant2'
 * '<S15>'  : 'SensorProcessing/GPS_Preprocessing/GPS_Status_Check/Compare To Constant4'
 * '<S16>'  : 'SensorProcessing/GPS_Preprocessing/GPS_Status_Check/Detect Change'
 * '<S17>'  : 'SensorProcessing/GPS_Preprocessing/GPS_Status_Check/Detect Change1'
 * '<S18>'  : 'SensorProcessing/GPS_Preprocessing/GPS_Status_Check/Gps_Status'
 * '<S19>'  : 'SensorProcessing/GPS_Preprocessing/GPS_Status_Check/Gps_Status/Compare To Constant'
 * '<S20>'  : 'SensorProcessing/GPS_Preprocessing/GPS_Status_Check/Gps_Status/Compare To Constant2'
 * '<S21>'  : 'SensorProcessing/GPS_Preprocessing/GPS_Status_Check/Gps_Status/Compare To Constant3'
 * '<S22>'  : 'SensorProcessing/IMU_Preprocessing/Calibration'
 * '<S23>'  : 'SensorProcessing/IMU_Preprocessing/IMU_detection'
 * '<S24>'  : 'SensorProcessing/IMU_Preprocessing/IMU_detection/Compare To Constant4'
 * '<S25>'  : 'SensorProcessing/IMU_Preprocessing/IMU_detection/Detect Change'
 * '<S26>'  : 'SensorProcessing/IMU_Preprocessing/IMU_detection/Interval Test1'
 * '<S27>'  : 'SensorProcessing/IMU_Preprocessing/IMU_detection/Interval Test2'
 * '<S28>'  : 'SensorProcessing/IMU_Preprocessing/IMU_detection/Interval Test3'
 * '<S29>'  : 'SensorProcessing/Mag_Preprocessing/Calibration'
 * '<S30>'  : 'SensorProcessing/Mag_Preprocessing/Mag_Declination_Compensate'
 * '<S31>'  : 'SensorProcessing/Mag_Preprocessing/Mag_detection'
 * '<S32>'  : 'SensorProcessing/Mag_Preprocessing/Mag_detection/Compare To Constant4'
 * '<S33>'  : 'SensorProcessing/Mag_Preprocessing/Mag_detection/Detect Change'
 * '<S34>'  : 'SensorProcessing/Mag_Preprocessing/Mag_detection/Interval Test2'
 * '<S35>'  : 'SensorProcessing/Mag_Preprocessing/Mag_detection/Interval Test3'
 */
#endif                                 /* RTW_HEADER_SensorProcessing_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
