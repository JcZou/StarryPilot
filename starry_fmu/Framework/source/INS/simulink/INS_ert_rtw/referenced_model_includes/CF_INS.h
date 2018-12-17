/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: CF_INS.h
 *
 * Code generated for Simulink model 'CF_INS'.
 *
 * Model version                  : 1.338
 * Simulink Coder version         : 9.0 (R2018b) 24-May-2018
 * C/C++ source code generated on : Sun Dec 16 13:14:49 2018
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_CF_INS_h_
#define RTW_HEADER_CF_INS_h_
#include <math.h>
#include <string.h>
#ifndef CF_INS_COMMON_INCLUDES_
# define CF_INS_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* CF_INS_COMMON_INCLUDES_ */

#include "CF_INS_types.h"
#include "rt_nonfinite.h"
#include "rtGetInf.h"

/* Block states (default storage) for model 'CF_INS' */
#ifndef CF_INS_MDLREF_HIDE_CHILD_

typedef struct {
  real32_T DiscreteTimeIntegrator_DSTATE[4];/* '<S5>/Discrete-Time Integrator' */
  real32_T DiscreteTimeIntegrator1_DSTATE[3];/* '<S37>/Discrete-Time Integrator1' */
  real32_T DiscreteTimeIntegrator_DSTATE_k;/* '<S61>/Discrete-Time Integrator' */
  real32_T DiscreteTimeIntegrator1_DSTAT_h;/* '<S66>/Discrete-Time Integrator1' */
  real32_T DiscreteTimeIntegrator_DSTATE_i;/* '<S59>/Discrete-Time Integrator' */
  real32_T DiscreteTimeIntegrator_DSTAT_ku;/* '<S62>/Discrete-Time Integrator' */
  real32_T DiscreteTimeIntegrator1_DSTAT_c;/* '<S62>/Discrete-Time Integrator1' */
  int8_T DiscreteTimeIntegrator_PrevRese;/* '<S5>/Discrete-Time Integrator' */
  int8_T DiscreteTimeIntegrator1_PrevRes;/* '<S37>/Discrete-Time Integrator1' */
  uint8_T DiscreteTimeIntegrator_IC_LOADI;/* '<S5>/Discrete-Time Integrator' */
  uint8_T DiscreteTimeIntegrator_IC_LOA_c;/* '<S59>/Discrete-Time Integrator' */
  uint8_T DiscreteTimeIntegrator1_IC_LOAD;/* '<S62>/Discrete-Time Integrator1' */
} DW_CF_INS_f_T;

#endif                                 /*CF_INS_MDLREF_HIDE_CHILD_*/

/* Invariant block signals for model 'CF_INS' */
#ifndef CF_INS_MDLREF_HIDE_CHILD_

typedef struct {
  const real32_T Constant7[3];         /* '<S37>/Constant7' */
} ConstB_CF_INS_h_T;

#endif                                 /*CF_INS_MDLREF_HIDE_CHILD_*/

#ifndef CF_INS_MDLREF_HIDE_CHILD_

/* Real-time Model Data Structure */
struct tag_RTM_CF_INS_T {
  const char_T **errorStatus;
};

#endif                                 /*CF_INS_MDLREF_HIDE_CHILD_*/

#ifndef CF_INS_MDLREF_HIDE_CHILD_

typedef struct {
  RT_MODEL_CF_INS_T rtm;
} MdlrefDW_CF_INS_T;

#endif                                 /*CF_INS_MDLREF_HIDE_CHILD_*/

extern void CF_INS_Init(void);
extern void CF_INS(const real32_T rtu_Sensor_Data_IMU_Data_rot_ra[3], const
                   real32_T rtu_Sensor_Data_IMU_Data_sfor_m[3], const real32_T
                   rtu_Sensor_Data_Mag_Data_mag_ga[3], const real32_T
                   *rtu_Sensor_Data_GPS_Data_pos_qu, const real32_T
                   *rtu_Sensor_Data_GPS_Data_vel_qu, const real32_T
                   *rtu_Sensor_Data_GPS_Data_height, const real32_T
                   *rtu_Sensor_Data_GPS_Data_velD_m, const boolean_T
                   *rtu_Sensor_Data_Baro_Data_valid, const real32_T
                   *rtu_Sensor_Data_Baro_Data_relat, INS_Out_Bus *rty_INS_Out);

/* Model reference registration function */
extern void CF_INS_initialize(const char_T **rt_errorStatus);

#ifndef CF_INS_MDLREF_HIDE_CHILD_

extern MdlrefDW_CF_INS_T CF_INS_MdlrefDW;

#endif                                 /*CF_INS_MDLREF_HIDE_CHILD_*/

#ifndef CF_INS_MDLREF_HIDE_CHILD_

/* Block states (default storage) */
extern DW_CF_INS_f_T CF_INS_DW;

#endif                                 /*CF_INS_MDLREF_HIDE_CHILD_*/

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S23>/Constant' : Unused code path elimination
 * Block '<S23>/Gain1' : Unused code path elimination
 * Block '<S23>/Gain2' : Unused code path elimination
 * Block '<S23>/Gain3' : Unused code path elimination
 * Block '<S23>/Product1' : Unused code path elimination
 * Block '<S23>/Product2' : Unused code path elimination
 * Block '<S23>/Product3' : Unused code path elimination
 * Block '<S23>/Product4' : Unused code path elimination
 * Block '<S23>/Product5' : Unused code path elimination
 * Block '<S23>/Product6' : Unused code path elimination
 * Block '<S23>/Product7' : Unused code path elimination
 * Block '<S23>/Product8' : Unused code path elimination
 * Block '<S23>/Product9' : Unused code path elimination
 * Block '<S23>/Sum' : Unused code path elimination
 * Block '<S23>/Sum1' : Unused code path elimination
 * Block '<S23>/Sum2' : Unused code path elimination
 * Block '<S23>/Sum3' : Unused code path elimination
 * Block '<S47>/Data Type Duplicate' : Unused code path elimination
 * Block '<S48>/Data Type Duplicate' : Unused code path elimination
 * Block '<S58>/Scope' : Unused code path elimination
 * Block '<S59>/Scope' : Unused code path elimination
 * Block '<S59>/Scope1' : Unused code path elimination
 * Block '<S62>/x_v_a_scope' : Unused code path elimination
 * Block '<S60>/Scope' : Unused code path elimination
 * Block '<S63>/Constant' : Unused code path elimination
 * Block '<S63>/Constant1' : Unused code path elimination
 * Block '<S64>/Delay Input1' : Unused code path elimination
 * Block '<S64>/FixPt Relational Operator' : Unused code path elimination
 * Block '<S63>/Discrete-Time Integrator' : Unused code path elimination
 * Block '<S63>/Discrete-Time Integrator1' : Unused code path elimination
 * Block '<S63>/Logical Operator' : Unused code path elimination
 * Block '<S63>/Memory' : Unused code path elimination
 * Block '<S63>/Subtract' : Unused code path elimination
 * Block '<S63>/Switch' : Unused code path elimination
 * Block '<S65>/Abs' : Unused code path elimination
 * Block '<S65>/Add' : Unused code path elimination
 * Block '<S65>/Add1' : Unused code path elimination
 * Block '<S65>/Add2' : Unused code path elimination
 * Block '<S65>/Add3' : Unused code path elimination
 * Block '<S65>/Add4' : Unused code path elimination
 * Block '<S65>/Add5' : Unused code path elimination
 * Block '<S65>/Add6' : Unused code path elimination
 * Block '<S65>/Divide' : Unused code path elimination
 * Block '<S65>/Gain' : Unused code path elimination
 * Block '<S65>/Gain1' : Unused code path elimination
 * Block '<S65>/Gain2' : Unused code path elimination
 * Block '<S65>/Gain3' : Unused code path elimination
 * Block '<S65>/Gain4' : Unused code path elimination
 * Block '<S65>/Multiply' : Unused code path elimination
 * Block '<S65>/Multiply1' : Unused code path elimination
 * Block '<S65>/Multiply2' : Unused code path elimination
 * Block '<S65>/Multiply3' : Unused code path elimination
 * Block '<S65>/Multiply4' : Unused code path elimination
 * Block '<S65>/Multiply5' : Unused code path elimination
 * Block '<S65>/Multiply6' : Unused code path elimination
 * Block '<S65>/Sign' : Unused code path elimination
 * Block '<S65>/Sign1' : Unused code path elimination
 * Block '<S65>/Sign2' : Unused code path elimination
 * Block '<S65>/Sign3' : Unused code path elimination
 * Block '<S65>/Sign4' : Unused code path elimination
 * Block '<S65>/Sign5' : Unused code path elimination
 * Block '<S65>/Sign6' : Unused code path elimination
 * Block '<S65>/Sqrt' : Unused code path elimination
 * Block '<S65>/Square' : Unused code path elimination
 * Block '<S65>/Subtract' : Unused code path elimination
 * Block '<S65>/Subtract1' : Unused code path elimination
 * Block '<S65>/Subtract2' : Unused code path elimination
 * Block '<S65>/Subtract3' : Unused code path elimination
 * Block '<S65>/Subtract4' : Unused code path elimination
 * Block '<S65>/Subtract5' : Unused code path elimination
 * Block '<S65>/Subtract6' : Unused code path elimination
 * Block '<S65>/const' : Unused code path elimination
 * Block '<S65>/const1' : Unused code path elimination
 * Block '<S66>/Scope' : Unused code path elimination
 * Block '<S68>/Constant' : Unused code path elimination
 * Block '<S68>/Gain' : Unused code path elimination
 * Block '<S68>/Gain1' : Unused code path elimination
 * Block '<S68>/Gain2' : Unused code path elimination
 * Block '<S68>/Product' : Unused code path elimination
 * Block '<S68>/Product1' : Unused code path elimination
 * Block '<S68>/Product2' : Unused code path elimination
 * Block '<S68>/Product3' : Unused code path elimination
 * Block '<S68>/Product4' : Unused code path elimination
 * Block '<S68>/Product5' : Unused code path elimination
 * Block '<S68>/Product6' : Unused code path elimination
 * Block '<S68>/Product7' : Unused code path elimination
 * Block '<S68>/Product8' : Unused code path elimination
 * Block '<S68>/Sum' : Unused code path elimination
 * Block '<S68>/Sum1' : Unused code path elimination
 * Block '<S68>/Sum2' : Unused code path elimination
 * Block '<S68>/Sum3' : Unused code path elimination
 * Block '<S69>/Constant' : Unused code path elimination
 * Block '<S69>/Gain' : Unused code path elimination
 * Block '<S69>/Gain1' : Unused code path elimination
 * Block '<S69>/Gain2' : Unused code path elimination
 * Block '<S69>/Product' : Unused code path elimination
 * Block '<S69>/Product1' : Unused code path elimination
 * Block '<S69>/Product2' : Unused code path elimination
 * Block '<S69>/Product3' : Unused code path elimination
 * Block '<S69>/Product4' : Unused code path elimination
 * Block '<S69>/Product5' : Unused code path elimination
 * Block '<S69>/Product6' : Unused code path elimination
 * Block '<S69>/Product7' : Unused code path elimination
 * Block '<S69>/Product8' : Unused code path elimination
 * Block '<S69>/Sum' : Unused code path elimination
 * Block '<S69>/Sum1' : Unused code path elimination
 * Block '<S69>/Sum2' : Unused code path elimination
 * Block '<S69>/Sum3' : Unused code path elimination
 * Block '<S47>/Column Vector' : Reshape block reduction
 * Block '<S47>/Column Vector1' : Reshape block reduction
 * Block '<S47>/Column Vector2' : Reshape block reduction
 * Block '<S48>/Column Vector1' : Reshape block reduction
 * Block '<S48>/Column Vector2' : Reshape block reduction
 * Block '<S59>/kp' : Eliminated nontunable gain of 1
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
 * '<Root>' : 'CF_INS'
 * '<S1>'   : 'CF_INS/AHRS'
 * '<S2>'   : 'CF_INS/INS_Out_Pack'
 * '<S3>'   : 'CF_INS/Position_Estimator'
 * '<S4>'   : 'CF_INS/AHRS/Attitude Correct'
 * '<S5>'   : 'CF_INS/AHRS/Attitude Predict'
 * '<S6>'   : 'CF_INS/AHRS/Attitude_Init'
 * '<S7>'   : 'CF_INS/AHRS/Attitude Correct/body to earth'
 * '<S8>'   : 'CF_INS/AHRS/Attitude Correct/body to earth1'
 * '<S9>'   : 'CF_INS/AHRS/Attitude Correct/cross_product'
 * '<S10>'  : 'CF_INS/AHRS/Attitude Correct/cross_product1'
 * '<S11>'  : 'CF_INS/AHRS/Attitude Correct/earth to body'
 * '<S12>'  : 'CF_INS/AHRS/Attitude Correct/vec_normalize'
 * '<S13>'  : 'CF_INS/AHRS/Attitude Correct/vec_normalize1'
 * '<S14>'  : 'CF_INS/AHRS/Attitude Correct/vec_normalize2'
 * '<S15>'  : 'CF_INS/AHRS/Attitude Correct/body to earth/Subsystem'
 * '<S16>'  : 'CF_INS/AHRS/Attitude Correct/body to earth/Subsystem1'
 * '<S17>'  : 'CF_INS/AHRS/Attitude Correct/body to earth/Subsystem2'
 * '<S18>'  : 'CF_INS/AHRS/Attitude Correct/body to earth/quat normalize'
 * '<S19>'  : 'CF_INS/AHRS/Attitude Correct/body to earth/quat normalize/quat modulus'
 * '<S20>'  : 'CF_INS/AHRS/Attitude Correct/body to earth/quat normalize/quat modulus/quat norm'
 * '<S21>'  : 'CF_INS/AHRS/Attitude Correct/body to earth1/Subsystem'
 * '<S22>'  : 'CF_INS/AHRS/Attitude Correct/body to earth1/Subsystem1'
 * '<S23>'  : 'CF_INS/AHRS/Attitude Correct/body to earth1/Subsystem2'
 * '<S24>'  : 'CF_INS/AHRS/Attitude Correct/body to earth1/quat normalize'
 * '<S25>'  : 'CF_INS/AHRS/Attitude Correct/body to earth1/quat normalize/quat modulus'
 * '<S26>'  : 'CF_INS/AHRS/Attitude Correct/body to earth1/quat normalize/quat modulus/quat norm'
 * '<S27>'  : 'CF_INS/AHRS/Attitude Correct/cross_product/Subsystem'
 * '<S28>'  : 'CF_INS/AHRS/Attitude Correct/cross_product/Subsystem1'
 * '<S29>'  : 'CF_INS/AHRS/Attitude Correct/cross_product1/Subsystem'
 * '<S30>'  : 'CF_INS/AHRS/Attitude Correct/cross_product1/Subsystem1'
 * '<S31>'  : 'CF_INS/AHRS/Attitude Correct/earth to body/Subsystem'
 * '<S32>'  : 'CF_INS/AHRS/Attitude Correct/earth to body/Subsystem1'
 * '<S33>'  : 'CF_INS/AHRS/Attitude Correct/earth to body/Subsystem2'
 * '<S34>'  : 'CF_INS/AHRS/Attitude Correct/earth to body/quat normalize'
 * '<S35>'  : 'CF_INS/AHRS/Attitude Correct/earth to body/quat normalize/quat modulus'
 * '<S36>'  : 'CF_INS/AHRS/Attitude Correct/earth to body/quat normalize/quat modulus/quat norm'
 * '<S37>'  : 'CF_INS/AHRS/Attitude Predict/Rot_Bias_Compensate'
 * '<S38>'  : 'CF_INS/AHRS/Attitude Predict/quat mult'
 * '<S39>'  : 'CF_INS/AHRS/Attitude Predict/quat normalize'
 * '<S40>'  : 'CF_INS/AHRS/Attitude Predict/quat to euler'
 * '<S41>'  : 'CF_INS/AHRS/Attitude Predict/quat mult/q(0)'
 * '<S42>'  : 'CF_INS/AHRS/Attitude Predict/quat mult/q(1)'
 * '<S43>'  : 'CF_INS/AHRS/Attitude Predict/quat mult/q(2)'
 * '<S44>'  : 'CF_INS/AHRS/Attitude Predict/quat mult/q(3)'
 * '<S45>'  : 'CF_INS/AHRS/Attitude Predict/quat normalize/quat modulus'
 * '<S46>'  : 'CF_INS/AHRS/Attitude Predict/quat normalize/quat modulus/quat norm'
 * '<S47>'  : 'CF_INS/AHRS/Attitude_Init/Cross_Product'
 * '<S48>'  : 'CF_INS/AHRS/Attitude_Init/Cross_Product1'
 * '<S49>'  : 'CF_INS/AHRS/Attitude_Init/Euclidean_Norm1'
 * '<S50>'  : 'CF_INS/AHRS/Attitude_Init/Euclidean_Norm2'
 * '<S51>'  : 'CF_INS/AHRS/Attitude_Init/M to Quat'
 * '<S52>'  : 'CF_INS/AHRS/Attitude_Init/M to Quat/If Action Subsystem'
 * '<S53>'  : 'CF_INS/AHRS/Attitude_Init/M to Quat/If Action Subsystem1'
 * '<S54>'  : 'CF_INS/AHRS/Attitude_Init/M to Quat/If Action Subsystem2'
 * '<S55>'  : 'CF_INS/AHRS/Attitude_Init/M to Quat/If Action Subsystem3'
 * '<S56>'  : 'CF_INS/Position_Estimator/Horizontal_Filter'
 * '<S57>'  : 'CF_INS/Position_Estimator/Vertical_Filter'
 * '<S58>'  : 'CF_INS/Position_Estimator/Vertical_Filter/Alt Correct'
 * '<S59>'  : 'CF_INS/Position_Estimator/Vertical_Filter/Alt Predict'
 * '<S60>'  : 'CF_INS/Position_Estimator/Vertical_Filter/Vel_z Correct'
 * '<S61>'  : 'CF_INS/Position_Estimator/Vertical_Filter/Vel_z Predict'
 * '<S62>'  : 'CF_INS/Position_Estimator/Vertical_Filter/Vel_z Correct/Linear Observer'
 * '<S63>'  : 'CF_INS/Position_Estimator/Vertical_Filter/Vel_z Correct/TD'
 * '<S64>'  : 'CF_INS/Position_Estimator/Vertical_Filter/Vel_z Correct/TD/Detect Change'
 * '<S65>'  : 'CF_INS/Position_Estimator/Vertical_Filter/Vel_z Correct/TD/fhan'
 * '<S66>'  : 'CF_INS/Position_Estimator/Vertical_Filter/Vel_z Predict/acc_z_bias'
 * '<S67>'  : 'CF_INS/Position_Estimator/Vertical_Filter/Vel_z Predict/body to earth'
 * '<S68>'  : 'CF_INS/Position_Estimator/Vertical_Filter/Vel_z Predict/body to earth/Subsystem'
 * '<S69>'  : 'CF_INS/Position_Estimator/Vertical_Filter/Vel_z Predict/body to earth/Subsystem1'
 * '<S70>'  : 'CF_INS/Position_Estimator/Vertical_Filter/Vel_z Predict/body to earth/Subsystem2'
 * '<S71>'  : 'CF_INS/Position_Estimator/Vertical_Filter/Vel_z Predict/body to earth/quat normalize'
 * '<S72>'  : 'CF_INS/Position_Estimator/Vertical_Filter/Vel_z Predict/body to earth/quat normalize/quat modulus'
 * '<S73>'  : 'CF_INS/Position_Estimator/Vertical_Filter/Vel_z Predict/body to earth/quat normalize/quat modulus/quat norm'
 */
#endif                                 /* RTW_HEADER_CF_INS_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
