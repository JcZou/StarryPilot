/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: CF_INS.h
 *
 * Code generated for Simulink model 'CF_INS'.
 *
 * Model version                  : 1.440
 * Simulink Coder version         : 9.0 (R2018b) 24-May-2018
 * C/C++ source code generated on : Tue Dec 25 18:22:29 2018
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
  real32_T DiscreteTimeIntegrator1_DSTAT_h;/* '<S73>/Discrete-Time Integrator1' */
  real32_T DiscreteTimeIntegrator_DSTATE_i;/* '<S59>/Discrete-Time Integrator' */
  real32_T Delay_DSTATE[10];           /* '<S60>/Delay' */
  real32_T DiscreteTimeIntegrator_DSTAT_ku;/* '<S67>/Discrete-Time Integrator' */
  real32_T DiscreteTimeIntegrator1_DSTAT_c;/* '<S67>/Discrete-Time Integrator1' */
  real32_T Delay_DSTATE_j[10];         /* '<S58>/Delay' */
  int8_T DiscreteTimeIntegrator_PrevRese;/* '<S5>/Discrete-Time Integrator' */
  int8_T DiscreteTimeIntegrator1_PrevRes;/* '<S37>/Discrete-Time Integrator1' */
  uint8_T DiscreteTimeIntegrator_IC_LOADI;/* '<S5>/Discrete-Time Integrator' */
  uint8_T DiscreteTimeIntegrator1_IC_LOAD;/* '<S67>/Discrete-Time Integrator1' */
} DW_CF_INS_f_T;

#endif                                 /*CF_INS_MDLREF_HIDE_CHILD_*/

/* Invariant block signals for model 'CF_INS' */
#ifndef CF_INS_MDLREF_HIDE_CHILD_

typedef struct {
  const real32_T Constant7[3];         /* '<S37>/Constant7' */
  const real32_T Constant;             /* '<S58>/Constant' */
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
 * Block '<S64>/Scope' : Unused code path elimination
 * Block '<S67>/x_v_a_scope' : Unused code path elimination
 * Block '<S68>/Gain' : Unused code path elimination
 * Block '<S68>/Scope' : Unused code path elimination
 * Block '<S68>/Scope2' : Unused code path elimination
 * Block '<S69>/Constant' : Unused code path elimination
 * Block '<S69>/Constant1' : Unused code path elimination
 * Block '<S70>/Delay Input1' : Unused code path elimination
 * Block '<S70>/FixPt Relational Operator' : Unused code path elimination
 * Block '<S69>/Discrete-Time Integrator' : Unused code path elimination
 * Block '<S69>/Discrete-Time Integrator1' : Unused code path elimination
 * Block '<S69>/Logical Operator' : Unused code path elimination
 * Block '<S69>/Memory' : Unused code path elimination
 * Block '<S69>/Subtract' : Unused code path elimination
 * Block '<S69>/Switch' : Unused code path elimination
 * Block '<S71>/Abs' : Unused code path elimination
 * Block '<S71>/Add' : Unused code path elimination
 * Block '<S71>/Add1' : Unused code path elimination
 * Block '<S71>/Add2' : Unused code path elimination
 * Block '<S71>/Add3' : Unused code path elimination
 * Block '<S71>/Add4' : Unused code path elimination
 * Block '<S71>/Add5' : Unused code path elimination
 * Block '<S71>/Add6' : Unused code path elimination
 * Block '<S71>/Divide' : Unused code path elimination
 * Block '<S71>/Gain' : Unused code path elimination
 * Block '<S71>/Gain1' : Unused code path elimination
 * Block '<S71>/Gain2' : Unused code path elimination
 * Block '<S71>/Gain3' : Unused code path elimination
 * Block '<S71>/Gain4' : Unused code path elimination
 * Block '<S71>/Multiply' : Unused code path elimination
 * Block '<S71>/Multiply1' : Unused code path elimination
 * Block '<S71>/Multiply2' : Unused code path elimination
 * Block '<S71>/Multiply3' : Unused code path elimination
 * Block '<S71>/Multiply4' : Unused code path elimination
 * Block '<S71>/Multiply5' : Unused code path elimination
 * Block '<S71>/Multiply6' : Unused code path elimination
 * Block '<S71>/Sign' : Unused code path elimination
 * Block '<S71>/Sign1' : Unused code path elimination
 * Block '<S71>/Sign2' : Unused code path elimination
 * Block '<S71>/Sign3' : Unused code path elimination
 * Block '<S71>/Sign4' : Unused code path elimination
 * Block '<S71>/Sign5' : Unused code path elimination
 * Block '<S71>/Sign6' : Unused code path elimination
 * Block '<S71>/Sqrt' : Unused code path elimination
 * Block '<S71>/Square' : Unused code path elimination
 * Block '<S71>/Subtract' : Unused code path elimination
 * Block '<S71>/Subtract1' : Unused code path elimination
 * Block '<S71>/Subtract2' : Unused code path elimination
 * Block '<S71>/Subtract3' : Unused code path elimination
 * Block '<S71>/Subtract4' : Unused code path elimination
 * Block '<S71>/Subtract5' : Unused code path elimination
 * Block '<S71>/Subtract6' : Unused code path elimination
 * Block '<S71>/const' : Unused code path elimination
 * Block '<S71>/const1' : Unused code path elimination
 * Block '<S61>/Scope' : Unused code path elimination
 * Block '<S75>/Constant' : Unused code path elimination
 * Block '<S75>/Gain' : Unused code path elimination
 * Block '<S75>/Gain1' : Unused code path elimination
 * Block '<S75>/Gain2' : Unused code path elimination
 * Block '<S75>/Product' : Unused code path elimination
 * Block '<S75>/Product1' : Unused code path elimination
 * Block '<S75>/Product2' : Unused code path elimination
 * Block '<S75>/Product3' : Unused code path elimination
 * Block '<S75>/Product4' : Unused code path elimination
 * Block '<S75>/Product5' : Unused code path elimination
 * Block '<S75>/Product6' : Unused code path elimination
 * Block '<S75>/Product7' : Unused code path elimination
 * Block '<S75>/Product8' : Unused code path elimination
 * Block '<S75>/Sum' : Unused code path elimination
 * Block '<S75>/Sum1' : Unused code path elimination
 * Block '<S75>/Sum2' : Unused code path elimination
 * Block '<S75>/Sum3' : Unused code path elimination
 * Block '<S76>/Constant' : Unused code path elimination
 * Block '<S76>/Gain' : Unused code path elimination
 * Block '<S76>/Gain1' : Unused code path elimination
 * Block '<S76>/Gain2' : Unused code path elimination
 * Block '<S76>/Product' : Unused code path elimination
 * Block '<S76>/Product1' : Unused code path elimination
 * Block '<S76>/Product2' : Unused code path elimination
 * Block '<S76>/Product3' : Unused code path elimination
 * Block '<S76>/Product4' : Unused code path elimination
 * Block '<S76>/Product5' : Unused code path elimination
 * Block '<S76>/Product6' : Unused code path elimination
 * Block '<S76>/Product7' : Unused code path elimination
 * Block '<S76>/Product8' : Unused code path elimination
 * Block '<S76>/Sum' : Unused code path elimination
 * Block '<S76>/Sum1' : Unused code path elimination
 * Block '<S76>/Sum2' : Unused code path elimination
 * Block '<S76>/Sum3' : Unused code path elimination
 * Block '<S73>/Scope' : Unused code path elimination
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
 * '<S62>'  : 'CF_INS/Position_Estimator/Vertical_Filter/Alt Correct/Baro_AltitudeError_Calc'
 * '<S63>'  : 'CF_INS/Position_Estimator/Vertical_Filter/Alt Correct/GPS_HeightError_Calc'
 * '<S64>'  : 'CF_INS/Position_Estimator/Vertical_Filter/Alt Correct/Baro_AltitudeError_Calc/Scope'
 * '<S65>'  : 'CF_INS/Position_Estimator/Vertical_Filter/Vel_z Correct/Baro_VelError_Calc'
 * '<S66>'  : 'CF_INS/Position_Estimator/Vertical_Filter/Vel_z Correct/GPS_VelError_Calc'
 * '<S67>'  : 'CF_INS/Position_Estimator/Vertical_Filter/Vel_z Correct/Baro_VelError_Calc/Linear Observer'
 * '<S68>'  : 'CF_INS/Position_Estimator/Vertical_Filter/Vel_z Correct/Baro_VelError_Calc/Scope'
 * '<S69>'  : 'CF_INS/Position_Estimator/Vertical_Filter/Vel_z Correct/Baro_VelError_Calc/Scope/TD'
 * '<S70>'  : 'CF_INS/Position_Estimator/Vertical_Filter/Vel_z Correct/Baro_VelError_Calc/Scope/TD/Detect Change'
 * '<S71>'  : 'CF_INS/Position_Estimator/Vertical_Filter/Vel_z Correct/Baro_VelError_Calc/Scope/TD/fhan'
 * '<S72>'  : 'CF_INS/Position_Estimator/Vertical_Filter/Vel_z Predict/Subsystem'
 * '<S73>'  : 'CF_INS/Position_Estimator/Vertical_Filter/Vel_z Predict/acc_z_bias_estimation'
 * '<S74>'  : 'CF_INS/Position_Estimator/Vertical_Filter/Vel_z Predict/Subsystem/body to earth'
 * '<S75>'  : 'CF_INS/Position_Estimator/Vertical_Filter/Vel_z Predict/Subsystem/body to earth/Subsystem'
 * '<S76>'  : 'CF_INS/Position_Estimator/Vertical_Filter/Vel_z Predict/Subsystem/body to earth/Subsystem1'
 * '<S77>'  : 'CF_INS/Position_Estimator/Vertical_Filter/Vel_z Predict/Subsystem/body to earth/Subsystem2'
 * '<S78>'  : 'CF_INS/Position_Estimator/Vertical_Filter/Vel_z Predict/Subsystem/body to earth/quat normalize'
 * '<S79>'  : 'CF_INS/Position_Estimator/Vertical_Filter/Vel_z Predict/Subsystem/body to earth/quat normalize/quat modulus'
 * '<S80>'  : 'CF_INS/Position_Estimator/Vertical_Filter/Vel_z Predict/Subsystem/body to earth/quat normalize/quat modulus/quat norm'
 */
#endif                                 /* RTW_HEADER_CF_INS_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
