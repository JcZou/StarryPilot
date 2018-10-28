/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: INS.h
 *
 * Code generated for Simulink model 'INS'.
 *
 * Model version                  : 1.141
 * Simulink Coder version         : 9.0 (R2018b) 24-May-2018
 * C/C++ source code generated on : Sun Oct 28 12:14:29 2018
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_INS_h_
#define RTW_HEADER_INS_h_
#include <math.h>
#include <string.h>
#include <stddef.h>
#ifndef INS_COMMON_INCLUDES_
# define INS_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* INS_COMMON_INCLUDES_ */

#include "INS_types.h"

/* Child system includes */
#include "EKF_BaroCorrect_private.h"
#include "EKF_BaroCorrect.h"
#include "EKF_BaroJacobian_private.h"
#include "EKF_BaroJacobian.h"
#include "EKF_GpsCorrect_private.h"
#include "EKF_GpsCorrect.h"
#include "EKF_GpsJacobian_private.h"
#include "EKF_GpsJacobian.h"
#include "EKF_ImuCorrect_private.h"
#include "EKF_ImuCorrect.h"
#include "EKF_ImuJacobian_private.h"
#include "EKF_ImuJacobian.h"
#include "EKF_MagCorrect_private.h"
#include "EKF_MagCorrect.h"
#include "EKF_MagJacobian_private.h"
#include "EKF_MagJacobian.h"
#include "EKF_Predict_private.h"
#include "EKF_Predict.h"
#include "EKF_PredictJacobian_private.h"
#include "EKF_PredictJacobian.h"
#include "rtGetInf.h"
#include "rt_nonfinite.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

/* Block signals (default storage) */
typedef struct {
  boolean_T blockOrdering_e;           /* '<S13>/Correct' */
  boolean_T blockOrdering_i;           /* '<S12>/Correct' */
} B_INS_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real32_T P[196];                     /* '<S1>/DataStoreMemory - P' */
  real32_T x[14];                      /* '<S1>/DataStoreMemory - x' */
} DW_INS_T;

/* Invariant block signals (default storage) */
typedef struct {
  const boolean_T DataTypeConversion_Enable1;/* '<S1>/DataTypeConversion_Enable1' */
  const boolean_T DataTypeConversion_Enable2;/* '<S1>/DataTypeConversion_Enable2' */
  const boolean_T DataTypeConversion_Enable3;/* '<S1>/DataTypeConversion_Enable3' */
  const boolean_T DataTypeConversion_Enable4;/* '<S1>/DataTypeConversion_Enable4' */
} ConstB_INS_T;

/* External inputs (root inport signals with default storage) */
typedef struct {
  IMU_Bus IMU;                         /* '<Root>/IMU' */
  Mag_Bus MAG;                         /* '<Root>/MAG' */
  GPS_Bus GPS;                         /* '<Root>/GPS' */
  BARO_Bus BARO;                       /* '<Root>/BARO' */
} ExtU_INS_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  State_Bus States;                    /* '<Root>/States' */
} ExtY_INS_T;

/* Real-time Model Data Structure */
struct tag_RTM_INS_T {
  const char_T * volatile errorStatus;
};

/* Block signals (default storage) */
extern B_INS_T INS_B;

/* Block states (default storage) */
extern DW_INS_T INS_DW;

/* External inputs (root inport signals with default storage) */
extern ExtU_INS_T INS_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_INS_T INS_Y;

/* External data declarations for dependent source files */
extern const State_Bus INS_rtZState_Bus;/* State_Bus ground */
extern const ConstB_INS_T INS_ConstB;  /* constant block i/o */

/* Model entry point functions */
extern void INS_initialize(void);
extern void INS_step(void);
extern void INS_terminate(void);

/* Real-time Model object */
extern RT_MODEL_INS_T *const INS_M;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S12>/RegisterSimulinkFcn' : Unused code path elimination
 * Block '<S13>/RegisterSimulinkFcn' : Unused code path elimination
 * Block '<S14>/RegisterSimulinkFcn' : Unused code path elimination
 * Block '<S15>/RegisterSimulinkFcn' : Unused code path elimination
 * Block '<S17>/RegisterSimulinkFcn' : Unused code path elimination
 * Block '<S1>/checkMeasurementFcn1Signals' : Unused code path elimination
 * Block '<S1>/checkMeasurementFcn2Signals' : Unused code path elimination
 * Block '<S1>/checkMeasurementFcn3Signals' : Unused code path elimination
 * Block '<S1>/checkMeasurementFcn4Signals' : Unused code path elimination
 * Block '<S1>/checkStateTransitionFcnSignals' : Unused code path elimination
 * Block '<S1>/DataTypeConversion_Q' : Eliminate redundant data type conversion
 * Block '<S1>/DataTypeConversion_R1' : Eliminate redundant data type conversion
 * Block '<S1>/DataTypeConversion_R2' : Eliminate redundant data type conversion
 * Block '<S1>/DataTypeConversion_R3' : Eliminate redundant data type conversion
 * Block '<S1>/DataTypeConversion_R4' : Eliminate redundant data type conversion
 * Block '<S1>/DataTypeConversion_uMeas1' : Eliminate redundant data type conversion
 * Block '<S1>/DataTypeConversion_uMeas2' : Eliminate redundant data type conversion
 * Block '<S1>/DataTypeConversion_uMeas3' : Eliminate redundant data type conversion
 * Block '<S1>/DataTypeConversion_uMeas4' : Eliminate redundant data type conversion
 * Block '<S1>/DataTypeConversion_uState' : Eliminate redundant data type conversion
 * Block '<S1>/DataTypeConversion_y1' : Eliminate redundant data type conversion
 * Block '<S1>/DataTypeConversion_y2' : Eliminate redundant data type conversion
 * Block '<S1>/DataTypeConversion_y3' : Eliminate redundant data type conversion
 * Block '<S1>/DataTypeConversion_y4' : Eliminate redundant data type conversion
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
 * '<Root>' : 'INS'
 * '<S1>'   : 'INS/EKF'
 * '<S2>'   : 'INS/Predict Function'
 * '<S3>'   : 'INS/Simulink Function'
 * '<S4>'   : 'INS/Simulink Function1'
 * '<S5>'   : 'INS/Simulink Function2'
 * '<S6>'   : 'INS/Simulink Function3'
 * '<S7>'   : 'INS/Simulink Function4'
 * '<S8>'   : 'INS/Simulink Function5'
 * '<S9>'   : 'INS/Simulink Function6'
 * '<S10>'  : 'INS/Simulink Function7'
 * '<S11>'  : 'INS/Simulink Function8'
 * '<S12>'  : 'INS/EKF/Correct1'
 * '<S13>'  : 'INS/EKF/Correct2'
 * '<S14>'  : 'INS/EKF/Correct3'
 * '<S15>'  : 'INS/EKF/Correct4'
 * '<S16>'  : 'INS/EKF/Output'
 * '<S17>'  : 'INS/EKF/Predict'
 * '<S18>'  : 'INS/EKF/Correct1/Correct'
 * '<S19>'  : 'INS/EKF/Correct2/Correct'
 * '<S20>'  : 'INS/EKF/Correct3/Correct'
 * '<S21>'  : 'INS/EKF/Correct4/Correct'
 * '<S22>'  : 'INS/EKF/Predict/Predict'
 * '<S23>'  : 'INS/Predict Function/MATLAB Function'
 * '<S24>'  : 'INS/Simulink Function/MATLAB Function'
 * '<S25>'  : 'INS/Simulink Function1/MATLAB Function'
 * '<S26>'  : 'INS/Simulink Function2/MATLAB Function'
 * '<S27>'  : 'INS/Simulink Function3/MATLAB Function'
 * '<S28>'  : 'INS/Simulink Function4/MATLAB Function'
 * '<S29>'  : 'INS/Simulink Function5/MATLAB Function'
 * '<S30>'  : 'INS/Simulink Function6/MATLAB Function'
 * '<S31>'  : 'INS/Simulink Function7/MATLAB Function'
 * '<S32>'  : 'INS/Simulink Function8/MATLAB Function'
 */
#endif                                 /* RTW_HEADER_INS_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
