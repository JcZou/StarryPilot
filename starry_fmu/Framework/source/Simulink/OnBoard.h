/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: OnBoard.h
 *
 * Code generated for Simulink model 'OnBoard'.
 *
 * Model version                  : 1.1010
 * Simulink Coder version         : 9.0 (R2018b) 24-May-2018
 * C/C++ source code generated on : Sun Oct 21 17:18:49 2018
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#ifndef RTW_HEADER_OnBoard_h_
#define RTW_HEADER_OnBoard_h_
#include <math.h>
#ifndef OnBoard_COMMON_INCLUDES_
# define OnBoard_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* OnBoard_COMMON_INCLUDES_ */

/* Macros for accessing real-time model data structure */

/* Block signals and states (default storage) for system '<Root>' */
typedef struct {
  real32_T DiscreteTimeIntegrator_DSTATE;/* '<S9>/Discrete-Time Integrator' */
  real32_T DiscreteTimeIntegrator_DSTATE_l;/* '<S6>/Discrete-Time Integrator' */
  real32_T DiscreteTimeIntegrator1_DSTATE;/* '<S3>/Discrete-Time Integrator1' */
  real32_T DiscreteTimeIntegrator2_DSTATE;/* '<S3>/Discrete-Time Integrator2' */
  real32_T DiscreteTimeIntegrator_DSTATE_f;/* '<S8>/Discrete-Time Integrator' */
  real32_T DiscreteTimeIntegrator_DSTATE_h;/* '<S7>/Discrete-Time Integrator' */
} D_Work;

/* External inputs (root inport signals with default storage) */
typedef struct {
  int32_T alpha_m_l;                   /* '<Root>/alpha_m_l' */
  int32_T alpha_m_r;                   /* '<Root>/alpha_m_r' */
  real32_T theta_rad_m;                /* '<Root>/theta_rad_m' */
  real32_T theta_dot_radDs_m;          /* '<Root>/theta_dot_radDs_m' */
} ExternalInputs;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real32_T pwm_l;                      /* '<Root>/pwm_l' */
  real32_T pwm_r;                      /* '<Root>/pwm_r' */
} ExternalOutputs;

/* Block signals and states (default storage) */
extern D_Work rtDWork;

/* External inputs (root inport signals with default storage) */
extern ExternalInputs rtU;

/* External outputs (root outports fed by signals with default storage) */
extern ExternalOutputs rtY;

/* Model entry point functions */
extern void OnBoard_initialize(void);
extern void OnBoard_step(void);

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S5>/Scope' : Unused code path elimination
 * Block '<S5>/Sum1' : Unused code path elimination
 * Block '<S1>/Pilot_cmd1' : Unused code path elimination
 * Block '<S5>/Gain6' : Eliminated nontunable gain of 1
 * Block '<S5>/Gain7' : Eliminated nontunable gain of 1
 * Block '<S10>/Gain' : Eliminated nontunable gain of 1
 * Block '<S11>/Gain' : Eliminated nontunable gain of 1
 */

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Note that this particular code originates from a subsystem build,
 * and has its own system numbers different from the parent model.
 * Refer to the system hierarchy for this subsystem below, and use the
 * MATLAB hilite_system command to trace the generated code back
 * to the parent model.  For example,
 *
 * hilite_system('BackupController/Balance & Drive Control/OnBoard')    - opens subsystem BackupController/Balance & Drive Control/OnBoard
 * hilite_system('BackupController/Balance & Drive Control/OnBoard/Kp') - opens and selects block Kp
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'BackupController/Balance & Drive Control'
 * '<S1>'   : 'BackupController/Balance & Drive Control/OnBoard'
 * '<S2>'   : 'BackupController/Balance & Drive Control/OnBoard/Controller1'
 * '<S3>'   : 'BackupController/Balance & Drive Control/OnBoard/INS_Interface'
 * '<S4>'   : 'BackupController/Balance & Drive Control/OnBoard/PWM_Mapping'
 * '<S5>'   : 'BackupController/Balance & Drive Control/OnBoard/Controller1/Subsystem'
 * '<S6>'   : 'BackupController/Balance & Drive Control/OnBoard/Controller1/Subsystem/Differentiator'
 * '<S7>'   : 'BackupController/Balance & Drive Control/OnBoard/Controller1/Subsystem/filter'
 * '<S8>'   : 'BackupController/Balance & Drive Control/OnBoard/INS_Interface/Differentiator'
 * '<S9>'   : 'BackupController/Balance & Drive Control/OnBoard/INS_Interface/filter'
 * '<S10>'  : 'BackupController/Balance & Drive Control/OnBoard/PWM_Mapping/Friction Compensator'
 * '<S11>'  : 'BackupController/Balance & Drive Control/OnBoard/PWM_Mapping/Friction Compensator1'
 */
#endif                                 /* RTW_HEADER_OnBoard_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
