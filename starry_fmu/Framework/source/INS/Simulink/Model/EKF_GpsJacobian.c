/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: EKF_GpsJacobian.c
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

#include "EKF_GpsJacobian.h"

/* Include model header file for global data */
#include "INS.h"
#include "INS_private.h"

/* Output and update for Simulink Function: '<Root>/Simulink Function8' */
void EKF_GpsJacobian(const real32_T rtu_X[14], real32_T rty_H[28])
{
  UNUSED_PARAMETER(rtu_X);

  /* SignalConversion: '<S11>/TmpSignal ConversionAtHInport1' incorporates:
   *  MATLAB Function: '<S11>/MATLAB Function'
   */
  memset(&rty_H[0], 0, 28U * sizeof(real32_T));

  /* MATLAB Function: '<S11>/MATLAB Function' incorporates:
   *  SignalConversion: '<S11>/TmpSignal ConversionAtHInport1'
   */
  rty_H[0] = 1.0F;
  rty_H[3] = 1.0F;
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
