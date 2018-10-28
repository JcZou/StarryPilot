/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: EKF_BaroJacobian.c
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

#include "EKF_BaroJacobian.h"

/* Include model header file for global data */
#include "INS.h"
#include "INS_private.h"

/* Output and update for Simulink Function: '<Root>/Simulink Function6' */
void EKF_BaroJacobian(const real32_T rtu_X[14], real32_T rty_H[14])
{
  int32_T i;
  UNUSED_PARAMETER(rtu_X);
  for (i = 0; i < 14; i++) {
    /* SignalConversion: '<S9>/TmpSignal ConversionAtHInport1' incorporates:
     *  MATLAB Function: '<S9>/MATLAB Function'
     */
    rty_H[i] = 0.0F;
  }

  /* MATLAB Function: '<S9>/MATLAB Function' incorporates:
   *  SignalConversion: '<S9>/TmpSignal ConversionAtHInport1'
   */
  rty_H[2] = 1.0F;
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
