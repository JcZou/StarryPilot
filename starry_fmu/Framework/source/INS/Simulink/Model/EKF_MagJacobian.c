/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: EKF_MagJacobian.c
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

#include "EKF_MagJacobian.h"

/* Include model header file for global data */
#include "INS.h"
#include "INS_private.h"

/* Output and update for Simulink Function: '<Root>/Simulink Function4' */
void EKF_MagJacobian(const real32_T rtu_X[14], real32_T rty_H[28])
{
  real32_T mx;
  real32_T my;
  real32_T mag_norm;
  real32_T tmp;
  real32_T tmp_0;
  real32_T tmp_1;
  real32_T tmp_2;

  /* SignalConversion: '<S7>/TmpSignal ConversionAtHInport1' incorporates:
   *  MATLAB Function: '<S7>/MATLAB Function'
   */
  memset(&rty_H[0], 0, 28U * sizeof(real32_T));

  /* MATLAB Function: '<S7>/MATLAB Function' incorporates:
   *  Inport: '<Root>/MAG'
   *  SignalConversion: '<S7>/TmpLatchAtUOutport1'
   *  SignalConversion: '<S7>/TmpSignal ConversionAtHInport1'
   *  SignalConversion: '<S7>/TmpSignal ConversionAtXOutport1'
   */
  mx = sqrtf((INS_U.MAG.mag[0] * INS_U.MAG.mag[0] + INS_U.MAG.mag[1] *
              INS_U.MAG.mag[1]) + INS_U.MAG.mag[2] * INS_U.MAG.mag[2]);
  if (rtIsNaNF(mx) || (mx < 0.001)) {
    mag_norm = 0.001F;
  } else {
    mag_norm = mx;
  }

  mx = INS_U.MAG.mag[0] / mag_norm;
  my = INS_U.MAG.mag[1] / mag_norm;
  mag_norm = INS_U.MAG.mag[2] / mag_norm;
  tmp_2 = ((rtu_X[6] * mx - rtu_X[9] * my) + rtu_X[8] * mag_norm) * 2.0F;
  rty_H[12] = tmp_2;
  tmp_1 = ((rtu_X[7] * mx + rtu_X[8] * my) + rtu_X[9] * mag_norm) * 2.0F;
  rty_H[14] = tmp_1;
  tmp = rtu_X[7] * my;
  tmp_0 = rtu_X[6] * mag_norm;
  rty_H[16] = ((-rtu_X[8] * mx + tmp) + tmp_0) * 2.0F;
  my *= rtu_X[6];
  mag_norm *= rtu_X[7];
  rty_H[18] = ((-rtu_X[9] * mx - my) + mag_norm) * 2.0F;
  rty_H[13] = ((rtu_X[9] * mx + my) - mag_norm) * 2.0F;
  rty_H[15] = ((rtu_X[8] * mx - tmp) - tmp_0) * 2.0F;
  rty_H[17] = tmp_1;
  rty_H[19] = tmp_2;
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
