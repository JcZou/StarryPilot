/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: EKF_ImuJacobian.c
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

#include "EKF_ImuJacobian.h"

/* Include model header file for global data */
#include "INS.h"
#include "INS_private.h"

/* Output and update for Simulink Function: '<Root>/Simulink Function2' */
void EKF_ImuJacobian(const real32_T rtu_X[14], real32_T rty_H[42])
{
  real32_T ax;
  real32_T ay;
  real32_T acc_norm;
  real32_T tmp;
  real32_T tmp_0;
  real32_T tmp_1;
  real32_T tmp_2;
  real32_T tmp_3;
  real32_T tmp_4;
  real32_T tmp_5;

  /* SignalConversion: '<S5>/TmpSignal ConversionAtHInport1' incorporates:
   *  MATLAB Function: '<S5>/MATLAB Function'
   */
  memset(&rty_H[0], 0, 42U * sizeof(real32_T));

  /* MATLAB Function: '<S5>/MATLAB Function' incorporates:
   *  Inport: '<Root>/IMU'
   *  SignalConversion: '<S5>/TmpLatchAtUOutport1'
   *  SignalConversion: '<S5>/TmpSignal ConversionAtHInport1'
   *  SignalConversion: '<S5>/TmpSignal ConversionAtXOutport1'
   */
  ax = sqrtf((INS_U.IMU.accel[0] * INS_U.IMU.accel[0] + INS_U.IMU.accel[1] *
              INS_U.IMU.accel[1]) + INS_U.IMU.accel[2] * INS_U.IMU.accel[2]);
  if (rtIsNaNF(ax) || (ax < 0.001)) {
    acc_norm = 0.001F;
  } else {
    acc_norm = ax;
  }

  ax = INS_U.IMU.accel[0] / acc_norm;
  ay = INS_U.IMU.accel[1] / acc_norm;
  acc_norm = INS_U.IMU.accel[2] / acc_norm;
  tmp_4 = rtu_X[9] * ay;
  tmp_5 = rtu_X[8] * acc_norm;
  tmp_2 = ((rtu_X[6] * ax - tmp_4) + tmp_5) * 2.0F;
  rty_H[18] = tmp_2;
  tmp_1 = ((rtu_X[7] * ax + rtu_X[8] * ay) + rtu_X[9] * acc_norm) * 2.0F;
  rty_H[21] = tmp_1;
  tmp = rtu_X[7] * ay;
  tmp_0 = rtu_X[6] * acc_norm;
  tmp_3 = ((-rtu_X[8] * ax + tmp) + tmp_0) * 2.0F;
  rty_H[24] = tmp_3;
  ay *= rtu_X[6];
  acc_norm *= rtu_X[7];
  rty_H[27] = ((-rtu_X[9] * ax - ay) + acc_norm) * 2.0F;
  ay = ((rtu_X[9] * ax + ay) - acc_norm) * 2.0F;
  rty_H[19] = ay;
  rty_H[22] = ((rtu_X[8] * ax - tmp) - tmp_0) * 2.0F;
  rty_H[25] = tmp_1;
  rty_H[28] = tmp_2;
  rty_H[20] = tmp_3;
  rty_H[23] = ay;
  rty_H[26] = ((-rtu_X[6] * ax + tmp_4) - tmp_5) * 2.0F;
  rty_H[29] = tmp_1;
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
