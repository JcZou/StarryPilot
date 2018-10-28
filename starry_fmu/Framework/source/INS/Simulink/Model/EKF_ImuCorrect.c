/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: EKF_ImuCorrect.c
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

#include "EKF_ImuCorrect.h"

/* Include model header file for global data */
#include "INS.h"
#include "INS_private.h"

/* Output and update for Simulink Function: '<Root>/Simulink Function' */
void EKF_ImuCorrect(const real32_T rtu_X[14], real32_T rty_y[3])
{
  real32_T ax;
  real32_T ay;
  real32_T acc_norm;
  real32_T t;
  real32_T accN_idx_0;
  real32_T accN_idx_1;
  real32_T accN_idx_2;
  real32_T accN_idx_0_tmp;
  real32_T accN_idx_0_tmp_0;
  real32_T accN_idx_0_tmp_1;
  real32_T accN_idx_0_tmp_2;
  real32_T accN_idx_0_tmp_3;
  real32_T accN_idx_1_tmp;
  real32_T accN_idx_1_tmp_0;

  /* MATLAB Function: '<S3>/MATLAB Function' incorporates:
   *  Inport: '<Root>/IMU'
   *  SignalConversion: '<S3>/TmpLatchAtUOutport1'
   *  SignalConversion: '<S3>/TmpSignal ConversionAtXOutport1'
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
  accN_idx_0_tmp = rtu_X[6] * rtu_X[6];
  accN_idx_0_tmp_0 = rtu_X[7] * rtu_X[7];
  accN_idx_2 = rtu_X[8] * rtu_X[8];
  t = rtu_X[9] * rtu_X[9];
  accN_idx_1 = rtu_X[7] * rtu_X[8];
  accN_idx_0_tmp_1 = rtu_X[6] * rtu_X[9];
  accN_idx_0_tmp_2 = rtu_X[7] * rtu_X[9];
  accN_idx_0_tmp_3 = rtu_X[6] * rtu_X[8];
  accN_idx_0 = ((((accN_idx_0_tmp + accN_idx_0_tmp_0) - accN_idx_2) - t) * ax +
                (accN_idx_1 - accN_idx_0_tmp_1) * 2.0F * ay) + (accN_idx_0_tmp_2
    + accN_idx_0_tmp_3) * 2.0F * acc_norm;
  accN_idx_1_tmp = rtu_X[8] * rtu_X[9];
  accN_idx_1_tmp_0 = rtu_X[6] * rtu_X[7];
  accN_idx_0_tmp -= accN_idx_0_tmp_0;
  accN_idx_1 = (((accN_idx_0_tmp + accN_idx_2) - t) * ay + (accN_idx_1 +
    accN_idx_0_tmp_1) * 2.0F * ax) + (accN_idx_1_tmp - accN_idx_1_tmp_0) * 2.0F *
    acc_norm;
  accN_idx_2 = ((accN_idx_0_tmp_2 - accN_idx_0_tmp_3) * 2.0F * ax +
                (accN_idx_1_tmp + accN_idx_1_tmp_0) * 2.0F * ay) +
    ((accN_idx_0_tmp - accN_idx_2) + t) * acc_norm;
  ay = 1.29246971E-26F;
  acc_norm = fabsf(accN_idx_0);
  if (acc_norm > 1.29246971E-26F) {
    ax = 1.0F;
    ay = acc_norm;
  } else {
    t = acc_norm / 1.29246971E-26F;
    ax = t * t;
  }

  acc_norm = fabsf(accN_idx_1);
  if (acc_norm > ay) {
    t = ay / acc_norm;
    ax = ax * t * t + 1.0F;
    ay = acc_norm;
  } else {
    t = acc_norm / ay;
    ax += t * t;
  }

  acc_norm = fabsf(accN_idx_2);
  if (acc_norm > ay) {
    t = ay / acc_norm;
    ax = ax * t * t + 1.0F;
    ay = acc_norm;
  } else {
    t = acc_norm / ay;
    ax += t * t;
  }

  ax = ay * sqrtf(ax);
  if (rtIsNaNF(ax) || (ax < 0.001)) {
    ax = 0.001F;
  }

  /* SignalConversion: '<S3>/TmpSignal ConversionAtyInport1' incorporates:
   *  MATLAB Function: '<S3>/MATLAB Function'
   */
  rty_y[0] = accN_idx_0 / ax;
  rty_y[1] = accN_idx_1 / ax;
  rty_y[2] = accN_idx_2 / ax;
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
