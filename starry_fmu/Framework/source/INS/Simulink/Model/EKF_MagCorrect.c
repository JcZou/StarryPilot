/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: EKF_MagCorrect.c
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

#include "EKF_MagCorrect.h"

/* Include model header file for global data */
#include "INS.h"
#include "INS_private.h"

/* Output and update for Simulink Function: '<Root>/Simulink Function3' */
void EKF_MagCorrect(const real32_T rtu_X[14], real32_T rty_y[2])
{
  real32_T mx;
  real32_T my;
  real32_T mag_norm;
  real32_T t;
  real32_T magN_idx_0;
  real32_T magN_idx_1;
  real32_T magN_idx_0_tmp;
  real32_T magN_idx_0_tmp_0;
  real32_T magN_idx_0_tmp_1;
  real32_T magN_idx_0_tmp_2;

  /* MATLAB Function: '<S6>/MATLAB Function' incorporates:
   *  Inport: '<Root>/MAG'
   *  SignalConversion: '<S6>/TmpLatchAtUOutport1'
   *  SignalConversion: '<S6>/TmpSignal ConversionAtXOutport1'
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
  magN_idx_1 = rtu_X[6] * rtu_X[6];
  t = rtu_X[7] * rtu_X[7];
  magN_idx_0_tmp = rtu_X[8] * rtu_X[8];
  magN_idx_0_tmp_0 = rtu_X[9] * rtu_X[9];
  magN_idx_0_tmp_1 = rtu_X[7] * rtu_X[8];
  magN_idx_0_tmp_2 = rtu_X[6] * rtu_X[9];
  magN_idx_0 = ((((magN_idx_1 + t) - magN_idx_0_tmp) - magN_idx_0_tmp_0) * mx +
                (magN_idx_0_tmp_1 - magN_idx_0_tmp_2) * 2.0F * my) + (rtu_X[7] *
    rtu_X[9] + rtu_X[6] * rtu_X[8]) * 2.0F * mag_norm;
  magN_idx_1 = ((((magN_idx_1 - t) + magN_idx_0_tmp) - magN_idx_0_tmp_0) * my +
                (magN_idx_0_tmp_1 + magN_idx_0_tmp_2) * 2.0F * mx) + (rtu_X[8] *
    rtu_X[9] - rtu_X[6] * rtu_X[7]) * 2.0F * mag_norm;
  my = 1.29246971E-26F;
  mag_norm = fabsf(magN_idx_0);
  if (mag_norm > 1.29246971E-26F) {
    mx = 1.0F;
    my = mag_norm;
  } else {
    t = mag_norm / 1.29246971E-26F;
    mx = t * t;
  }

  mag_norm = fabsf(magN_idx_1);
  if (mag_norm > my) {
    t = my / mag_norm;
    mx = mx * t * t + 1.0F;
    my = mag_norm;
  } else {
    t = mag_norm / my;
    mx += t * t;
  }

  mx = my * sqrtf(mx);
  if (rtIsNaNF(mx) || (mx < 0.001)) {
    mx = 0.001F;
  }

  /* SignalConversion: '<S6>/TmpSignal ConversionAtyInport1' incorporates:
   *  MATLAB Function: '<S6>/MATLAB Function'
   */
  rty_y[0] = magN_idx_0 / mx;
  rty_y[1] = magN_idx_1 / mx;
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
