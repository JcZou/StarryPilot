/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: EKF_Predict.c
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

#include "EKF_Predict.h"

/* Include model header file for global data */
#include "INS.h"
#include "INS_private.h"

/* Output and update for Simulink Function: '<Root>/Predict Function' */
void EKF_Predict(const real32_T rtu_X[14], const real32_T rtu_W[10], real32_T
                 rty_NextX[14])
{
  real32_T gx;
  real32_T gy;
  real32_T gz;
  real32_T az;
  real32_T rtb_TmpSignalConversionAtXOutpo[14];
  real32_T rtb_NextX[14];
  int32_T i;
  real32_T rtb_NextX_tmp;
  real32_T rtb_NextX_tmp_0;
  real32_T rtb_NextX_tmp_1;
  real32_T rtb_NextX_tmp_2;
  real32_T rtb_NextX_tmp_3;
  real32_T rtb_NextX_tmp_4;
  real32_T rtb_NextX_tmp_5;
  real32_T rtb_NextX_tmp_6;
  real32_T rtb_NextX_tmp_7;
  real32_T rtb_NextX_tmp_8;
  UNUSED_PARAMETER(rtu_W);

  /* SignalConversion: '<S2>/TmpSignal ConversionAtXOutport1' */
  for (i = 0; i < 14; i++) {
    rtb_TmpSignalConversionAtXOutpo[i] = rtu_X[i];
  }

  /* End of SignalConversion: '<S2>/TmpSignal ConversionAtXOutport1' */

  /* MATLAB Function: '<S2>/MATLAB Function' incorporates:
   *  Constant: '<Root>/Ts'
   *  Inport: '<Root>/IMU'
   *  SignalConversion: '<S2>/TmpLatchAtTsOutport1'
   *  SignalConversion: '<S2>/TmpLatchAtUOutport1'
   */
  gx = INS_U.IMU.gyro[0] - rtb_TmpSignalConversionAtXOutpo[10];
  gy = INS_U.IMU.gyro[1] - rtb_TmpSignalConversionAtXOutpo[11];
  gz = INS_U.IMU.gyro[2] - rtb_TmpSignalConversionAtXOutpo[12];
  az = INS_U.IMU.accel[2] - rtb_TmpSignalConversionAtXOutpo[13];
  rtb_NextX[0] = 0.004F * rtb_TmpSignalConversionAtXOutpo[3] +
    rtb_TmpSignalConversionAtXOutpo[0];
  rtb_NextX[1] = 0.004F * rtb_TmpSignalConversionAtXOutpo[4] +
    rtb_TmpSignalConversionAtXOutpo[1];
  rtb_NextX[2] = 0.004F * rtb_TmpSignalConversionAtXOutpo[5] +
    rtb_TmpSignalConversionAtXOutpo[2];
  rtb_NextX_tmp = rtb_TmpSignalConversionAtXOutpo[6] *
    rtb_TmpSignalConversionAtXOutpo[6];
  rtb_NextX_tmp_0 = rtb_TmpSignalConversionAtXOutpo[7] *
    rtb_TmpSignalConversionAtXOutpo[7];
  rtb_NextX_tmp_1 = rtb_TmpSignalConversionAtXOutpo[8] *
    rtb_TmpSignalConversionAtXOutpo[8];
  rtb_NextX_tmp_2 = rtb_TmpSignalConversionAtXOutpo[9] *
    rtb_TmpSignalConversionAtXOutpo[9];
  rtb_NextX_tmp_3 = rtb_TmpSignalConversionAtXOutpo[7] *
    rtb_TmpSignalConversionAtXOutpo[8];
  rtb_NextX_tmp_4 = rtb_TmpSignalConversionAtXOutpo[6] *
    rtb_TmpSignalConversionAtXOutpo[9];
  rtb_NextX_tmp_5 = rtb_TmpSignalConversionAtXOutpo[7] *
    rtb_TmpSignalConversionAtXOutpo[9];
  rtb_NextX_tmp_6 = rtb_TmpSignalConversionAtXOutpo[6] *
    rtb_TmpSignalConversionAtXOutpo[8];
  rtb_NextX[3] = (((((rtb_NextX_tmp + rtb_NextX_tmp_0) - rtb_NextX_tmp_1) -
                    rtb_NextX_tmp_2) * INS_U.IMU.accel[0] + (rtb_NextX_tmp_3 -
    rtb_NextX_tmp_4) * 2.0F * INS_U.IMU.accel[1]) + (rtb_NextX_tmp_5 +
    rtb_NextX_tmp_6) * 2.0F * az) * 0.004F + rtb_TmpSignalConversionAtXOutpo[3];
  rtb_NextX_tmp_7 = rtb_TmpSignalConversionAtXOutpo[8] *
    rtb_TmpSignalConversionAtXOutpo[9];
  rtb_NextX_tmp_8 = rtb_TmpSignalConversionAtXOutpo[6] *
    rtb_TmpSignalConversionAtXOutpo[7];
  rtb_NextX_tmp -= rtb_NextX_tmp_0;
  rtb_NextX[4] = ((((rtb_NextX_tmp + rtb_NextX_tmp_1) - rtb_NextX_tmp_2) *
                   INS_U.IMU.accel[1] + (rtb_NextX_tmp_3 + rtb_NextX_tmp_4) *
                   2.0F * INS_U.IMU.accel[0]) + (rtb_NextX_tmp_7 -
    rtb_NextX_tmp_8) * 2.0F * az) * 0.004F + rtb_TmpSignalConversionAtXOutpo[4];
  rtb_NextX[5] = ((((rtb_NextX_tmp_5 - rtb_NextX_tmp_6) * 2.0F *
                    INS_U.IMU.accel[0] + (rtb_NextX_tmp_7 + rtb_NextX_tmp_8) *
                    2.0F * INS_U.IMU.accel[1]) + ((rtb_NextX_tmp -
    rtb_NextX_tmp_1) + rtb_NextX_tmp_2) * az) + 9.81F) * 0.004F +
    rtb_TmpSignalConversionAtXOutpo[5];
  rtb_NextX[6] = ((-rtb_TmpSignalConversionAtXOutpo[7] * gx -
                   rtb_TmpSignalConversionAtXOutpo[8] * gy) -
                  rtb_TmpSignalConversionAtXOutpo[9] * gz) / 2.0F * 0.004F +
    rtb_TmpSignalConversionAtXOutpo[6];
  rtb_NextX[7] = ((rtb_TmpSignalConversionAtXOutpo[6] * gx -
                   rtb_TmpSignalConversionAtXOutpo[9] * gy) +
                  rtb_TmpSignalConversionAtXOutpo[8] * gz) / 2.0F * 0.004F +
    rtb_TmpSignalConversionAtXOutpo[7];
  rtb_NextX[8] = ((rtb_TmpSignalConversionAtXOutpo[9] * gx +
                   rtb_TmpSignalConversionAtXOutpo[6] * gy) -
                  rtb_TmpSignalConversionAtXOutpo[7] * gz) / 2.0F * 0.004F +
    rtb_TmpSignalConversionAtXOutpo[8];
  rtb_NextX[9] = ((-rtb_TmpSignalConversionAtXOutpo[8] * gx +
                   rtb_TmpSignalConversionAtXOutpo[7] * gy) +
                  rtb_TmpSignalConversionAtXOutpo[6] * gz) / 2.0F * 0.004F +
    rtb_TmpSignalConversionAtXOutpo[9];
  rtb_NextX[10] = rtb_TmpSignalConversionAtXOutpo[10];
  rtb_NextX[11] = rtb_TmpSignalConversionAtXOutpo[11];
  rtb_NextX[12] = rtb_TmpSignalConversionAtXOutpo[12];
  rtb_NextX[13] = rtb_TmpSignalConversionAtXOutpo[13];
  gx = sqrtf(((rtb_NextX[6] * rtb_NextX[6] + rtb_NextX[7] * rtb_NextX[7]) +
              rtb_NextX[8] * rtb_NextX[8]) + rtb_NextX[9] * rtb_NextX[9]);
  rtb_NextX[6] /= gx;
  rtb_NextX[7] /= gx;
  rtb_NextX[8] /= gx;
  rtb_NextX[9] /= gx;

  /* SignalConversion: '<S2>/TmpSignal ConversionAtNextXInport1' */
  for (i = 0; i < 14; i++) {
    rty_NextX[i] = rtb_NextX[i];
  }

  /* End of SignalConversion: '<S2>/TmpSignal ConversionAtNextXInport1' */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
