/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: EKF_PredictJacobian.c
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

#include "EKF_PredictJacobian.h"

/* Include model header file for global data */
#include "INS.h"
#include "INS_private.h"

/* Output and update for Simulink Function: '<Root>/Simulink Function1' */
void EKF_PredictJacobian(const real32_T rtu_X[14], const real32_T rtu_W[10],
  real32_T rty_F[196], real32_T rty_G[140])
{
  real32_T X[14];
  real32_T gx;
  real32_T gy;
  real32_T gz;
  real32_T az;
  int8_T b_I[196];
  real32_T rtb_nextF[196];
  real32_T rtb_nextG[140];
  int32_T i;
  real32_T rtb_nextF_tmp;
  real32_T rtb_nextF_tmp_0;
  real32_T rtb_nextF_tmp_1;
  real32_T rtb_nextF_tmp_2;
  real32_T rtb_nextF_tmp_3;
  real32_T rtb_nextF_tmp_4;
  real32_T rtb_nextF_tmp_tmp;
  real32_T rtb_nextF_tmp_tmp_0;
  UNUSED_PARAMETER(rtu_W);

  /* MATLAB Function: '<S4>/MATLAB Function' incorporates:
   *  Inport: '<Root>/IMU'
   *  SignalConversion: '<S4>/TmpLatchAtUOutport1'
   *  SignalConversion: '<S4>/TmpSignal ConversionAtXOutport1'
   */
  for (i = 0; i < 14; i++) {
    X[i] = rtu_X[i];
  }

  memset(&rtb_nextF[0], 0, 196U * sizeof(real32_T));
  memset(&b_I[0], 0, 196U * sizeof(int8_T));
  for (i = 0; i < 14; i++) {
    b_I[i + 14 * i] = 1;
  }

  memset(&rtb_nextG[0], 0, 140U * sizeof(real32_T));
  gx = sqrtf(((rtu_X[6] * rtu_X[6] + rtu_X[7] * rtu_X[7]) + rtu_X[8] * rtu_X[8])
             + rtu_X[9] * rtu_X[9]);
  X[6] = rtu_X[6] / gx;
  X[7] = rtu_X[7] / gx;
  X[8] = rtu_X[8] / gx;
  X[9] = rtu_X[9] / gx;
  gx = INS_U.IMU.gyro[0] - X[10];
  gy = INS_U.IMU.gyro[1] - X[11];
  gz = INS_U.IMU.gyro[2] - X[12];
  az = INS_U.IMU.accel[2] - X[13];
  rtb_nextF[42] = 1.0F;
  rtb_nextF[57] = 1.0F;
  rtb_nextF[72] = 1.0F;
  rtb_nextF_tmp_tmp = X[9] * INS_U.IMU.accel[1];
  rtb_nextF_tmp_tmp_0 = X[8] * az;
  rtb_nextF_tmp_3 = ((X[6] * INS_U.IMU.accel[0] - rtb_nextF_tmp_tmp) +
                     rtb_nextF_tmp_tmp_0) * 2.0F;
  rtb_nextF[87] = rtb_nextF_tmp_3;
  rtb_nextF_tmp_2 = ((X[7] * INS_U.IMU.accel[0] + X[8] * INS_U.IMU.accel[1]) +
                     X[9] * az) * 2.0F;
  rtb_nextF[101] = rtb_nextF_tmp_2;
  rtb_nextF_tmp_0 = X[7] * INS_U.IMU.accel[1];
  rtb_nextF_tmp_1 = X[6] * az;
  rtb_nextF_tmp_4 = ((-X[8] * INS_U.IMU.accel[0] + rtb_nextF_tmp_0) +
                     rtb_nextF_tmp_1) * 2.0F;
  rtb_nextF[115] = rtb_nextF_tmp_4;
  rtb_nextF_tmp = X[6] * INS_U.IMU.accel[1];
  az *= X[7];
  rtb_nextF[129] = ((-X[9] * INS_U.IMU.accel[0] - rtb_nextF_tmp) + az) * 2.0F;
  rtb_nextF_tmp = ((X[9] * INS_U.IMU.accel[0] + rtb_nextF_tmp) - az) * 2.0F;
  rtb_nextF[88] = rtb_nextF_tmp;
  rtb_nextF[102] = ((X[8] * INS_U.IMU.accel[0] - rtb_nextF_tmp_0) -
                    rtb_nextF_tmp_1) * 2.0F;
  rtb_nextF[116] = rtb_nextF_tmp_2;
  rtb_nextF[130] = rtb_nextF_tmp_3;
  rtb_nextF[89] = rtb_nextF_tmp_4;
  rtb_nextF[103] = rtb_nextF_tmp;
  rtb_nextF[117] = ((-X[6] * INS_U.IMU.accel[0] + rtb_nextF_tmp_tmp) -
                    rtb_nextF_tmp_tmp_0) * 2.0F;
  rtb_nextF[131] = rtb_nextF_tmp_2;
  rtb_nextF_tmp_3 = X[6] * X[7];
  rtb_nextF[186] = (-X[8] * X[9] + rtb_nextF_tmp_3) * 2.0F;
  rtb_nextF_tmp_2 = X[7] * X[7];
  rtb_nextF_tmp_0 = X[8] * X[8];
  rtb_nextF_tmp_1 = X[9] * X[9];
  rtb_nextF[187] = ((-X[6] * X[6] + rtb_nextF_tmp_2) + rtb_nextF_tmp_0) -
    rtb_nextF_tmp_1;
  rtb_nextF[90] = 0.0F;
  rtb_nextF_tmp_4 = -gx / 2.0F;
  rtb_nextF[104] = rtb_nextF_tmp_4;
  rtb_nextF_tmp = -gy / 2.0F;
  rtb_nextF[118] = rtb_nextF_tmp;
  az = -gz / 2.0F;
  rtb_nextF[132] = az;
  rtb_nextF[91] = gx / 2.0F;
  rtb_nextF[105] = 0.0F;
  rtb_nextF[119] = gz / 2.0F;
  rtb_nextF[133] = rtb_nextF_tmp;
  rtb_nextF[92] = gy / 2.0F;
  rtb_nextF[106] = az;
  rtb_nextF[120] = 0.0F;
  rtb_nextF[134] = gx / 2.0F;
  rtb_nextF[93] = gz / 2.0F;
  rtb_nextF[107] = gy / 2.0F;
  rtb_nextF[121] = rtb_nextF_tmp_4;
  rtb_nextF[135] = 0.0F;
  rtb_nextF[146] = X[7] / 2.0F;
  rtb_nextF[160] = X[8] / 2.0F;
  rtb_nextF[174] = X[9] / 2.0F;
  rtb_nextF_tmp_4 = -X[6] / 2.0F;
  rtb_nextF[147] = rtb_nextF_tmp_4;
  rtb_nextF[161] = X[9] / 2.0F;
  rtb_nextF_tmp = -X[8] / 2.0F;
  rtb_nextF[175] = rtb_nextF_tmp;
  az = -X[9] / 2.0F;
  rtb_nextF[148] = az;
  rtb_nextF[162] = rtb_nextF_tmp_4;
  rtb_nextF[176] = X[7] / 2.0F;
  rtb_nextF[149] = X[8] / 2.0F;
  gx = -X[7] / 2.0F;
  rtb_nextF[163] = gx;
  rtb_nextF[177] = rtb_nextF_tmp_4;
  for (i = 0; i < 196; i++) {
    /* SignalConversion: '<S4>/TmpSignal ConversionAtFInport1' incorporates:
     *  Constant: '<Root>/Ts'
     *  MATLAB Function: '<S4>/MATLAB Function'
     *  SignalConversion: '<S4>/TmpLatchAtTsOutport1'
     */
    rty_F[i] = rtb_nextF[i] * 0.004F + (real32_T)b_I[i];
  }

  /* MATLAB Function: '<S4>/MATLAB Function' */
  rtb_nextF_tmp_4 = X[6] * X[6];
  rtb_nextG[45] = ((rtb_nextF_tmp_4 + rtb_nextF_tmp_2) - rtb_nextF_tmp_0) -
    rtb_nextF_tmp_1;
  gy = X[7] * X[8];
  gz = X[6] * X[9];
  rtb_nextG[59] = (gy - gz) * 2.0F;
  rtb_nextF_tmp_tmp = X[7] * X[9];
  rtb_nextF_tmp_tmp_0 = X[6] * X[8];
  rtb_nextG[73] = (rtb_nextF_tmp_tmp + rtb_nextF_tmp_tmp_0) * 2.0F;
  rtb_nextG[46] = (gy + gz) * 2.0F;
  rtb_nextF_tmp_4 -= rtb_nextF_tmp_2;
  rtb_nextG[60] = (rtb_nextF_tmp_4 + rtb_nextF_tmp_0) - rtb_nextF_tmp_1;
  gy = X[8] * X[9];
  rtb_nextG[74] = (gy - rtb_nextF_tmp_3) * 2.0F;
  rtb_nextG[47] = (rtb_nextF_tmp_tmp - rtb_nextF_tmp_tmp_0) * 2.0F;
  rtb_nextG[61] = (gy + rtb_nextF_tmp_3) * 2.0F;
  rtb_nextG[75] = (rtb_nextF_tmp_4 - rtb_nextF_tmp_0) + rtb_nextF_tmp_1;
  rtb_nextG[6] = gx;
  rtb_nextG[20] = rtb_nextF_tmp;
  rtb_nextG[34] = az;
  rtb_nextG[7] = X[6] / 2.0F;
  rtb_nextG[21] = az;
  rtb_nextG[35] = X[8] / 2.0F;
  rtb_nextG[8] = X[9] / 2.0F;
  rtb_nextG[22] = X[6] / 2.0F;
  rtb_nextG[36] = gx;
  rtb_nextG[9] = rtb_nextF_tmp;
  rtb_nextG[23] = X[7] / 2.0F;
  rtb_nextG[37] = X[6] / 2.0F;
  rtb_nextG[94] = 1.0F;
  rtb_nextG[109] = 1.0F;
  rtb_nextG[124] = 1.0F;
  rtb_nextG[139] = 1.0F;
  for (i = 0; i < 140; i++) {
    /* SignalConversion: '<S4>/TmpSignal ConversionAtGInport1' incorporates:
     *  Constant: '<Root>/Ts'
     *  MATLAB Function: '<S4>/MATLAB Function'
     *  SignalConversion: '<S4>/TmpLatchAtTsOutport1'
     */
    rty_G[i] = rtb_nextG[i] * 0.004F;
  }
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
