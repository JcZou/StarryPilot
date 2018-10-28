/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: INS.c
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

#include "INS.h"
#include "INS_private.h"

const State_Bus INS_rtZState_Bus = {
  {
    0.0F, 0.0F, 0.0F }
  ,                                    /* pos_O */

  {
    0.0F, 0.0F, 0.0F }
  ,                                    /* vel_O */

  {
    0.0F, 0.0F, 0.0F, 0.0F }
  ,                                    /* quat */

  {
    0.0F, 0.0F, 0.0F }
  ,                                    /* gyr_bias */
  0.0F                                 /* az_bias */
} ;                                    /* State_Bus ground */

/* Block signals (default storage) */
B_INS_T INS_B;

/* Block states (default storage) */
DW_INS_T INS_DW;

/* External inputs (root inport signals with default storage) */
ExtU_INS_T INS_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_INS_T INS_Y;

/* Real-time model */
RT_MODEL_INS_T INS_M_;
RT_MODEL_INS_T *const INS_M = &INS_M_;

/* Model step function */
void INS_step(void)
{
  real32_T dHdx[28];
  real32_T Pxy[28];
  real32_T Pyy[4];
  real32_T zEstimated[2];
  real32_T gain[28];
  int32_T r1;
  int32_T r2;
  real32_T a21;
  real32_T a22;
  real32_T dHdx_0[14];
  real32_T gain_0[14];
  real32_T dHdx_1[42];
  real32_T Pxy_0[42];
  real32_T Pyy_0[9];
  real32_T zEstimated_0[3];
  real32_T gain_1[42];
  int32_T r3;
  int32_T rtemp;
  real32_T varargout_2[140];
  real32_T tmp[10];
  real32_T dHdx_2[14];
  real32_T dHdx_3[28];
  real32_T dHdx_4[42];
  real32_T gain_2[196];
  real32_T varargout_1[196];
  real32_T varargout_2_0[140];
  real32_T varargout_1_0[196];
  real32_T varargout_2_1[196];
  real32_T tmp_0;
  real32_T a22_tmp;
  int32_T gain_tmp;
  int32_T gain_tmp_0;
  int32_T gain_tmp_1;
  int32_T gain_tmp_2;
  int32_T gain_tmp_3;
  int32_T gain_tmp_4;

  /* Outputs for Enabled SubSystem: '<S1>/Correct1' incorporates:
   *  EnablePort: '<S12>/Enable'
   */
  if (INS_ConstB.DataTypeConversion_Enable1) {
    /* MATLAB Function: '<S12>/Correct' incorporates:
     *  Constant: '<S1>/BlockOrdering'
     *  Constant: '<S1>/R1'
     *  DataStoreRead: '<S12>/Data Store ReadP'
     *  DataStoreRead: '<S12>/Data Store ReadX'
     */
    INS_B.blockOrdering_i = true;
    EKF_GpsJacobian(INS_DW.x, dHdx);
    for (gain_tmp_3 = 0; gain_tmp_3 < 2; gain_tmp_3++) {
      for (gain_tmp_0 = 0; gain_tmp_0 < 14; gain_tmp_0++) {
        gain[gain_tmp_0 + 14 * gain_tmp_3] = dHdx[(gain_tmp_0 << 1) + gain_tmp_3];
      }
    }

    for (gain_tmp_3 = 0; gain_tmp_3 < 2; gain_tmp_3++) {
      for (gain_tmp_0 = 0; gain_tmp_0 < 14; gain_tmp_0++) {
        r2 = gain_tmp_0 + 14 * gain_tmp_3;
        Pxy[r2] = 0.0F;
        rtemp = gain_tmp_0 << 1;
        r3 = gain_tmp_3 + rtemp;
        dHdx_3[r3] = 0.0F;
        for (r1 = 0; r1 < 14; r1++) {
          Pxy[r2] = INS_DW.P[14 * r1 + gain_tmp_0] * gain[14 * gain_tmp_3 + r1]
            + Pxy[14 * gain_tmp_3 + gain_tmp_0];
          dHdx_3[r3] = dHdx[(r1 << 1) + gain_tmp_3] * INS_DW.P[14 * gain_tmp_0 +
            r1] + dHdx_3[rtemp + gain_tmp_3];
        }
      }

      for (gain_tmp_0 = 0; gain_tmp_0 < 2; gain_tmp_0++) {
        tmp_0 = 0.0F;
        for (r1 = 0; r1 < 14; r1++) {
          tmp_0 += dHdx_3[(r1 << 1) + gain_tmp_3] * gain[14 * gain_tmp_0 + r1];
        }

        r1 = gain_tmp_0 << 1;
        Pyy[gain_tmp_3 + r1] = rtCP_R1_Value[r1 + gain_tmp_3] + tmp_0;
      }
    }

    EKF_GpsCorrect(INS_DW.x, zEstimated);
    if (fabsf(Pyy[1]) > fabsf(Pyy[0])) {
      r1 = 1;
      r2 = 0;
    } else {
      r1 = 0;
      r2 = 1;
    }

    a21 = Pyy[r2] / Pyy[r1];
    a22_tmp = Pyy[2 + r1];
    a22 = Pyy[2 + r2] - a22_tmp * a21;
    for (rtemp = 0; rtemp < 14; rtemp++) {
      gain_tmp_0 = rtemp + 14 * r1;
      gain[gain_tmp_0] = Pxy[rtemp] / Pyy[r1];
      gain_tmp = 14 * r1 + rtemp;
      gain[rtemp + 14 * r2] = (Pxy[14 + rtemp] - gain[gain_tmp] * a22_tmp) / a22;
      gain[gain_tmp_0] = gain[gain_tmp] - gain[14 * r2 + rtemp] * a21;
      for (gain_tmp_3 = 0; gain_tmp_3 < 14; gain_tmp_3++) {
        gain_tmp_0 = rtemp + 14 * gain_tmp_3;
        gain_2[gain_tmp_0] = 0.0F;
        gain_tmp = gain_tmp_3 << 1;
        gain_tmp_1 = 14 * gain_tmp_3 + rtemp;
        gain_2[gain_tmp_0] = gain_2[gain_tmp_1] + dHdx[gain_tmp] * gain[rtemp];
        gain_2[gain_tmp_0] = dHdx[gain_tmp + 1] * gain[rtemp + 14] +
          gain_2[gain_tmp_1];
      }

      for (gain_tmp_3 = 0; gain_tmp_3 < 14; gain_tmp_3++) {
        tmp_0 = 0.0F;
        for (gain_tmp_0 = 0; gain_tmp_0 < 14; gain_tmp_0++) {
          tmp_0 += gain_2[14 * gain_tmp_0 + rtemp] * INS_DW.P[14 * gain_tmp_3 +
            gain_tmp_0];
        }

        varargout_1[rtemp + 14 * gain_tmp_3] = INS_DW.P[14 * gain_tmp_3 + rtemp]
          - tmp_0;
      }
    }

    /* DataStoreWrite: '<S12>/Data Store WriteP' */
    memcpy(&INS_DW.P[0], &varargout_1[0], 196U * sizeof(real32_T));

    /* MATLAB Function: '<S12>/Correct' incorporates:
     *  Inport: '<Root>/GPS'
     */
    tmp_0 = INS_U.GPS.xy[0] - zEstimated[0];
    a21 = INS_U.GPS.xy[1] - zEstimated[1];

    /* DataStoreWrite: '<S12>/Data Store WriteX' incorporates:
     *  DataStoreRead: '<S12>/Data Store ReadX'
     *  MATLAB Function: '<S12>/Correct'
     */
    for (gain_tmp_3 = 0; gain_tmp_3 < 14; gain_tmp_3++) {
      INS_DW.x[gain_tmp_3] += gain[gain_tmp_3 + 14] * a21 + gain[gain_tmp_3] *
        tmp_0;
    }

    /* End of DataStoreWrite: '<S12>/Data Store WriteX' */
  }

  /* End of Outputs for SubSystem: '<S1>/Correct1' */

  /* Outputs for Enabled SubSystem: '<S1>/Correct2' incorporates:
   *  EnablePort: '<S13>/Enable'
   */
  if (INS_ConstB.DataTypeConversion_Enable2) {
    /* MATLAB Function: '<S13>/Correct' incorporates:
     *  Constant: '<S1>/R2'
     *  DataStoreRead: '<S13>/Data Store ReadP'
     *  DataStoreRead: '<S13>/Data Store ReadX'
     *  Inport: '<Root>/BARO'
     */
    INS_B.blockOrdering_e = INS_B.blockOrdering_i;
    EKF_BaroJacobian(INS_DW.x, dHdx_0);
    a22_tmp = 0.0F;
    for (gain_tmp_3 = 0; gain_tmp_3 < 14; gain_tmp_3++) {
      dHdx_2[gain_tmp_3] = 0.0F;
      for (gain_tmp_0 = 0; gain_tmp_0 < 14; gain_tmp_0++) {
        dHdx_2[gain_tmp_3] += INS_DW.P[14 * gain_tmp_3 + gain_tmp_0] *
          dHdx_0[gain_tmp_0];
      }

      a22_tmp += dHdx_2[gain_tmp_3] * dHdx_0[gain_tmp_3];
    }

    EKF_BaroCorrect(INS_DW.x, &a22);
    a21 = INS_U.BARO.z - a22;
    for (gain_tmp_3 = 0; gain_tmp_3 < 14; gain_tmp_3++) {
      tmp_0 = 0.0F;
      for (gain_tmp_0 = 0; gain_tmp_0 < 14; gain_tmp_0++) {
        tmp_0 += INS_DW.P[14 * gain_tmp_0 + gain_tmp_3] * dHdx_0[gain_tmp_0];
      }

      gain_0[gain_tmp_3] = tmp_0 / (a22_tmp + 0.0001F);
      for (gain_tmp_0 = 0; gain_tmp_0 < 14; gain_tmp_0++) {
        gain_2[gain_tmp_3 + 14 * gain_tmp_0] = gain_0[gain_tmp_3] *
          dHdx_0[gain_tmp_0];
      }

      for (gain_tmp_0 = 0; gain_tmp_0 < 14; gain_tmp_0++) {
        tmp_0 = 0.0F;
        for (r1 = 0; r1 < 14; r1++) {
          tmp_0 += gain_2[14 * r1 + gain_tmp_3] * INS_DW.P[14 * gain_tmp_0 + r1];
        }

        varargout_1[gain_tmp_3 + 14 * gain_tmp_0] = INS_DW.P[14 * gain_tmp_0 +
          gain_tmp_3] - tmp_0;
      }
    }

    /* DataStoreWrite: '<S13>/Data Store WriteP' */
    memcpy(&INS_DW.P[0], &varargout_1[0], 196U * sizeof(real32_T));

    /* DataStoreWrite: '<S13>/Data Store WriteX' incorporates:
     *  DataStoreRead: '<S13>/Data Store ReadX'
     *  MATLAB Function: '<S13>/Correct'
     */
    for (gain_tmp_3 = 0; gain_tmp_3 < 14; gain_tmp_3++) {
      INS_DW.x[gain_tmp_3] += gain_0[gain_tmp_3] * a21;
    }

    /* End of DataStoreWrite: '<S13>/Data Store WriteX' */
  }

  /* End of Outputs for SubSystem: '<S1>/Correct2' */

  /* Outputs for Enabled SubSystem: '<S1>/Correct3' incorporates:
   *  EnablePort: '<S14>/Enable'
   */
  if (INS_ConstB.DataTypeConversion_Enable3) {
    /* MATLAB Function: '<S14>/Correct' incorporates:
     *  Constant: '<S1>/R3'
     *  DataStoreRead: '<S14>/Data Store ReadP'
     *  DataStoreRead: '<S14>/Data Store ReadX'
     */
    EKF_ImuJacobian(INS_DW.x, dHdx_1);
    for (gain_tmp_3 = 0; gain_tmp_3 < 3; gain_tmp_3++) {
      for (gain_tmp_0 = 0; gain_tmp_0 < 14; gain_tmp_0++) {
        gain_1[gain_tmp_0 + 14 * gain_tmp_3] = dHdx_1[3 * gain_tmp_0 +
          gain_tmp_3];
      }
    }

    for (gain_tmp_3 = 0; gain_tmp_3 < 3; gain_tmp_3++) {
      for (gain_tmp_0 = 0; gain_tmp_0 < 14; gain_tmp_0++) {
        r2 = gain_tmp_0 + 14 * gain_tmp_3;
        Pxy_0[r2] = 0.0F;
        rtemp = gain_tmp_3 + 3 * gain_tmp_0;
        dHdx_4[rtemp] = 0.0F;
        for (r1 = 0; r1 < 14; r1++) {
          Pxy_0[r2] = INS_DW.P[14 * r1 + gain_tmp_0] * gain_1[14 * gain_tmp_3 +
            r1] + Pxy_0[14 * gain_tmp_3 + gain_tmp_0];
          dHdx_4[rtemp] = dHdx_1[3 * r1 + gain_tmp_3] * INS_DW.P[14 * gain_tmp_0
            + r1] + dHdx_4[3 * gain_tmp_0 + gain_tmp_3];
        }
      }

      for (gain_tmp_0 = 0; gain_tmp_0 < 3; gain_tmp_0++) {
        tmp_0 = 0.0F;
        for (r1 = 0; r1 < 14; r1++) {
          tmp_0 += dHdx_4[3 * r1 + gain_tmp_3] * gain_1[14 * gain_tmp_0 + r1];
        }

        Pyy_0[gain_tmp_3 + 3 * gain_tmp_0] = rtCP_R3_Value[3 * gain_tmp_0 +
          gain_tmp_3] + tmp_0;
      }
    }

    EKF_ImuCorrect(INS_DW.x, zEstimated_0);
    r1 = 0;
    r2 = 1;
    r3 = 2;
    a22 = fabsf(Pyy_0[0]);
    a21 = fabsf(Pyy_0[1]);
    if (a21 > a22) {
      a22 = a21;
      r1 = 1;
      r2 = 0;
    }

    if (fabsf(Pyy_0[2]) > a22) {
      r1 = 2;
      r2 = 1;
      r3 = 0;
    }

    Pyy_0[r2] /= Pyy_0[r1];
    Pyy_0[r3] /= Pyy_0[r1];
    Pyy_0[3 + r2] -= Pyy_0[3 + r1] * Pyy_0[r2];
    Pyy_0[3 + r3] -= Pyy_0[3 + r1] * Pyy_0[r3];
    Pyy_0[6 + r2] -= Pyy_0[6 + r1] * Pyy_0[r2];
    Pyy_0[6 + r3] -= Pyy_0[6 + r1] * Pyy_0[r3];
    if (fabsf(Pyy_0[3 + r3]) > fabsf(Pyy_0[3 + r2])) {
      rtemp = r2;
      r2 = r3;
      r3 = rtemp;
    }

    Pyy_0[3 + r3] /= Pyy_0[3 + r2];
    Pyy_0[6 + r3] -= Pyy_0[3 + r3] * Pyy_0[6 + r2];
    for (rtemp = 0; rtemp < 14; rtemp++) {
      gain_tmp_0 = rtemp + 14 * r1;
      gain_1[gain_tmp_0] = Pxy_0[rtemp] / Pyy_0[r1];
      gain_tmp = 14 * r1 + rtemp;
      gain_tmp_1 = rtemp + 14 * r2;
      gain_1[gain_tmp_1] = Pxy_0[14 + rtemp] - gain_1[gain_tmp] * Pyy_0[3 + r1];
      gain_tmp_3 = rtemp + 14 * r3;
      gain_1[gain_tmp_3] = Pxy_0[28 + rtemp] - gain_1[gain_tmp] * Pyy_0[6 + r1];
      gain_tmp_2 = 14 * r2 + rtemp;
      gain_1[gain_tmp_1] = gain_1[gain_tmp_2] / Pyy_0[3 + r2];
      gain_tmp_4 = 14 * r3 + rtemp;
      gain_1[gain_tmp_3] = gain_1[gain_tmp_4] - gain_1[gain_tmp_2] * Pyy_0[6 +
        r2];
      gain_1[gain_tmp_3] = gain_1[gain_tmp_4] / Pyy_0[6 + r3];
      gain_1[gain_tmp_1] = gain_1[gain_tmp_2] - gain_1[gain_tmp_4] * Pyy_0[3 +
        r3];
      gain_1[gain_tmp_0] = gain_1[gain_tmp] - gain_1[gain_tmp_4] * Pyy_0[r3];
      gain_1[gain_tmp_0] = gain_1[gain_tmp] - gain_1[gain_tmp_2] * Pyy_0[r2];
      for (gain_tmp_3 = 0; gain_tmp_3 < 14; gain_tmp_3++) {
        gain_tmp_0 = rtemp + 14 * gain_tmp_3;
        gain_2[gain_tmp_0] = 0.0F;
        gain_tmp = 14 * gain_tmp_3 + rtemp;
        gain_2[gain_tmp_0] = gain_2[gain_tmp] + dHdx_1[3 * gain_tmp_3] *
          gain_1[rtemp];
        gain_2[gain_tmp_0] = dHdx_1[3 * gain_tmp_3 + 1] * gain_1[rtemp + 14] +
          gain_2[gain_tmp];
        gain_2[gain_tmp_0] = dHdx_1[3 * gain_tmp_3 + 2] * gain_1[rtemp + 28] +
          gain_2[gain_tmp];
      }

      for (gain_tmp_3 = 0; gain_tmp_3 < 14; gain_tmp_3++) {
        tmp_0 = 0.0F;
        for (gain_tmp_0 = 0; gain_tmp_0 < 14; gain_tmp_0++) {
          tmp_0 += gain_2[14 * gain_tmp_0 + rtemp] * INS_DW.P[14 * gain_tmp_3 +
            gain_tmp_0];
        }

        varargout_1[rtemp + 14 * gain_tmp_3] = INS_DW.P[14 * gain_tmp_3 + rtemp]
          - tmp_0;
      }
    }

    /* DataStoreWrite: '<S14>/Data Store WriteP' */
    memcpy(&INS_DW.P[0], &varargout_1[0], 196U * sizeof(real32_T));

    /* DataStoreWrite: '<S14>/Data Store WriteX' incorporates:
     *  DataStoreRead: '<S14>/Data Store ReadX'
     *  MATLAB Function: '<S14>/Correct'
     */
    for (gain_tmp_3 = 0; gain_tmp_3 < 14; gain_tmp_3++) {
      INS_DW.x[gain_tmp_3] += ((0.0F - zEstimated_0[0]) * gain_1[gain_tmp_3] +
        gain_1[gain_tmp_3 + 14] * (0.0F - zEstimated_0[1])) + gain_1[gain_tmp_3
        + 28] * (-1.0F - zEstimated_0[2]);
    }

    /* End of DataStoreWrite: '<S14>/Data Store WriteX' */
  }

  /* End of Outputs for SubSystem: '<S1>/Correct3' */

  /* Outputs for Enabled SubSystem: '<S1>/Correct4' incorporates:
   *  EnablePort: '<S15>/Enable'
   */
  if (INS_ConstB.DataTypeConversion_Enable4) {
    /* MATLAB Function: '<S15>/Correct' incorporates:
     *  Constant: '<S1>/R4'
     *  DataStoreRead: '<S15>/Data Store ReadP'
     *  DataStoreRead: '<S15>/Data Store ReadX'
     */
    EKF_MagJacobian(INS_DW.x, dHdx);
    for (gain_tmp_3 = 0; gain_tmp_3 < 2; gain_tmp_3++) {
      for (gain_tmp_0 = 0; gain_tmp_0 < 14; gain_tmp_0++) {
        gain[gain_tmp_0 + 14 * gain_tmp_3] = dHdx[(gain_tmp_0 << 1) + gain_tmp_3];
      }
    }

    for (gain_tmp_3 = 0; gain_tmp_3 < 2; gain_tmp_3++) {
      for (gain_tmp_0 = 0; gain_tmp_0 < 14; gain_tmp_0++) {
        r2 = gain_tmp_0 + 14 * gain_tmp_3;
        Pxy[r2] = 0.0F;
        rtemp = gain_tmp_0 << 1;
        r3 = gain_tmp_3 + rtemp;
        dHdx_3[r3] = 0.0F;
        for (r1 = 0; r1 < 14; r1++) {
          Pxy[r2] = INS_DW.P[14 * r1 + gain_tmp_0] * gain[14 * gain_tmp_3 + r1]
            + Pxy[14 * gain_tmp_3 + gain_tmp_0];
          dHdx_3[r3] = dHdx[(r1 << 1) + gain_tmp_3] * INS_DW.P[14 * gain_tmp_0 +
            r1] + dHdx_3[rtemp + gain_tmp_3];
        }
      }

      for (gain_tmp_0 = 0; gain_tmp_0 < 2; gain_tmp_0++) {
        tmp_0 = 0.0F;
        for (r1 = 0; r1 < 14; r1++) {
          tmp_0 += dHdx_3[(r1 << 1) + gain_tmp_3] * gain[14 * gain_tmp_0 + r1];
        }

        r1 = gain_tmp_0 << 1;
        Pyy[gain_tmp_3 + r1] = rtCP_R4_Value[r1 + gain_tmp_3] + tmp_0;
      }
    }

    EKF_MagCorrect(INS_DW.x, zEstimated);
    if (fabsf(Pyy[1]) > fabsf(Pyy[0])) {
      r1 = 1;
      r2 = 0;
    } else {
      r1 = 0;
      r2 = 1;
    }

    a21 = Pyy[r2] / Pyy[r1];
    a22_tmp = Pyy[2 + r1];
    a22 = Pyy[2 + r2] - a22_tmp * a21;
    for (rtemp = 0; rtemp < 14; rtemp++) {
      gain_tmp_0 = rtemp + 14 * r1;
      gain[gain_tmp_0] = Pxy[rtemp] / Pyy[r1];
      gain_tmp = 14 * r1 + rtemp;
      gain[rtemp + 14 * r2] = (Pxy[14 + rtemp] - gain[gain_tmp] * a22_tmp) / a22;
      gain[gain_tmp_0] = gain[gain_tmp] - gain[14 * r2 + rtemp] * a21;
      for (gain_tmp_3 = 0; gain_tmp_3 < 14; gain_tmp_3++) {
        gain_tmp_0 = rtemp + 14 * gain_tmp_3;
        gain_2[gain_tmp_0] = 0.0F;
        gain_tmp = gain_tmp_3 << 1;
        gain_tmp_1 = 14 * gain_tmp_3 + rtemp;
        gain_2[gain_tmp_0] = gain_2[gain_tmp_1] + dHdx[gain_tmp] * gain[rtemp];
        gain_2[gain_tmp_0] = dHdx[gain_tmp + 1] * gain[rtemp + 14] +
          gain_2[gain_tmp_1];
      }

      for (gain_tmp_3 = 0; gain_tmp_3 < 14; gain_tmp_3++) {
        tmp_0 = 0.0F;
        for (gain_tmp_0 = 0; gain_tmp_0 < 14; gain_tmp_0++) {
          tmp_0 += gain_2[14 * gain_tmp_0 + rtemp] * INS_DW.P[14 * gain_tmp_3 +
            gain_tmp_0];
        }

        varargout_1[rtemp + 14 * gain_tmp_3] = INS_DW.P[14 * gain_tmp_3 + rtemp]
          - tmp_0;
      }
    }

    /* DataStoreWrite: '<S15>/Data Store WriteP' */
    memcpy(&INS_DW.P[0], &varargout_1[0], 196U * sizeof(real32_T));

    /* DataStoreWrite: '<S15>/Data Store WriteX' incorporates:
     *  DataStoreRead: '<S15>/Data Store ReadX'
     *  MATLAB Function: '<S15>/Correct'
     */
    for (gain_tmp_3 = 0; gain_tmp_3 < 14; gain_tmp_3++) {
      INS_DW.x[gain_tmp_3] += (1.0F - zEstimated[0]) * gain[gain_tmp_3] +
        gain[gain_tmp_3 + 14] * (0.0F - zEstimated[1]);
    }

    /* End of DataStoreWrite: '<S15>/Data Store WriteX' */
  }

  /* End of Outputs for SubSystem: '<S1>/Correct4' */

  /* Outport: '<Root>/States' incorporates:
   *  BusCreator: '<Root>/BusConversion_InsertedFor_States_at_inport_0'
   *  DataStoreRead: '<S16>/Data Store Read'
   */
  INS_Y.States.pos_O[0] = INS_DW.x[0];
  INS_Y.States.vel_O[0] = INS_DW.x[3];
  INS_Y.States.pos_O[1] = INS_DW.x[1];
  INS_Y.States.vel_O[1] = INS_DW.x[4];
  INS_Y.States.pos_O[2] = INS_DW.x[2];
  INS_Y.States.vel_O[2] = INS_DW.x[5];
  INS_Y.States.quat[0] = INS_DW.x[6];
  INS_Y.States.quat[1] = INS_DW.x[7];
  INS_Y.States.quat[2] = INS_DW.x[8];
  INS_Y.States.quat[3] = INS_DW.x[9];
  INS_Y.States.gyr_bias[0] = INS_DW.x[10];
  INS_Y.States.gyr_bias[1] = INS_DW.x[11];
  INS_Y.States.gyr_bias[2] = INS_DW.x[12];
  INS_Y.States.az_bias = INS_DW.x[13];

  /* Outputs for Atomic SubSystem: '<S1>/Predict' */
  /* MATLAB Function: '<S17>/Predict' incorporates:
   *  Constant: '<S1>/Q'
   *  DataStoreRead: '<S17>/Data Store ReadP'
   *  DataStoreRead: '<S17>/Data Store ReadX'
   */
  for (gain_tmp_3 = 0; gain_tmp_3 < 14; gain_tmp_3++) {
    dHdx_0[gain_tmp_3] = INS_DW.x[gain_tmp_3];
  }

  for (gain_tmp_3 = 0; gain_tmp_3 < 10; gain_tmp_3++) {
    tmp[gain_tmp_3] = 0.0F;
  }

  EKF_PredictJacobian(INS_DW.x, tmp, gain_2, varargout_2);
  for (gain_tmp_3 = 0; gain_tmp_3 < 10; gain_tmp_3++) {
    tmp[gain_tmp_3] = 0.0F;
  }

  EKF_Predict(INS_DW.x, tmp, dHdx_0);
  for (gain_tmp_3 = 0; gain_tmp_3 < 14; gain_tmp_3++) {
    for (gain_tmp_0 = 0; gain_tmp_0 < 14; gain_tmp_0++) {
      r2 = gain_tmp_3 + 14 * gain_tmp_0;
      varargout_1[r2] = 0.0F;
      for (r1 = 0; r1 < 14; r1++) {
        varargout_1[r2] = gain_2[14 * r1 + gain_tmp_3] * INS_DW.P[14 *
          gain_tmp_0 + r1] + varargout_1[14 * gain_tmp_0 + gain_tmp_3];
      }
    }

    for (gain_tmp_0 = 0; gain_tmp_0 < 10; gain_tmp_0++) {
      r2 = gain_tmp_3 + 14 * gain_tmp_0;
      varargout_2_0[r2] = 0.0F;
      for (r1 = 0; r1 < 10; r1++) {
        varargout_2_0[r2] = varargout_2[14 * r1 + gain_tmp_3] * rtCP_Q_Value[10 *
          gain_tmp_0 + r1] + varargout_2_0[14 * gain_tmp_0 + gain_tmp_3];
      }
    }

    for (gain_tmp_0 = 0; gain_tmp_0 < 14; gain_tmp_0++) {
      r2 = gain_tmp_3 + 14 * gain_tmp_0;
      varargout_1_0[r2] = 0.0F;
      for (r1 = 0; r1 < 14; r1++) {
        varargout_1_0[r2] = varargout_1[14 * r1 + gain_tmp_3] * gain_2[14 * r1 +
          gain_tmp_0] + varargout_1_0[14 * gain_tmp_0 + gain_tmp_3];
      }

      varargout_2_1[r2] = 0.0F;
      for (r1 = 0; r1 < 10; r1++) {
        varargout_2_1[r2] = varargout_2_0[14 * r1 + gain_tmp_3] * varargout_2[14
          * r1 + gain_tmp_0] + varargout_2_1[14 * gain_tmp_0 + gain_tmp_3];
      }
    }
  }

  /* DataStoreWrite: '<S17>/Data Store WriteP' incorporates:
   *  MATLAB Function: '<S17>/Predict'
   */
  for (gain_tmp_3 = 0; gain_tmp_3 < 196; gain_tmp_3++) {
    INS_DW.P[gain_tmp_3] = varargout_1_0[gain_tmp_3] + varargout_2_1[gain_tmp_3];
  }

  /* End of DataStoreWrite: '<S17>/Data Store WriteP' */

  /* DataStoreWrite: '<S17>/Data Store WriteX' incorporates:
   *  MATLAB Function: '<S17>/Predict'
   */
  for (gain_tmp_3 = 0; gain_tmp_3 < 14; gain_tmp_3++) {
    INS_DW.x[gain_tmp_3] = dHdx_0[gain_tmp_3];
  }

  /* End of DataStoreWrite: '<S17>/Data Store WriteX' */
  /* End of Outputs for SubSystem: '<S1>/Predict' */
}

/* Model initialize function */
void INS_initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* initialize error status */
  rtmSetErrorStatus(INS_M, (NULL));

  /* block I/O */
  (void) memset(((void *) &INS_B), 0,
                sizeof(B_INS_T));

  /* states (dwork) */
  (void) memset((void *)&INS_DW, 0,
                sizeof(DW_INS_T));

  /* external inputs */
  (void)memset(&INS_U, 0, sizeof(ExtU_INS_T));

  /* external outputs */
  INS_Y.States = INS_rtZState_Bus;

  {
    int32_T i;

    /* Start for DataStoreMemory: '<S1>/DataStoreMemory - P' */
    memcpy(&INS_DW.P[0], &rtCP_DataStoreMemoryP_InitialVa[0], 196U * sizeof
           (real32_T));

    /* Start for DataStoreMemory: '<S1>/DataStoreMemory - x' */
    for (i = 0; i < 14; i++) {
      INS_DW.x[i] = rtCP_DataStoreMemoryx_InitialVa[i];
    }

    /* End of Start for DataStoreMemory: '<S1>/DataStoreMemory - x' */
  }
}

/* Model terminate function */
void INS_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
