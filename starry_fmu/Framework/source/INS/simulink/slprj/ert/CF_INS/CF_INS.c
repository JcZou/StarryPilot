/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: CF_INS.c
 *
 * Code generated for Simulink model 'CF_INS'.
 *
 * Model version                  : 1.440
 * Simulink Coder version         : 9.0 (R2018b) 24-May-2018
 * C/C++ source code generated on : Tue Dec 25 18:22:29 2018
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "CF_INS.h"
#include "CF_INS_private.h"
#include "rt_atan2f_snf.h"

MdlrefDW_CF_INS_T CF_INS_MdlrefDW;

/* Block states (default storage) */
DW_CF_INS_f_T CF_INS_DW;

/* System initialize for referenced model: 'CF_INS' */
void CF_INS_Init(void)
{
  /* InitializeConditions for DiscreteIntegrator: '<S5>/Discrete-Time Integrator' */
  CF_INS_DW.DiscreteTimeIntegrator_IC_LOADI = 1U;
  CF_INS_DW.DiscreteTimeIntegrator_PrevRese = 0;

  /* InitializeConditions for DiscreteIntegrator: '<S37>/Discrete-Time Integrator1' */
  CF_INS_DW.DiscreteTimeIntegrator1_DSTATE[0] = CF_INS_ConstB.Constant7[0];
  CF_INS_DW.DiscreteTimeIntegrator1_DSTATE[1] = CF_INS_ConstB.Constant7[1];
  CF_INS_DW.DiscreteTimeIntegrator1_DSTATE[2] = CF_INS_ConstB.Constant7[2];
  CF_INS_DW.DiscreteTimeIntegrator1_PrevRes = 0;

  /* InitializeConditions for DiscreteIntegrator: '<S59>/Discrete-Time Integrator' */
  CF_INS_DW.DiscreteTimeIntegrator_DSTATE_i = CF_INS_ConstB.Constant;

  /* InitializeConditions for DiscreteIntegrator: '<S67>/Discrete-Time Integrator1' */
  CF_INS_DW.DiscreteTimeIntegrator1_IC_LOAD = 1U;
}

/* Output and update for referenced model: 'CF_INS' */
void CF_INS(const real32_T rtu_Sensor_Data_IMU_Data_rot_ra[3], const real32_T
            rtu_Sensor_Data_IMU_Data_sfor_m[3], const real32_T
            rtu_Sensor_Data_Mag_Data_mag_ga[3], const real32_T
            *rtu_Sensor_Data_GPS_Data_pos_qu, const real32_T
            *rtu_Sensor_Data_GPS_Data_vel_qu, const real32_T
            *rtu_Sensor_Data_GPS_Data_height, const real32_T
            *rtu_Sensor_Data_GPS_Data_velD_m, const boolean_T
            *rtu_Sensor_Data_Baro_Data_valid, const real32_T
            *rtu_Sensor_Data_Baro_Data_relat, INS_Out_Bus *rty_INS_Out)
{
  int_T idxDelay;
  real32_T rtb_Max;
  real32_T rtb_Max1;
  real32_T rtb_Product3_cy;
  real32_T rtb_R_BO[9];
  real32_T rtb_Sum_i;
  real32_T rtb_Sum_e;
  real32_T rtb_Multiply2_o;
  real32_T rtb_Multiply1_e;
  real32_T rtb_DiscreteTimeIntegrator_n;
  real32_T rtb_Subtract1_j;
  real32_T rtb_DiscreteTimeIntegrator1_b;
  real32_T rtb_DiscreteTimeIntegrator1_i;
  real32_T rtb_ki;
  real32_T rtb_vz_dot;
  real32_T rtb_Add_p;
  real32_T rtb_q1;
  real32_T rtb_MathFunction_k[9];
  real32_T rtb_MathFunction_p_idx_1;
  real32_T rtb_DiscreteTimeIntegrator1_idx;
  real32_T rtb_DiscreteTimeIntegrator1_i_0;
  real32_T rtb_Switch_a_idx_3;
  real32_T rtb_rot_est_B_idx_0;
  real32_T rtb_rot_est_B_idx_1;
  real32_T rtb_rot_est_B_idx_2;
  real32_T rtb_MathFunction_jg_idx_1;
  real32_T rtb_Switch_a_idx_2;

  /* Sum: '<S50>/Sum of Elements' incorporates:
   *  Math: '<S50>/Math Function'
   *  Sum: '<S12>/Sum of Elements'
   */
  rtb_Product3_cy = (rtu_Sensor_Data_IMU_Data_sfor_m[0] *
                     rtu_Sensor_Data_IMU_Data_sfor_m[0] +
                     rtu_Sensor_Data_IMU_Data_sfor_m[1] *
                     rtu_Sensor_Data_IMU_Data_sfor_m[1]) +
    rtu_Sensor_Data_IMU_Data_sfor_m[2] * rtu_Sensor_Data_IMU_Data_sfor_m[2];

  /* MinMax: '<S6>/Max' incorporates:
   *  Constant: '<S6>/Constant2'
   *  Sqrt: '<S50>/Sqrt'
   *  Sum: '<S50>/Sum of Elements'
   */
  rtb_Max = fmaxf(sqrtf(rtb_Product3_cy), 0.001F);

  /* Product: '<S6>/Divide' incorporates:
   *  Gain: '<S6>/Gain'
   */
  rtb_rot_est_B_idx_0 = -rtu_Sensor_Data_IMU_Data_sfor_m[0] / rtb_Max;
  rtb_rot_est_B_idx_1 = -rtu_Sensor_Data_IMU_Data_sfor_m[1] / rtb_Max;
  rtb_rot_est_B_idx_2 = -rtu_Sensor_Data_IMU_Data_sfor_m[2] / rtb_Max;

  /* SignalConversion: '<S49>/TmpSignal ConversionAtMath FunctionInport1' incorporates:
   *  Product: '<S47>/Product'
   *  Product: '<S47>/Product1'
   *  Product: '<S47>/Product2'
   *  Product: '<S47>/Product3'
   *  Product: '<S47>/Product4'
   *  Product: '<S47>/Product5'
   *  Sum: '<S47>/Subtract'
   *  Sum: '<S47>/Subtract1'
   *  Sum: '<S47>/Subtract2'
   */
  rtb_Max = rtb_rot_est_B_idx_1 * rtu_Sensor_Data_Mag_Data_mag_ga[2] -
    rtb_rot_est_B_idx_2 * rtu_Sensor_Data_Mag_Data_mag_ga[1];
  rtb_DiscreteTimeIntegrator1_idx = rtb_rot_est_B_idx_2 *
    rtu_Sensor_Data_Mag_Data_mag_ga[0] - rtb_rot_est_B_idx_0 *
    rtu_Sensor_Data_Mag_Data_mag_ga[2];
  rtb_DiscreteTimeIntegrator1_i_0 = rtb_rot_est_B_idx_0 *
    rtu_Sensor_Data_Mag_Data_mag_ga[1] - rtb_rot_est_B_idx_1 *
    rtu_Sensor_Data_Mag_Data_mag_ga[0];

  /* MinMax: '<S6>/Max1' incorporates:
   *  Constant: '<S6>/Constant1'
   *  Math: '<S49>/Math Function'
   *  Sqrt: '<S49>/Sqrt'
   *  Sum: '<S49>/Sum of Elements'
   */
  rtb_Max1 = fmaxf(sqrtf((rtb_Max * rtb_Max + rtb_DiscreteTimeIntegrator1_idx *
    rtb_DiscreteTimeIntegrator1_idx) + rtb_DiscreteTimeIntegrator1_i_0 *
    rtb_DiscreteTimeIntegrator1_i_0), 0.001F);

  /* Product: '<S6>/Divide1' */
  rtb_q1 = rtb_Max / rtb_Max1;

  /* SignalConversion: '<S6>/ConcatBufferAtVector ConcatenateIn2' */
  rtb_R_BO[3] = rtb_q1;

  /* SignalConversion: '<S6>/ConcatBufferAtVector ConcatenateIn3' */
  rtb_R_BO[6] = rtb_rot_est_B_idx_0;

  /* Product: '<S6>/Divide1' */
  rtb_Max = rtb_q1;
  rtb_q1 = rtb_DiscreteTimeIntegrator1_idx / rtb_Max1;

  /* SignalConversion: '<S6>/ConcatBufferAtVector ConcatenateIn2' */
  rtb_R_BO[4] = rtb_q1;

  /* SignalConversion: '<S6>/ConcatBufferAtVector ConcatenateIn3' */
  rtb_R_BO[7] = rtb_rot_est_B_idx_1;

  /* Product: '<S6>/Divide1' */
  rtb_DiscreteTimeIntegrator1_idx = rtb_q1;
  rtb_q1 = rtb_DiscreteTimeIntegrator1_i_0 / rtb_Max1;

  /* SignalConversion: '<S6>/ConcatBufferAtVector ConcatenateIn2' */
  rtb_R_BO[5] = rtb_q1;

  /* SignalConversion: '<S6>/ConcatBufferAtVector ConcatenateIn3' */
  rtb_R_BO[8] = rtb_rot_est_B_idx_2;

  /* Reshape: '<S48>/Column Vector' incorporates:
   *  Product: '<S48>/Product'
   *  Product: '<S48>/Product1'
   *  Product: '<S48>/Product2'
   *  Product: '<S48>/Product3'
   *  Product: '<S48>/Product4'
   *  Product: '<S48>/Product5'
   *  Sum: '<S48>/Subtract'
   *  Sum: '<S48>/Subtract1'
   *  Sum: '<S48>/Subtract2'
   */
  rtb_R_BO[0] = rtb_DiscreteTimeIntegrator1_idx * rtb_rot_est_B_idx_2 - rtb_q1 *
    rtb_rot_est_B_idx_1;
  rtb_R_BO[1] = rtb_q1 * rtb_rot_est_B_idx_0 - rtb_Max * rtb_rot_est_B_idx_2;
  rtb_R_BO[2] = rtb_Max * rtb_rot_est_B_idx_1 - rtb_DiscreteTimeIntegrator1_idx *
    rtb_rot_est_B_idx_0;

  /* Math: '<S6>/Math Function' */
  for (idxDelay = 0; idxDelay < 3; idxDelay++) {
    rtb_MathFunction_k[3 * idxDelay] = rtb_R_BO[idxDelay];
    rtb_MathFunction_k[1 + 3 * idxDelay] = rtb_R_BO[idxDelay + 3];
    rtb_MathFunction_k[2 + 3 * idxDelay] = rtb_R_BO[idxDelay + 6];
  }

  /* End of Math: '<S6>/Math Function' */

  /* If: '<S51>/If' incorporates:
   *  Selector: '<S51>/Selector'
   *  Selector: '<S51>/Selector1'
   *  Selector: '<S51>/Selector2'
   *  Sum: '<S51>/Sum'
   *  Sum: '<S51>/Sum1'
   *  Sum: '<S51>/Sum2'
   */
  if ((rtb_MathFunction_k[0] + rtb_MathFunction_k[4]) + rtb_MathFunction_k[8] >=
      0.0F) {
    /* Outputs for IfAction SubSystem: '<S51>/If Action Subsystem' incorporates:
     *  ActionPort: '<S52>/Action Port'
     */
    /* Sqrt: '<S52>/Sqrt' incorporates:
     *  Constant: '<S52>/Constant'
     *  Constant: '<S52>/Protection'
     *  Gain: '<S52>/Gain'
     *  MinMax: '<S52>/MinMax'
     *  Selector: '<S52>/Selector'
     *  Selector: '<S52>/Selector1'
     *  Selector: '<S52>/Selector2'
     *  Sum: '<S52>/Add'
     */
    rtb_q1 = sqrtf(fmaxf(1.0E-6F, (((1.0F + rtb_MathFunction_k[0]) +
      rtb_MathFunction_k[4]) + rtb_MathFunction_k[8]) * 0.25F));

    /* Product: '<S52>/Divide' incorporates:
     *  Gain: '<S52>/Gain1'
     *  Selector: '<S52>/Selector3'
     *  Selector: '<S52>/Selector4'
     *  Sum: '<S52>/Add1'
     */
    rtb_Max = (rtb_MathFunction_k[1] - rtb_MathFunction_k[3]) * 0.25F / rtb_q1;

    /* Product: '<S52>/Divide1' incorporates:
     *  Gain: '<S52>/Gain2'
     *  Selector: '<S52>/Selector5'
     *  Selector: '<S52>/Selector6'
     *  Sum: '<S52>/Add2'
     */
    rtb_Max1 = (rtb_MathFunction_k[5] - rtb_MathFunction_k[7]) * 0.25F / rtb_q1;

    /* Product: '<S52>/Divide2' incorporates:
     *  Gain: '<S52>/Gain3'
     *  Selector: '<S52>/Selector7'
     *  Selector: '<S52>/Selector8'
     *  Sum: '<S52>/Add3'
     */
    rtb_DiscreteTimeIntegrator1_idx = (rtb_MathFunction_k[6] -
      rtb_MathFunction_k[2]) * 0.25F / rtb_q1;

    /* End of Outputs for SubSystem: '<S51>/If Action Subsystem' */
  } else if ((rtb_MathFunction_k[0] - rtb_MathFunction_k[4]) -
             rtb_MathFunction_k[8] >= 0.0F) {
    /* Outputs for IfAction SubSystem: '<S51>/If Action Subsystem1' incorporates:
     *  ActionPort: '<S53>/Action Port'
     */
    /* Sqrt: '<S53>/Sqrt' incorporates:
     *  Constant: '<S53>/Constant'
     *  Constant: '<S53>/Protection'
     *  Gain: '<S53>/Gain'
     *  MinMax: '<S53>/MinMax'
     *  Selector: '<S53>/Selector'
     *  Selector: '<S53>/Selector1'
     *  Selector: '<S53>/Selector2'
     *  Sum: '<S53>/Add'
     */
    rtb_Max1 = sqrtf(fmaxf(1.0E-6F, (((1.0F + rtb_MathFunction_k[0]) -
      rtb_MathFunction_k[4]) - rtb_MathFunction_k[8]) * 0.25F));

    /* Product: '<S53>/Divide' incorporates:
     *  Gain: '<S53>/Gain1'
     *  Selector: '<S53>/Selector3'
     *  Selector: '<S53>/Selector4'
     *  Sum: '<S53>/Add1'
     */
    rtb_DiscreteTimeIntegrator1_idx = (rtb_MathFunction_k[3] +
      rtb_MathFunction_k[1]) * 0.25F / rtb_Max1;

    /* Product: '<S53>/Divide1' incorporates:
     *  Gain: '<S53>/Gain2'
     *  Selector: '<S53>/Selector5'
     *  Selector: '<S53>/Selector6'
     *  Sum: '<S53>/Add2'
     */
    rtb_q1 = (rtb_MathFunction_k[5] - rtb_MathFunction_k[7]) * 0.25F / rtb_Max1;

    /* Product: '<S53>/Divide2' incorporates:
     *  Gain: '<S53>/Gain3'
     *  Selector: '<S53>/Selector7'
     *  Selector: '<S53>/Selector8'
     *  Sum: '<S53>/Add3'
     */
    rtb_Max = (rtb_MathFunction_k[6] + rtb_MathFunction_k[2]) * 0.25F / rtb_Max1;

    /* End of Outputs for SubSystem: '<S51>/If Action Subsystem1' */
  } else if ((rtb_MathFunction_k[4] - rtb_MathFunction_k[0]) -
             rtb_MathFunction_k[8] >= 0.0F) {
    /* Outputs for IfAction SubSystem: '<S51>/If Action Subsystem2' incorporates:
     *  ActionPort: '<S54>/Action Port'
     */
    /* Sqrt: '<S54>/Sqrt1' incorporates:
     *  Constant: '<S54>/Constant'
     *  Constant: '<S54>/Protection'
     *  Gain: '<S54>/Gain4'
     *  MinMax: '<S54>/MinMax1'
     *  Selector: '<S54>/Selector'
     *  Selector: '<S54>/Selector1'
     *  Selector: '<S54>/Selector2'
     *  Sum: '<S54>/Add'
     */
    rtb_DiscreteTimeIntegrator1_idx = sqrtf(fmaxf(1.0E-6F, (((1.0F -
      rtb_MathFunction_k[0]) + rtb_MathFunction_k[4]) - rtb_MathFunction_k[8]) *
      0.25F));

    /* Product: '<S54>/Divide' incorporates:
     *  Gain: '<S54>/Gain1'
     *  Selector: '<S54>/Selector3'
     *  Selector: '<S54>/Selector4'
     *  Sum: '<S54>/Add1'
     */
    rtb_Max1 = (rtb_MathFunction_k[3] + rtb_MathFunction_k[1]) * 0.25F /
      rtb_DiscreteTimeIntegrator1_idx;

    /* Product: '<S54>/Divide1' incorporates:
     *  Gain: '<S54>/Gain2'
     *  Selector: '<S54>/Selector5'
     *  Selector: '<S54>/Selector6'
     *  Sum: '<S54>/Add2'
     */
    rtb_Max = (rtb_MathFunction_k[7] + rtb_MathFunction_k[5]) * 0.25F /
      rtb_DiscreteTimeIntegrator1_idx;

    /* Product: '<S54>/Divide2' incorporates:
     *  Gain: '<S54>/Gain3'
     *  Selector: '<S54>/Selector7'
     *  Selector: '<S54>/Selector8'
     *  Sum: '<S54>/Add3'
     */
    rtb_q1 = (rtb_MathFunction_k[6] - rtb_MathFunction_k[2]) * 0.25F /
      rtb_DiscreteTimeIntegrator1_idx;

    /* End of Outputs for SubSystem: '<S51>/If Action Subsystem2' */
  } else {
    /* Outputs for IfAction SubSystem: '<S51>/If Action Subsystem3' incorporates:
     *  ActionPort: '<S55>/Action Port'
     */
    /* Sqrt: '<S55>/Sqrt' incorporates:
     *  Constant: '<S55>/Constant'
     *  Constant: '<S55>/Protection'
     *  Gain: '<S55>/Gain'
     *  MinMax: '<S55>/MinMax'
     *  Selector: '<S55>/Selector'
     *  Selector: '<S55>/Selector1'
     *  Selector: '<S55>/Selector2'
     *  Sum: '<S55>/Add'
     */
    rtb_Max = sqrtf(fmaxf(1.0E-6F, (((1.0F - rtb_MathFunction_k[0]) -
      rtb_MathFunction_k[4]) + rtb_MathFunction_k[8]) * 0.25F));

    /* Product: '<S55>/Divide' incorporates:
     *  Gain: '<S55>/Gain1'
     *  Selector: '<S55>/Selector3'
     *  Selector: '<S55>/Selector4'
     *  Sum: '<S55>/Add1'
     */
    rtb_q1 = (rtb_MathFunction_k[1] - rtb_MathFunction_k[3]) * 0.25F / rtb_Max;

    /* Product: '<S55>/Divide1' incorporates:
     *  Gain: '<S55>/Gain2'
     *  Selector: '<S55>/Selector5'
     *  Selector: '<S55>/Selector6'
     *  Sum: '<S55>/Add2'
     */
    rtb_DiscreteTimeIntegrator1_idx = (rtb_MathFunction_k[7] +
      rtb_MathFunction_k[5]) * 0.25F / rtb_Max;

    /* Product: '<S55>/Divide2' incorporates:
     *  Gain: '<S55>/Gain3'
     *  Selector: '<S55>/Selector7'
     *  Selector: '<S55>/Selector8'
     *  Sum: '<S55>/Add3'
     */
    rtb_Max1 = (rtb_MathFunction_k[6] + rtb_MathFunction_k[2]) * 0.25F / rtb_Max;

    /* End of Outputs for SubSystem: '<S51>/If Action Subsystem3' */
  }

  /* End of If: '<S51>/If' */

  /* Math: '<S12>/Math Function1'
   *
   * About '<S12>/Math Function1':
   *  Operator: sqrt
   */
  if (rtb_Product3_cy < 0.0F) {
    rtb_Product3_cy = -sqrtf(fabsf(rtb_Product3_cy));
  } else {
    rtb_Product3_cy = sqrtf(rtb_Product3_cy);
  }

  /* End of Math: '<S12>/Math Function1' */

  /* Switch: '<S12>/Switch' incorporates:
   *  Constant: '<S12>/Constant'
   *  Product: '<S12>/Product'
   */
  if (rtb_Product3_cy > 0.0F) {
    rtb_DiscreteTimeIntegrator1_i_0 = rtu_Sensor_Data_IMU_Data_sfor_m[0];
    rtb_rot_est_B_idx_1 = rtu_Sensor_Data_IMU_Data_sfor_m[1];
    rtb_rot_est_B_idx_0 = rtu_Sensor_Data_IMU_Data_sfor_m[2];
    rtb_rot_est_B_idx_2 = rtb_Product3_cy;
  } else {
    rtb_DiscreteTimeIntegrator1_i_0 = rtu_Sensor_Data_IMU_Data_sfor_m[0] * 0.0F;
    rtb_rot_est_B_idx_1 = rtu_Sensor_Data_IMU_Data_sfor_m[1] * 0.0F;
    rtb_rot_est_B_idx_0 = rtu_Sensor_Data_IMU_Data_sfor_m[2] * 0.0F;
    rtb_rot_est_B_idx_2 = 1.0F;
  }

  /* End of Switch: '<S12>/Switch' */

  /* Sum: '<S13>/Sum of Elements' incorporates:
   *  Math: '<S13>/Math Function'
   */
  rtb_Subtract1_j = (rtu_Sensor_Data_Mag_Data_mag_ga[0] *
                     rtu_Sensor_Data_Mag_Data_mag_ga[0] +
                     rtu_Sensor_Data_Mag_Data_mag_ga[1] *
                     rtu_Sensor_Data_Mag_Data_mag_ga[1]) +
    rtu_Sensor_Data_Mag_Data_mag_ga[2] * rtu_Sensor_Data_Mag_Data_mag_ga[2];

  /* Math: '<S13>/Math Function1' incorporates:
   *  Sum: '<S13>/Sum of Elements'
   *
   * About '<S13>/Math Function1':
   *  Operator: sqrt
   */
  if (rtb_Subtract1_j < 0.0F) {
    rtb_Product3_cy = -sqrtf(fabsf(rtb_Subtract1_j));
  } else {
    rtb_Product3_cy = sqrtf(rtb_Subtract1_j);
  }

  /* End of Math: '<S13>/Math Function1' */

  /* Switch: '<S13>/Switch' incorporates:
   *  Constant: '<S13>/Constant'
   *  Product: '<S13>/Product'
   */
  if (rtb_Product3_cy > 0.0F) {
    rtb_Sum_i = rtu_Sensor_Data_Mag_Data_mag_ga[0];
    rtb_MathFunction_p_idx_1 = rtu_Sensor_Data_Mag_Data_mag_ga[1];
    rtb_Switch_a_idx_2 = rtu_Sensor_Data_Mag_Data_mag_ga[2];
    rtb_Switch_a_idx_3 = rtb_Product3_cy;
  } else {
    rtb_Sum_i = rtu_Sensor_Data_Mag_Data_mag_ga[0] * 0.0F;
    rtb_MathFunction_p_idx_1 = rtu_Sensor_Data_Mag_Data_mag_ga[1] * 0.0F;
    rtb_Switch_a_idx_2 = rtu_Sensor_Data_Mag_Data_mag_ga[2] * 0.0F;
    rtb_Switch_a_idx_3 = 1.0F;
  }

  /* End of Switch: '<S13>/Switch' */

  /* DiscreteIntegrator: '<S5>/Discrete-Time Integrator' */
  if (CF_INS_DW.DiscreteTimeIntegrator_IC_LOADI != 0) {
    CF_INS_DW.DiscreteTimeIntegrator_DSTATE[0] = rtb_q1;
    CF_INS_DW.DiscreteTimeIntegrator_DSTATE[1] = rtb_Max1;
    CF_INS_DW.DiscreteTimeIntegrator_DSTATE[2] = rtb_DiscreteTimeIntegrator1_idx;
    CF_INS_DW.DiscreteTimeIntegrator_DSTATE[3] = rtb_Max;
  }

  if (CF_INS_DW.DiscreteTimeIntegrator_PrevRese != 0) {
    CF_INS_DW.DiscreteTimeIntegrator_DSTATE[0] = rtb_q1;
    CF_INS_DW.DiscreteTimeIntegrator_DSTATE[1] = rtb_Max1;
    CF_INS_DW.DiscreteTimeIntegrator_DSTATE[2] = rtb_DiscreteTimeIntegrator1_idx;
    CF_INS_DW.DiscreteTimeIntegrator_DSTATE[3] = rtb_Max;
  }

  /* Sqrt: '<S45>/sqrt' incorporates:
   *  DiscreteIntegrator: '<S5>/Discrete-Time Integrator'
   *  Product: '<S46>/Product'
   *  Product: '<S46>/Product1'
   *  Product: '<S46>/Product2'
   *  Product: '<S46>/Product3'
   *  Sum: '<S46>/Sum'
   */
  rtb_Product3_cy = sqrtf(((CF_INS_DW.DiscreteTimeIntegrator_DSTATE[0] *
    CF_INS_DW.DiscreteTimeIntegrator_DSTATE[0] +
    CF_INS_DW.DiscreteTimeIntegrator_DSTATE[1] *
    CF_INS_DW.DiscreteTimeIntegrator_DSTATE[1]) +
    CF_INS_DW.DiscreteTimeIntegrator_DSTATE[2] *
    CF_INS_DW.DiscreteTimeIntegrator_DSTATE[2]) +
    CF_INS_DW.DiscreteTimeIntegrator_DSTATE[3] *
    CF_INS_DW.DiscreteTimeIntegrator_DSTATE[3]);

  /* Product: '<S39>/Product1' incorporates:
   *  DiscreteIntegrator: '<S5>/Discrete-Time Integrator'
   */
  rtb_Max = CF_INS_DW.DiscreteTimeIntegrator_DSTATE[1] / rtb_Product3_cy;

  /* Product: '<S39>/Product' incorporates:
   *  DiscreteIntegrator: '<S5>/Discrete-Time Integrator'
   */
  rtb_DiscreteTimeIntegrator1_idx = CF_INS_DW.DiscreteTimeIntegrator_DSTATE[0] /
    rtb_Product3_cy;

  /* Product: '<S39>/Product2' incorporates:
   *  DiscreteIntegrator: '<S5>/Discrete-Time Integrator'
   */
  rtb_Max1 = CF_INS_DW.DiscreteTimeIntegrator_DSTATE[2] / rtb_Product3_cy;

  /* Product: '<S39>/Product3' incorporates:
   *  DiscreteIntegrator: '<S5>/Discrete-Time Integrator'
   */
  rtb_Product3_cy = CF_INS_DW.DiscreteTimeIntegrator_DSTATE[3] / rtb_Product3_cy;

  /* Product: '<S13>/Divide1' */
  rtb_q1 = rtb_Sum_i / rtb_Switch_a_idx_3;
  rtb_MathFunction_jg_idx_1 = rtb_MathFunction_p_idx_1 / rtb_Switch_a_idx_3;
  rtb_Switch_a_idx_2 /= rtb_Switch_a_idx_3;

  /* Product: '<S26>/Product2' incorporates:
   *  Math: '<S40>/Square1'
   */
  rtb_vz_dot = rtb_Max1 * rtb_Max1;

  /* Product: '<S26>/Product1' incorporates:
   *  Math: '<S40>/Square'
   */
  rtb_ki = rtb_Max * rtb_Max;

  /* Product: '<S26>/Product3' incorporates:
   *  Math: '<S40>/Square2'
   */
  rtb_Add_p = rtb_Product3_cy * rtb_Product3_cy;

  /* Sqrt: '<S25>/sqrt' incorporates:
   *  Product: '<S26>/Product'
   *  Product: '<S26>/Product1'
   *  Product: '<S26>/Product2'
   *  Product: '<S26>/Product3'
   *  Sqrt: '<S19>/sqrt'
   *  Sqrt: '<S35>/sqrt'
   *  Sqrt: '<S79>/sqrt'
   *  Sum: '<S26>/Sum'
   */
  rtb_DiscreteTimeIntegrator1_i = sqrtf(((rtb_DiscreteTimeIntegrator1_idx *
    rtb_DiscreteTimeIntegrator1_idx + rtb_ki) + rtb_vz_dot) + rtb_Add_p);

  /* Product: '<S24>/Product2' incorporates:
   *  Sqrt: '<S25>/sqrt'
   */
  rtb_Sum_e = rtb_Max1 / rtb_DiscreteTimeIntegrator1_i;

  /* Product: '<S24>/Product3' incorporates:
   *  Sqrt: '<S25>/sqrt'
   */
  rtb_Multiply2_o = rtb_Product3_cy / rtb_DiscreteTimeIntegrator1_i;

  /* Product: '<S24>/Product1' incorporates:
   *  Sqrt: '<S25>/sqrt'
   */
  rtb_Multiply1_e = rtb_Max / rtb_DiscreteTimeIntegrator1_i;

  /* Product: '<S24>/Product' incorporates:
   *  Sqrt: '<S25>/sqrt'
   */
  rtb_Sum_i = rtb_DiscreteTimeIntegrator1_idx / rtb_DiscreteTimeIntegrator1_i;

  /* Product: '<S22>/Product7' incorporates:
   *  Product: '<S21>/Product7'
   */
  rtb_MathFunction_p_idx_1 = rtb_Multiply2_o * rtb_Multiply2_o;

  /* Product: '<S22>/Product' incorporates:
   *  Product: '<S21>/Product'
   */
  rtb_Switch_a_idx_3 = rtb_Multiply1_e * rtb_Sum_e;

  /* Product: '<S22>/Product1' incorporates:
   *  Product: '<S21>/Product1'
   */
  rtb_DiscreteTimeIntegrator1_b = rtb_Sum_i * rtb_Multiply2_o;

  /* Sum: '<S22>/Sum' incorporates:
   *  Constant: '<S22>/Constant'
   *  Gain: '<S22>/Gain'
   *  Gain: '<S22>/Gain1'
   *  Gain: '<S22>/Gain2'
   *  Product: '<S22>/Product'
   *  Product: '<S22>/Product1'
   *  Product: '<S22>/Product2'
   *  Product: '<S22>/Product3'
   *  Product: '<S22>/Product4'
   *  Product: '<S22>/Product5'
   *  Product: '<S22>/Product6'
   *  Product: '<S22>/Product7'
   *  Product: '<S22>/Product8'
   *  Sum: '<S22>/Sum1'
   *  Sum: '<S22>/Sum2'
   *  Sum: '<S22>/Sum3'
   */
  rtb_Subtract1_j = (((0.5F - rtb_Multiply1_e * rtb_Multiply1_e) -
                      rtb_MathFunction_p_idx_1) * 2.0F *
                     rtb_MathFunction_jg_idx_1 + (rtb_Switch_a_idx_3 +
    rtb_DiscreteTimeIntegrator1_b) * 2.0F * rtb_q1) + (rtb_Sum_e *
    rtb_Multiply2_o - rtb_Sum_i * rtb_Multiply1_e) * 2.0F * rtb_Switch_a_idx_2;

  /* SignalConversion: '<S4>/ConcatBufferAtMatrix ConcatenateIn1' incorporates:
   *  Constant: '<S21>/Constant'
   *  Gain: '<S21>/Gain'
   *  Gain: '<S21>/Gain1'
   *  Gain: '<S21>/Gain2'
   *  Product: '<S21>/Product2'
   *  Product: '<S21>/Product3'
   *  Product: '<S21>/Product4'
   *  Product: '<S21>/Product5'
   *  Product: '<S21>/Product6'
   *  Product: '<S21>/Product8'
   *  Sum: '<S21>/Sum'
   *  Sum: '<S21>/Sum1'
   *  Sum: '<S21>/Sum2'
   *  Sum: '<S21>/Sum3'
   */
  rtb_Sum_i = (((0.5F - rtb_Sum_e * rtb_Sum_e) - rtb_MathFunction_p_idx_1) *
               2.0F * rtb_q1 + (rtb_Switch_a_idx_3 -
    rtb_DiscreteTimeIntegrator1_b) * 2.0F * rtb_MathFunction_jg_idx_1) +
    (rtb_Multiply1_e * rtb_Multiply2_o + rtb_Sum_i * rtb_Sum_e) * 2.0F *
    rtb_Switch_a_idx_2;
  rtb_MathFunction_p_idx_1 = rtb_Subtract1_j;

  /* Sum: '<S14>/Sum of Elements' incorporates:
   *  Math: '<S14>/Math Function'
   *  SignalConversion: '<S4>/ConcatBufferAtMatrix ConcatenateIn1'
   */
  rtb_Subtract1_j = rtb_Sum_i * rtb_Sum_i + rtb_Subtract1_j * rtb_Subtract1_j;

  /* Math: '<S14>/Math Function1' incorporates:
   *  Sum: '<S14>/Sum of Elements'
   *
   * About '<S14>/Math Function1':
   *  Operator: sqrt
   */
  if (rtb_Subtract1_j < 0.0F) {
    rtb_Subtract1_j = -sqrtf(fabsf(rtb_Subtract1_j));
  } else {
    rtb_Subtract1_j = sqrtf(rtb_Subtract1_j);
  }

  /* End of Math: '<S14>/Math Function1' */

  /* Switch: '<S14>/Switch' incorporates:
   *  Constant: '<S14>/Constant'
   *  Product: '<S14>/Product'
   */
  if (rtb_Subtract1_j > 0.0F) {
    rtb_Switch_a_idx_3 = rtb_Subtract1_j;
  } else {
    rtb_Sum_i *= 0.0F;
    rtb_MathFunction_p_idx_1 *= 0.0F;
    rtb_Switch_a_idx_3 = 1.0F;
  }

  /* End of Switch: '<S14>/Switch' */

  /* Product: '<S12>/Divide1' */
  rtb_q1 = rtb_DiscreteTimeIntegrator1_i_0 / rtb_rot_est_B_idx_2;
  rtb_MathFunction_jg_idx_1 = rtb_rot_est_B_idx_1 / rtb_rot_est_B_idx_2;
  rtb_Switch_a_idx_2 = rtb_rot_est_B_idx_0 / rtb_rot_est_B_idx_2;

  /* Product: '<S18>/Product1' */
  rtb_DiscreteTimeIntegrator_n = rtb_Max / rtb_DiscreteTimeIntegrator1_i;

  /* Product: '<S18>/Product2' */
  rtb_Multiply1_e = rtb_Max1 / rtb_DiscreteTimeIntegrator1_i;

  /* Product: '<S18>/Product' */
  rtb_DiscreteTimeIntegrator1_i_0 = rtb_DiscreteTimeIntegrator1_idx /
    rtb_DiscreteTimeIntegrator1_i;

  /* Product: '<S18>/Product3' */
  rtb_Subtract1_j = rtb_Product3_cy / rtb_DiscreteTimeIntegrator1_i;

  /* Sum: '<S16>/Sum' incorporates:
   *  Constant: '<S16>/Constant'
   *  Gain: '<S16>/Gain'
   *  Gain: '<S16>/Gain1'
   *  Gain: '<S16>/Gain2'
   *  Product: '<S16>/Product'
   *  Product: '<S16>/Product1'
   *  Product: '<S16>/Product2'
   *  Product: '<S16>/Product3'
   *  Product: '<S16>/Product4'
   *  Product: '<S16>/Product5'
   *  Product: '<S16>/Product6'
   *  Product: '<S16>/Product7'
   *  Product: '<S16>/Product8'
   *  Sum: '<S16>/Sum1'
   *  Sum: '<S16>/Sum2'
   *  Sum: '<S16>/Sum3'
   */
  rtb_Multiply2_o = (((0.5F - rtb_DiscreteTimeIntegrator_n *
                       rtb_DiscreteTimeIntegrator_n) - rtb_Subtract1_j *
                      rtb_Subtract1_j) * 2.0F * rtb_MathFunction_jg_idx_1 +
                     (rtb_DiscreteTimeIntegrator_n * rtb_Multiply1_e +
                      rtb_DiscreteTimeIntegrator1_i_0 * rtb_Subtract1_j) * 2.0F *
                     rtb_q1) + (rtb_Multiply1_e * rtb_Subtract1_j -
    rtb_DiscreteTimeIntegrator1_i_0 * rtb_DiscreteTimeIntegrator_n) * 2.0F *
    rtb_Switch_a_idx_2;

  /* Sum: '<S17>/Sum' incorporates:
   *  Constant: '<S17>/Constant'
   *  Gain: '<S17>/Gain1'
   *  Gain: '<S17>/Gain2'
   *  Gain: '<S17>/Gain3'
   *  Product: '<S17>/Product1'
   *  Product: '<S17>/Product2'
   *  Product: '<S17>/Product3'
   *  Product: '<S17>/Product4'
   *  Product: '<S17>/Product5'
   *  Product: '<S17>/Product6'
   *  Product: '<S17>/Product7'
   *  Product: '<S17>/Product8'
   *  Product: '<S17>/Product9'
   *  Sum: '<S17>/Sum1'
   *  Sum: '<S17>/Sum2'
   *  Sum: '<S17>/Sum3'
   */
  rtb_Sum_e = ((rtb_DiscreteTimeIntegrator_n * rtb_Subtract1_j -
                rtb_DiscreteTimeIntegrator1_i_0 * rtb_Multiply1_e) * 2.0F *
               rtb_q1 + (rtb_DiscreteTimeIntegrator1_i_0 *
    rtb_DiscreteTimeIntegrator_n + rtb_Multiply1_e * rtb_Subtract1_j) * 2.0F *
               rtb_MathFunction_jg_idx_1) + ((0.5F -
    rtb_DiscreteTimeIntegrator_n * rtb_DiscreteTimeIntegrator_n) -
    rtb_Multiply1_e * rtb_Multiply1_e) * 2.0F * rtb_Switch_a_idx_2;

  /* Sum: '<S15>/Sum' incorporates:
   *  Constant: '<S15>/Constant'
   *  Gain: '<S15>/Gain'
   *  Gain: '<S15>/Gain1'
   *  Gain: '<S15>/Gain2'
   *  Product: '<S15>/Product'
   *  Product: '<S15>/Product1'
   *  Product: '<S15>/Product2'
   *  Product: '<S15>/Product3'
   *  Product: '<S15>/Product4'
   *  Product: '<S15>/Product5'
   *  Product: '<S15>/Product6'
   *  Product: '<S15>/Product7'
   *  Product: '<S15>/Product8'
   *  Sum: '<S15>/Sum1'
   *  Sum: '<S15>/Sum2'
   *  Sum: '<S15>/Sum3'
   */
  rtb_DiscreteTimeIntegrator1_b = (((0.5F - rtb_Multiply1_e * rtb_Multiply1_e) -
    rtb_Subtract1_j * rtb_Subtract1_j) * 2.0F * rtb_q1 +
    (rtb_DiscreteTimeIntegrator_n * rtb_Multiply1_e -
     rtb_DiscreteTimeIntegrator1_i_0 * rtb_Subtract1_j) * 2.0F *
    rtb_MathFunction_jg_idx_1) + (rtb_DiscreteTimeIntegrator_n * rtb_Subtract1_j
    + rtb_DiscreteTimeIntegrator1_i_0 * rtb_Multiply1_e) * 2.0F *
    rtb_Switch_a_idx_2;

  /* Product: '<S14>/Divide1' */
  rtb_q1 = rtb_Sum_i / rtb_Switch_a_idx_3;
  rtb_MathFunction_jg_idx_1 = rtb_MathFunction_p_idx_1 / rtb_Switch_a_idx_3;

  /* Product: '<S29>/Multiply2' */
  rtb_DiscreteTimeIntegrator1_i_0 = rtb_q1 * 0.0F;

  /* Product: '<S30>/Multiply1' */
  rtb_rot_est_B_idx_1 = rtb_q1 * 0.0F;

  /* Product: '<S30>/Multiply2' */
  rtb_rot_est_B_idx_2 = rtb_MathFunction_jg_idx_1;

  /* Sum: '<S4>/Add1' incorporates:
   *  Product: '<S27>/Multiply'
   *  Product: '<S27>/Multiply1'
   *  Product: '<S27>/Multiply2'
   *  Product: '<S28>/Multiply'
   *  Product: '<S28>/Multiply1'
   *  Product: '<S28>/Multiply2'
   *  Product: '<S29>/Multiply'
   *  Sum: '<S10>/Sum'
   *  Sum: '<S9>/Sum'
   */
  rtb_q1 = (-rtb_Multiply2_o - rtb_Sum_e * 0.0F) + rtb_MathFunction_jg_idx_1 *
    0.0F;
  rtb_MathFunction_jg_idx_1 = (rtb_Sum_e * 0.0F -
    (-rtb_DiscreteTimeIntegrator1_b)) + (0.0F - rtb_rot_est_B_idx_1);
  rtb_Switch_a_idx_2 = (rtb_DiscreteTimeIntegrator1_b * 0.0F - rtb_Multiply2_o *
                        0.0F) + (rtb_DiscreteTimeIntegrator1_i_0 -
    rtb_rot_est_B_idx_2);

  /* Product: '<S34>/Product1' */
  rtb_Subtract1_j = rtb_Max / rtb_DiscreteTimeIntegrator1_i;

  /* Product: '<S34>/Product2' */
  rtb_DiscreteTimeIntegrator_n = rtb_Max1 / rtb_DiscreteTimeIntegrator1_i;

  /* Product: '<S34>/Product' */
  rtb_Multiply1_e = rtb_DiscreteTimeIntegrator1_idx /
    rtb_DiscreteTimeIntegrator1_i;

  /* Product: '<S34>/Product3' */
  rtb_DiscreteTimeIntegrator1_b = rtb_Product3_cy /
    rtb_DiscreteTimeIntegrator1_i;

  /* Sum: '<S31>/Sum' incorporates:
   *  Constant: '<S31>/Constant'
   *  Gain: '<S31>/Gain'
   *  Gain: '<S31>/Gain2'
   *  Gain: '<S31>/Gain3'
   *  Product: '<S31>/Product'
   *  Product: '<S31>/Product1'
   *  Product: '<S31>/Product2'
   *  Product: '<S31>/Product4'
   *  Product: '<S31>/Product5'
   *  Product: '<S31>/Product6'
   *  Product: '<S31>/Product7'
   *  Product: '<S31>/Product8'
   *  Product: '<S31>/Product9'
   *  Sum: '<S31>/Sum1'
   *  Sum: '<S31>/Sum2'
   *  Sum: '<S31>/Sum3'
   */
  rtb_Sum_e = (((0.5F - rtb_DiscreteTimeIntegrator_n *
                 rtb_DiscreteTimeIntegrator_n) - rtb_DiscreteTimeIntegrator1_b *
                rtb_DiscreteTimeIntegrator1_b) * 2.0F * rtb_q1 +
               (rtb_Subtract1_j * rtb_DiscreteTimeIntegrator_n + rtb_Multiply1_e
                * rtb_DiscreteTimeIntegrator1_b) * 2.0F *
               rtb_MathFunction_jg_idx_1) + (rtb_Subtract1_j *
    rtb_DiscreteTimeIntegrator1_b - rtb_Multiply1_e *
    rtb_DiscreteTimeIntegrator_n) * 2.0F * rtb_Switch_a_idx_2;

  /* Sum: '<S32>/Sum' incorporates:
   *  Constant: '<S32>/Constant'
   *  Gain: '<S32>/Gain1'
   *  Gain: '<S32>/Gain2'
   *  Gain: '<S32>/Gain3'
   *  Product: '<S32>/Product1'
   *  Product: '<S32>/Product2'
   *  Product: '<S32>/Product3'
   *  Product: '<S32>/Product4'
   *  Product: '<S32>/Product5'
   *  Product: '<S32>/Product6'
   *  Product: '<S32>/Product7'
   *  Product: '<S32>/Product8'
   *  Product: '<S32>/Product9'
   *  Sum: '<S32>/Sum1'
   *  Sum: '<S32>/Sum2'
   *  Sum: '<S32>/Sum3'
   */
  rtb_Sum_i = (((0.5F - rtb_Subtract1_j * rtb_Subtract1_j) -
                rtb_DiscreteTimeIntegrator1_b * rtb_DiscreteTimeIntegrator1_b) *
               2.0F * rtb_MathFunction_jg_idx_1 + (rtb_Subtract1_j *
    rtb_DiscreteTimeIntegrator_n - rtb_Multiply1_e *
    rtb_DiscreteTimeIntegrator1_b) * 2.0F * rtb_q1) + (rtb_Multiply1_e *
    rtb_Subtract1_j + rtb_DiscreteTimeIntegrator_n *
    rtb_DiscreteTimeIntegrator1_b) * 2.0F * rtb_Switch_a_idx_2;

  /* Sum: '<S33>/Sum' incorporates:
   *  Constant: '<S33>/Constant'
   *  Gain: '<S33>/Gain1'
   *  Gain: '<S33>/Gain2'
   *  Gain: '<S33>/Gain3'
   *  Product: '<S33>/Product1'
   *  Product: '<S33>/Product2'
   *  Product: '<S33>/Product3'
   *  Product: '<S33>/Product4'
   *  Product: '<S33>/Product5'
   *  Product: '<S33>/Product6'
   *  Product: '<S33>/Product7'
   *  Product: '<S33>/Product8'
   *  Product: '<S33>/Product9'
   *  Sum: '<S33>/Sum1'
   *  Sum: '<S33>/Sum2'
   *  Sum: '<S33>/Sum3'
   */
  rtb_DiscreteTimeIntegrator1_b = ((rtb_Subtract1_j *
    rtb_DiscreteTimeIntegrator1_b + rtb_Multiply1_e *
    rtb_DiscreteTimeIntegrator_n) * 2.0F * rtb_q1 +
    (rtb_DiscreteTimeIntegrator_n * rtb_DiscreteTimeIntegrator1_b -
     rtb_Multiply1_e * rtb_Subtract1_j) * 2.0F * rtb_MathFunction_jg_idx_1) +
    ((0.5F - rtb_Subtract1_j * rtb_Subtract1_j) - rtb_DiscreteTimeIntegrator_n *
     rtb_DiscreteTimeIntegrator_n) * 2.0F * rtb_Switch_a_idx_2;

  /* DiscreteIntegrator: '<S37>/Discrete-Time Integrator1' */
  if (CF_INS_DW.DiscreteTimeIntegrator1_PrevRes != 0) {
    CF_INS_DW.DiscreteTimeIntegrator1_DSTATE[0] = CF_INS_ConstB.Constant7[0];
    CF_INS_DW.DiscreteTimeIntegrator1_DSTATE[1] = CF_INS_ConstB.Constant7[1];
    CF_INS_DW.DiscreteTimeIntegrator1_DSTATE[2] = CF_INS_ConstB.Constant7[2];
  }

  /* Sum: '<S37>/Subtract' incorporates:
   *  DiscreteIntegrator: '<S37>/Discrete-Time Integrator1'
   *  Gain: '<S37>/gain'
   */
  rtb_rot_est_B_idx_0 = rtu_Sensor_Data_IMU_Data_rot_ra[0] -
    (-CF_INS_DW.DiscreteTimeIntegrator1_DSTATE[0]);
  rtb_rot_est_B_idx_1 = rtu_Sensor_Data_IMU_Data_rot_ra[1] -
    (-CF_INS_DW.DiscreteTimeIntegrator1_DSTATE[1]);
  rtb_rot_est_B_idx_2 = rtu_Sensor_Data_IMU_Data_rot_ra[2] -
    (-CF_INS_DW.DiscreteTimeIntegrator1_DSTATE[2]);

  /* Sum: '<S5>/Add' incorporates:
   *  Gain: '<S5>/Kp'
   */
  rtb_q1 = 0.25F * rtb_Sum_e + rtb_rot_est_B_idx_0;
  rtb_MathFunction_jg_idx_1 = 0.25F * rtb_Sum_i + rtb_rot_est_B_idx_1;
  rtb_Switch_a_idx_2 = 0.25F * rtb_DiscreteTimeIntegrator1_b +
    rtb_rot_est_B_idx_2;

  /* Gain: '<S37>/Ki' */
  rtb_DiscreteTimeIntegrator1_i_0 = 0.1F * rtb_DiscreteTimeIntegrator1_b;

  /* DiscreteIntegrator: '<S61>/Discrete-Time Integrator' */
  rtb_Multiply2_o = CF_INS_DW.DiscreteTimeIntegrator_DSTATE_k;

  /* Product: '<S78>/Product' */
  rtb_DiscreteTimeIntegrator1_b = rtb_DiscreteTimeIntegrator1_idx /
    rtb_DiscreteTimeIntegrator1_i;

  /* Product: '<S78>/Product2' */
  rtb_Subtract1_j = rtb_Max1 / rtb_DiscreteTimeIntegrator1_i;

  /* Product: '<S78>/Product1' */
  rtb_DiscreteTimeIntegrator_n = rtb_Max / rtb_DiscreteTimeIntegrator1_i;

  /* Product: '<S78>/Product3' */
  rtb_DiscreteTimeIntegrator1_i = rtb_Product3_cy /
    rtb_DiscreteTimeIntegrator1_i;

  /* Product: '<S77>/Product4' incorporates:
   *  Gain: '<S77>/Gain3'
   *  Product: '<S77>/Product1'
   *  Product: '<S77>/Product9'
   *  Sum: '<S77>/Sum1'
   */
  rtb_Multiply1_e = (rtb_DiscreteTimeIntegrator_n *
                     rtb_DiscreteTimeIntegrator1_i -
                     rtb_DiscreteTimeIntegrator1_b * rtb_Subtract1_j) * 2.0F *
    rtu_Sensor_Data_IMU_Data_sfor_m[0];

  /* Product: '<S77>/Product5' incorporates:
   *  Gain: '<S77>/Gain1'
   *  Product: '<S77>/Product2'
   *  Product: '<S77>/Product3'
   *  Sum: '<S77>/Sum2'
   */
  rtb_DiscreteTimeIntegrator1_i = (rtb_DiscreteTimeIntegrator1_b *
    rtb_DiscreteTimeIntegrator_n + rtb_Subtract1_j *
    rtb_DiscreteTimeIntegrator1_i) * 2.0F * rtu_Sensor_Data_IMU_Data_sfor_m[1];

  /* Sum: '<S61>/Subtract' incorporates:
   *  Constant: '<S72>/Gravity'
   *  Constant: '<S77>/Constant'
   *  DiscreteIntegrator: '<S73>/Discrete-Time Integrator1'
   *  Gain: '<S73>/gain'
   *  Gain: '<S77>/Gain2'
   *  Product: '<S77>/Product6'
   *  Product: '<S77>/Product7'
   *  Product: '<S77>/Product8'
   *  Sum: '<S72>/Add2'
   *  Sum: '<S77>/Sum'
   *  Sum: '<S77>/Sum3'
   */
  rtb_DiscreteTimeIntegrator1_b = ((((0.5F - rtb_DiscreteTimeIntegrator_n *
    rtb_DiscreteTimeIntegrator_n) - rtb_Subtract1_j * rtb_Subtract1_j) * 2.0F *
    rtu_Sensor_Data_IMU_Data_sfor_m[2] + (rtb_Multiply1_e +
    rtb_DiscreteTimeIntegrator1_i)) + 9.8066F) -
    (-CF_INS_DW.DiscreteTimeIntegrator1_DSTAT_h);

  /* Product: '<S40>/Multiply6' incorporates:
   *  Constant: '<S40>/Constant'
   *  Product: '<S40>/Multiply4'
   *  Product: '<S40>/Multiply5'
   *  Sum: '<S40>/Subtract'
   */
  rtb_Subtract1_j = (rtb_DiscreteTimeIntegrator1_idx * rtb_Max1 -
                     rtb_Product3_cy * rtb_Max) * 2.0F;

  /* DiscreteIntegrator: '<S59>/Discrete-Time Integrator' */
  rtb_DiscreteTimeIntegrator1_i = CF_INS_DW.DiscreteTimeIntegrator_DSTATE_i;

  /* BusCreator: '<Root>/BusConversion_InsertedFor_INS_Out_at_inport_0' incorporates:
   *  Constant: '<S40>/Constant'
   *  Constant: '<S40>/Constant2'
   *  Product: '<S40>/Multiply'
   *  Product: '<S40>/Multiply1'
   *  Product: '<S40>/Multiply2'
   *  Product: '<S40>/Multiply3'
   *  Sum: '<S40>/Add'
   *  Sum: '<S40>/Add1'
   *  Sum: '<S40>/Subtract2'
   *  Trigonometry: '<S40>/Atan2'
   */
  rty_INS_Out->quat[0] = rtb_DiscreteTimeIntegrator1_idx;
  rty_INS_Out->quat[1] = rtb_Max;
  rty_INS_Out->quat[2] = rtb_Max1;
  rty_INS_Out->quat[3] = rtb_Product3_cy;
  rty_INS_Out->euler[0] = rt_atan2f_snf((rtb_DiscreteTimeIntegrator1_idx *
    rtb_Max + rtb_Max1 * rtb_Product3_cy) * 2.0F, 1.0F - (rtb_ki + rtb_vz_dot) *
    2.0F);
  rty_INS_Out->rot_radPs_B[0] = rtb_rot_est_B_idx_0;

  /* Saturate: '<S40>/Saturation' */
  if (rtb_Subtract1_j > 1.0F) {
    rtb_Subtract1_j = 1.0F;
  } else {
    if (rtb_Subtract1_j < -1.0F) {
      rtb_Subtract1_j = -1.0F;
    }
  }

  /* End of Saturate: '<S40>/Saturation' */

  /* BusCreator: '<Root>/BusConversion_InsertedFor_INS_Out_at_inport_0' incorporates:
   *  Constant: '<S40>/Constant'
   *  Constant: '<S40>/Constant1'
   *  Constant: '<S56>/Constant'
   *  Constant: '<S56>/Constant1'
   *  Product: '<S40>/Multiply10'
   *  Product: '<S40>/Multiply7'
   *  Product: '<S40>/Multiply8'
   *  Product: '<S40>/Multiply9'
   *  Sum: '<S40>/Add2'
   *  Sum: '<S40>/Add3'
   *  Sum: '<S40>/Subtract1'
   *  Trigonometry: '<S40>/Asin'
   *  Trigonometry: '<S40>/Atan1'
   */
  rty_INS_Out->euler[1] = asinf(rtb_Subtract1_j);
  rty_INS_Out->rot_radPs_B[1] = rtb_rot_est_B_idx_1;
  rty_INS_Out->euler[2] = rt_atan2f_snf((rtb_DiscreteTimeIntegrator1_idx *
    rtb_Product3_cy + rtb_Max * rtb_Max1) * 2.0F, 1.0F - (rtb_Add_p + rtb_vz_dot)
    * 2.0F);
  rty_INS_Out->rot_radPs_B[2] = rtb_rot_est_B_idx_2;
  rty_INS_Out->acc_mPs2_O[0] = 0.0F;
  rty_INS_Out->acc_mPs2_O[1] = 0.0F;
  rty_INS_Out->acc_mPs2_O[2] = rtb_DiscreteTimeIntegrator1_b;
  rty_INS_Out->vel_cmPs_O[0] = 0;
  rty_INS_Out->vel_cmPs_O[1] = 0;

  /* Gain: '<S57>/Gain1' incorporates:
   *  DiscreteIntegrator: '<S61>/Discrete-Time Integrator'
   */
  rtb_Subtract1_j = floorf(100.0F * CF_INS_DW.DiscreteTimeIntegrator_DSTATE_k);
  if (rtIsNaNF(rtb_Subtract1_j) || rtIsInfF(rtb_Subtract1_j)) {
    rtb_Subtract1_j = 0.0F;
  } else {
    rtb_Subtract1_j = fmodf(rtb_Subtract1_j, 4.2949673E+9F);
  }

  /* BusCreator: '<Root>/BusConversion_InsertedFor_INS_Out_at_inport_0' incorporates:
   *  Constant: '<S56>/Constant'
   *  Gain: '<S57>/Gain1'
   */
  rty_INS_Out->vel_cmPs_O[2] = rtb_Subtract1_j < 0.0F ? -(int32_T)(uint32_T)
    -rtb_Subtract1_j : (int32_T)(uint32_T)rtb_Subtract1_j;
  rty_INS_Out->lon_1e7_deg = 0;
  rty_INS_Out->lat_1e7_deg = 0;

  /* Gain: '<S57>/Gain' incorporates:
   *  DiscreteIntegrator: '<S59>/Discrete-Time Integrator'
   */
  rtb_Subtract1_j = floorf(100.0F * CF_INS_DW.DiscreteTimeIntegrator_DSTATE_i);
  if (rtIsNaNF(rtb_Subtract1_j) || rtIsInfF(rtb_Subtract1_j)) {
    rtb_Subtract1_j = 0.0F;
  } else {
    rtb_Subtract1_j = fmodf(rtb_Subtract1_j, 4.2949673E+9F);
  }

  /* BusCreator: '<Root>/BusConversion_InsertedFor_INS_Out_at_inport_0' incorporates:
   *  Constant: '<S2>/Constant'
   *  Gain: '<S57>/Gain'
   */
  rty_INS_Out->altitude_cm = rtb_Subtract1_j < 0.0F ? -(int32_T)(uint32_T)
    -rtb_Subtract1_j : (int32_T)(uint32_T)rtb_Subtract1_j;
  rty_INS_Out->timestamp_ms = 0U;

  /* Sum: '<S60>/Add' incorporates:
   *  Delay: '<S60>/Delay'
   *  DiscreteIntegrator: '<S67>/Discrete-Time Integrator'
   *  Gain: '<S65>/Gain'
   *  Gain: '<S65>/baro_kp'
   *  Gain: '<S66>/gps_kp'
   *  Product: '<S65>/Multiply1'
   *  Product: '<S66>/Multiply'
   *  Sum: '<S65>/Subtract1'
   *  Sum: '<S66>/Subtract'
   */
  rtb_Subtract1_j = (*rtu_Sensor_Data_GPS_Data_velD_m - CF_INS_DW.Delay_DSTATE[0])
    * *rtu_Sensor_Data_GPS_Data_vel_qu * 0.2F +
    (-CF_INS_DW.DiscreteTimeIntegrator_DSTAT_ku - CF_INS_DW.Delay_DSTATE[0]) *
    (real32_T)*rtu_Sensor_Data_Baro_Data_valid * 0.2F;

  /* Sum: '<S61>/Add1' */
  rtb_vz_dot = rtb_DiscreteTimeIntegrator1_b + rtb_Subtract1_j;

  /* Gain: '<S73>/ki' */
  rtb_ki = 0.05F * rtb_Subtract1_j;

  /* DiscreteIntegrator: '<S67>/Discrete-Time Integrator1' */
  if (CF_INS_DW.DiscreteTimeIntegrator1_IC_LOAD != 0) {
    CF_INS_DW.DiscreteTimeIntegrator1_DSTAT_c = *rtu_Sensor_Data_Baro_Data_relat;
  }

  /* Sum: '<S67>/Subtract' incorporates:
   *  DiscreteIntegrator: '<S67>/Discrete-Time Integrator1'
   */
  rtb_Subtract1_j = *rtu_Sensor_Data_Baro_Data_relat -
    CF_INS_DW.DiscreteTimeIntegrator1_DSTAT_c;

  /* Sum: '<S67>/Add' incorporates:
   *  DiscreteIntegrator: '<S67>/Discrete-Time Integrator'
   *  Gain: '<S67>/Gain2'
   */
  rtb_Add_p = 10.0F * rtb_Subtract1_j +
    CF_INS_DW.DiscreteTimeIntegrator_DSTAT_ku;

  /* Gain: '<S59>/Gain' incorporates:
   *  DiscreteIntegrator: '<S61>/Discrete-Time Integrator'
   */
  rtb_Multiply1_e = -CF_INS_DW.DiscreteTimeIntegrator_DSTATE_k;

  /* Update for DiscreteIntegrator: '<S5>/Discrete-Time Integrator' incorporates:
   *  Constant: '<S5>/Constant'
   *  Gain: '<S5>/Gain1'
   *  Product: '<S41>/Multiply'
   *  Product: '<S41>/Multiply1'
   *  Product: '<S41>/Multiply2'
   *  Product: '<S41>/Multiply3'
   *  Product: '<S42>/Multiply'
   *  Product: '<S42>/Multiply1'
   *  Product: '<S42>/Multiply2'
   *  Product: '<S42>/Multiply3'
   *  Product: '<S43>/Multiply'
   *  Product: '<S43>/Multiply1'
   *  Product: '<S43>/Multiply2'
   *  Product: '<S43>/Multiply3'
   *  Product: '<S44>/Multiply'
   *  Product: '<S44>/Multiply1'
   *  Product: '<S44>/Multiply2'
   *  Product: '<S44>/Multiply3'
   *  Sum: '<S41>/Add'
   *  Sum: '<S42>/Add'
   *  Sum: '<S43>/Add'
   *  Sum: '<S44>/Add'
   */
  CF_INS_DW.DiscreteTimeIntegrator_IC_LOADI = 0U;
  CF_INS_DW.DiscreteTimeIntegrator_DSTATE[0] +=
    (((rtb_DiscreteTimeIntegrator1_idx * 0.0F - rtb_Max * rtb_q1) - rtb_Max1 *
      rtb_MathFunction_jg_idx_1) - rtb_Product3_cy * rtb_Switch_a_idx_2) * 0.5F *
    0.002F;
  CF_INS_DW.DiscreteTimeIntegrator_DSTATE[1] += (((rtb_Max * 0.0F +
    rtb_DiscreteTimeIntegrator1_idx * rtb_q1) + rtb_Max1 * rtb_Switch_a_idx_2) -
    rtb_Product3_cy * rtb_MathFunction_jg_idx_1) * 0.5F * 0.002F;
  CF_INS_DW.DiscreteTimeIntegrator_DSTATE[2] += (((rtb_Max1 * 0.0F +
    rtb_DiscreteTimeIntegrator1_idx * rtb_MathFunction_jg_idx_1) +
    rtb_Product3_cy * rtb_q1) - rtb_Max * rtb_Switch_a_idx_2) * 0.5F * 0.002F;
  CF_INS_DW.DiscreteTimeIntegrator_DSTATE[3] += (((rtb_Product3_cy * 0.0F +
    rtb_DiscreteTimeIntegrator1_idx * rtb_Switch_a_idx_2) + rtb_Max *
    rtb_MathFunction_jg_idx_1) - rtb_Max1 * rtb_q1) * 0.5F * 0.002F;
  CF_INS_DW.DiscreteTimeIntegrator_PrevRese = 0;

  /* Update for DiscreteIntegrator: '<S37>/Discrete-Time Integrator1' incorporates:
   *  Gain: '<S37>/Ki'
   */
  CF_INS_DW.DiscreteTimeIntegrator1_DSTATE[0] += 0.1F * rtb_Sum_e * 0.002F;
  CF_INS_DW.DiscreteTimeIntegrator1_DSTATE[1] += 0.1F * rtb_Sum_i * 0.002F;
  CF_INS_DW.DiscreteTimeIntegrator1_DSTATE[2] += 0.002F *
    rtb_DiscreteTimeIntegrator1_i_0;
  CF_INS_DW.DiscreteTimeIntegrator1_PrevRes = 0;

  /* Update for DiscreteIntegrator: '<S61>/Discrete-Time Integrator' */
  CF_INS_DW.DiscreteTimeIntegrator_DSTATE_k += 0.002F * rtb_vz_dot;

  /* Update for DiscreteIntegrator: '<S73>/Discrete-Time Integrator1' */
  CF_INS_DW.DiscreteTimeIntegrator1_DSTAT_h += 0.002F * rtb_ki;
  if (CF_INS_DW.DiscreteTimeIntegrator1_DSTAT_h >= 0.5F) {
    CF_INS_DW.DiscreteTimeIntegrator1_DSTAT_h = 0.5F;
  } else {
    if (CF_INS_DW.DiscreteTimeIntegrator1_DSTAT_h <= -0.5F) {
      CF_INS_DW.DiscreteTimeIntegrator1_DSTAT_h = -0.5F;
    }
  }

  /* End of Update for DiscreteIntegrator: '<S73>/Discrete-Time Integrator1' */

  /* Update for DiscreteIntegrator: '<S59>/Discrete-Time Integrator' incorporates:
   *  Delay: '<S58>/Delay'
   *  Product: '<S62>/Multiply1'
   *  Product: '<S63>/Multiply'
   *  Sum: '<S58>/Add'
   *  Sum: '<S59>/Add'
   *  Sum: '<S62>/Subtract1'
   *  Sum: '<S63>/Subtract'
   */
  CF_INS_DW.DiscreteTimeIntegrator_DSTATE_i +=
    (((*rtu_Sensor_Data_GPS_Data_height - CF_INS_DW.Delay_DSTATE_j[0]) *
      *rtu_Sensor_Data_GPS_Data_pos_qu + (*rtu_Sensor_Data_Baro_Data_relat -
       CF_INS_DW.Delay_DSTATE_j[0]) * (real32_T)*rtu_Sensor_Data_Baro_Data_valid)
     + rtb_Multiply1_e) * 0.002F;
  for (idxDelay = 0; idxDelay < 9; idxDelay++) {
    /* Update for Delay: '<S60>/Delay' */
    CF_INS_DW.Delay_DSTATE[idxDelay] = CF_INS_DW.Delay_DSTATE[idxDelay + 1];

    /* Update for Delay: '<S58>/Delay' */
    CF_INS_DW.Delay_DSTATE_j[idxDelay] = CF_INS_DW.Delay_DSTATE_j[idxDelay + 1];
  }

  /* Update for Delay: '<S60>/Delay' */
  CF_INS_DW.Delay_DSTATE[9] = rtb_Multiply2_o;

  /* Update for DiscreteIntegrator: '<S67>/Discrete-Time Integrator' incorporates:
   *  Gain: '<S67>/Gain3'
   */
  CF_INS_DW.DiscreteTimeIntegrator_DSTAT_ku += 25.0F * rtb_Subtract1_j * 0.002F;

  /* Update for DiscreteIntegrator: '<S67>/Discrete-Time Integrator1' */
  CF_INS_DW.DiscreteTimeIntegrator1_IC_LOAD = 0U;
  CF_INS_DW.DiscreteTimeIntegrator1_DSTAT_c += 0.002F * rtb_Add_p;

  /* Update for Delay: '<S58>/Delay' */
  CF_INS_DW.Delay_DSTATE_j[9] = rtb_DiscreteTimeIntegrator1_i;
}

/* Model initialize function */
void CF_INS_initialize(const char_T **rt_errorStatus)
{
  RT_MODEL_CF_INS_T *const CF_INS_M = &(CF_INS_MdlrefDW.rtm);

  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* initialize error status */
  rtmSetErrorStatusPointer(CF_INS_M, rt_errorStatus);

  /* states (dwork) */
  (void) memset((void *)&CF_INS_DW, 0,
                sizeof(DW_CF_INS_f_T));
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
