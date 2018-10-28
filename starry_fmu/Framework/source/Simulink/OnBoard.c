/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: OnBoard.c
 *
 * Code generated for Simulink model 'OnBoard'.
 *
 * Model version                  : 1.1010
 * Simulink Coder version         : 9.0 (R2018b) 24-May-2018
 * C/C++ source code generated on : Sun Oct 21 17:18:49 2018
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#include "OnBoard.h"

/* Block signals and states (default storage) */
D_Work rtDWork;

/* External inputs (root inport signals with default storage) */
ExternalInputs rtU;

/* External outputs (root outports fed by signals with default storage) */
ExternalOutputs rtY;

/* Model step function */
void OnBoard_step(void)
{
  real32_T rtb_DiscreteTimeIntegrator_m;
  real32_T rtb_theta_m_k;
  real32_T rtb_Gain;
  real32_T rtb_DataTypeConversion1;

  /* Sum: '<S5>/Add2' incorporates:
   *  Constant: '<S5>/Constant'
   *  DiscreteIntegrator: '<S9>/Discrete-Time Integrator'
   *  Gain: '<S3>/direction2'
   *  Gain: '<S5>/Gain1'
   *  Inport: '<Root>/theta_rad_m'
   *  Sum: '<S5>/Add1'
   */
  rtb_theta_m_k = (0.0F - (-rtU.theta_rad_m)) * 10.0F -
    rtDWork.DiscreteTimeIntegrator_DSTATE;

  /* Gain: '<S6>/Gain' incorporates:
   *  DiscreteIntegrator: '<S6>/Discrete-Time Integrator'
   *  Sum: '<S6>/Add'
   */
  rtb_Gain = (rtb_theta_m_k - rtDWork.DiscreteTimeIntegrator_DSTATE_l) * 40.0F;

  /* Gain: '<S8>/Gain' incorporates:
   *  DiscreteIntegrator: '<S3>/Discrete-Time Integrator1'
   *  DiscreteIntegrator: '<S3>/Discrete-Time Integrator2'
   *  DiscreteIntegrator: '<S8>/Discrete-Time Integrator'
   *  Gain: '<S3>/Gain'
   *  Gain: '<S3>/direction2'
   *  Gain: '<S3>/pulse2rad'
   *  Gain: '<S3>/pulse2rad1'
   *  Inport: '<Root>/theta_rad_m'
   *  Sum: '<S3>/Sum1'
   *  Sum: '<S3>/Sum4'
   *  Sum: '<S3>/Sum6'
   *  Sum: '<S8>/Add'
   */
  rtb_DiscreteTimeIntegrator_m = (((0.0174532924F *
    rtDWork.DiscreteTimeIntegrator1_DSTATE + -rtU.theta_rad_m) + (0.0174532924F *
    rtDWork.DiscreteTimeIntegrator2_DSTATE + -rtU.theta_rad_m)) * 0.5F -
    rtDWork.DiscreteTimeIntegrator_DSTATE_f) * 40.0F;

  /* Sum: '<S5>/Add3' incorporates:
   *  Abs: '<S5>/Abs'
   *  DiscreteIntegrator: '<S7>/Discrete-Time Integrator'
   *  Gain: '<S5>/Gain5'
   *  Product: '<S5>/Product'
   */
  rtb_theta_m_k = 1.8F * rtb_theta_m_k - fabsf(rtb_DiscreteTimeIntegrator_m) *
    rtDWork.DiscreteTimeIntegrator_DSTATE_h;

  /* Saturate: '<S5>/Saturation' */
  if (rtb_theta_m_k > 8.0F) {
    rtb_DataTypeConversion1 = 8.0F;
  } else if (rtb_theta_m_k < -8.0F) {
    rtb_DataTypeConversion1 = -8.0F;
  } else {
    rtb_DataTypeConversion1 = rtb_theta_m_k;
  }

  /* End of Saturate: '<S5>/Saturation' */

  /* Outport: '<Root>/pwm_l' incorporates:
   *  Constant: '<S4>/Constant'
   *  Gain: '<S4>/direction'
   *  Product: '<S4>/Product'
   */
  rtY.pwm_l = -(rtb_DataTypeConversion1 / 8.0F);

  /* Saturate: '<S4>/Saturation1' incorporates:
   *  Constant: '<S4>/Constant1'
   *  Product: '<S4>/Product1'
   */
  rtb_DataTypeConversion1 /= 8.0F;

  /* Outport: '<Root>/pwm_r' incorporates:
   *  Gain: '<S4>/direction1'
   */
  rtY.pwm_r = -rtb_DataTypeConversion1;

  /* Update for DiscreteIntegrator: '<S9>/Discrete-Time Integrator' incorporates:
   *  Gain: '<S3>/direction3'
   *  Gain: '<S9>/Gain'
   *  Inport: '<Root>/theta_dot_radDs_m'
   *  Sum: '<S9>/Add'
   */
  rtDWork.DiscreteTimeIntegrator_DSTATE += (-rtU.theta_dot_radDs_m -
    rtDWork.DiscreteTimeIntegrator_DSTATE) * 20.0F * 0.004F;

  /* Update for DiscreteIntegrator: '<S6>/Discrete-Time Integrator' */
  rtDWork.DiscreteTimeIntegrator_DSTATE_l += 0.004F * rtb_Gain;

  /* Update for DiscreteIntegrator: '<S3>/Discrete-Time Integrator1' incorporates:
   *  DataTypeConversion: '<S3>/Data Type Conversion'
   *  Gain: '<S3>/direction'
   *  Inport: '<Root>/alpha_m_l'
   */
  rtDWork.DiscreteTimeIntegrator1_DSTATE += 0.004F * -(real32_T)rtU.alpha_m_l;

  /* Update for DiscreteIntegrator: '<S3>/Discrete-Time Integrator2' incorporates:
   *  DataTypeConversion: '<S3>/Data Type Conversion1'
   *  Gain: '<S3>/direction1'
   *  Inport: '<Root>/alpha_m_r'
   */
  rtDWork.DiscreteTimeIntegrator2_DSTATE += 0.004F * -(real32_T)rtU.alpha_m_r;

  /* Update for DiscreteIntegrator: '<S8>/Discrete-Time Integrator' */
  rtDWork.DiscreteTimeIntegrator_DSTATE_f += 0.004F *
    rtb_DiscreteTimeIntegrator_m;

  /* Signum: '<S5>/Sign' */
  if (rtb_theta_m_k < 0.0F) {
    rtb_theta_m_k = -1.0F;
  } else {
    if (rtb_theta_m_k > 0.0F) {
      rtb_theta_m_k = 1.0F;
    }
  }

  /* End of Signum: '<S5>/Sign' */

  /* Update for DiscreteIntegrator: '<S7>/Discrete-Time Integrator' incorporates:
   *  Gain: '<S5>/Gain'
   *  Gain: '<S7>/Gain'
   *  Sum: '<S7>/Add'
   */
  rtDWork.DiscreteTimeIntegrator_DSTATE_h += (-rtb_theta_m_k -
    rtDWork.DiscreteTimeIntegrator_DSTATE_h) * 90.0F * 0.004F;
}

/* Model initialize function */
void OnBoard_initialize(void)
{
  /* (no initialization code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
