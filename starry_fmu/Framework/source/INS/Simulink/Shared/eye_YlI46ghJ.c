/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: eye_YlI46ghJ.c
 *
 * Code generated for Simulink model 'INS'.
 *
 * Model version                  : 1.138
 * Simulink Coder version         : 9.0 (R2018b) 24-May-2018
 * C/C++ source code generated on : Sun Oct 28 11:44:51 2018
 */

#include "rtwtypes.h"
#include <string.h>
#include "eye_YlI46ghJ.h"

/* Function for MATLAB Function: '<S4>/MATLAB Function' */
void eye_YlI46ghJ(real_T b_I[196])
{
  int32_T k;
  memset(&b_I[0], 0, 196U * sizeof(real_T));
  for (k = 0; k < 14; k++) {
    b_I[k + 14 * k] = 1.0;
  }
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
