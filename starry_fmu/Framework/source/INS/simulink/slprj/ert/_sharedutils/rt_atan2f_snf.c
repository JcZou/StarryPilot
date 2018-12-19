/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: rt_atan2f_snf.c
 *
 * Code generated for Simulink model 'CF_INS'.
 *
 * Model version                  : 1.338
 * Simulink Coder version         : 9.0 (R2018b) 24-May-2018
 * C/C++ source code generated on : Sun Dec 16 13:14:49 2018
 */

#include "rtwtypes.h"
#include "rtGetNaN.h"
#include "rt_nonfinite.h"
#include <math.h>
#include "rt_defines.h"
#include "rt_atan2f_snf.h"

real32_T rt_atan2f_snf(real32_T u0, real32_T u1)
{
	real32_T y;
	int32_T u0_0;
	int32_T u1_0;

	if(rtIsNaNF(u0) || rtIsNaNF(u1)) {
		y = (rtNaNF);
	} else if(rtIsInfF(u0) && rtIsInfF(u1)) {
		if(u0 > 0.0F) {
			u0_0 = 1;
		} else {
			u0_0 = -1;
		}

		if(u1 > 0.0F) {
			u1_0 = 1;
		} else {
			u1_0 = -1;
		}

		y = atan2f((real32_T)u0_0, (real32_T)u1_0);
	} else if(u1 == 0.0F) {
		if(u0 > 0.0F) {
			y = RT_PIF / 2.0F;
		} else if(u0 < 0.0F) {
			y = -(RT_PIF / 2.0F);
		} else {
			y = 0.0F;
		}
	} else {
		y = atan2f(u0, u1);
	}

	return y;
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
