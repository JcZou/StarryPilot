#ifndef __EKF_H__
#define __EKF_H__

#include <arm_math.h>
#include "global.h"
#include "ap_math.h"

#define STATE_X			0
#define STATE_Y			1
#define STATE_Z			2
#define STATE_VX		3
#define STATE_VY		4
#define STATE_VZ		5
#define STATE_Q0		6
#define STATE_Q1		7
#define STATE_Q2		8
#define STATE_Q3		9
#define STATE_GX_BIAS	10
#define STATE_GY_BIAS	11
#define STATE_GZ_BIAS	12
#define STATE_AZ_BIAS	13

#define MAT_ELEMENT(mat, row, col)			(mat.pData[row*mat.numCols+col])

typedef struct
{
	arm_matrix_instance_f32 X;		// states
	arm_matrix_instance_f32 U;		// control vector
	arm_matrix_instance_f32 Z;		// observation vector
	
	arm_matrix_instance_f32 F;		
	arm_matrix_instance_f32 H;
	arm_matrix_instance_f32 G;
	arm_matrix_instance_f32 P;	
	arm_matrix_instance_f32 Q;	
	arm_matrix_instance_f32 R;
	
	arm_matrix_instance_f32 Y;
	arm_matrix_instance_f32 S;
	arm_matrix_instance_f32 K;		// kalman gain
	
	arm_matrix_instance_f32 IFT;
	arm_matrix_instance_f32 IFTT;
	arm_matrix_instance_f32 IFTP;
	arm_matrix_instance_f32 IFTPIFTT;
	arm_matrix_instance_f32 GQ;
	arm_matrix_instance_f32 GT;
	arm_matrix_instance_f32 GQGT;
	
	arm_matrix_instance_f32 HT;
	arm_matrix_instance_f32 PHT;
	arm_matrix_instance_f32 HPHT;
	arm_matrix_instance_f32 INV_S;
	arm_matrix_instance_f32 KY;
	arm_matrix_instance_f32 KH;
	arm_matrix_instance_f32 KHP;
	
	Vector3f_t magetic_field;
	
	float32_t dT;	// time interval
}EKF_Def;

uint8_t EKF14_Init(EKF_Def* ekf_t, float32_t dT);
void EKF14_Reset(EKF_Def* ekf_t);
uint8_t EKF14_Prediction(EKF_Def* ekf_t);
uint8_t EKF14_SerialPrediction(EKF_Def* ekf_t, uint32_t enable_bitmask);
uint8_t EKF14_Correct(EKF_Def* ekf_t);
uint8_t EKF14_SerialCorrect(EKF_Def* ekf_t, uint32_t enable_bitmask);
float32_t EKF14_Get_State(const EKF_Def* ekf_t, uint8_t state);

#endif
