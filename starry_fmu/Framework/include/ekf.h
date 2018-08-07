#ifndef __EKF_H__
#define __EKF_H__

#include <arm_math.h>
#include "global.h"

typedef struct
{
	arm_matrix_instance_f32 X;		// states
	arm_matrix_instance_f32 U;		// control vector
	arm_matrix_instance_f32 Z;		// observation vector
	
	arm_matrix_instance_f32 F;		
	arm_matrix_instance_f32 B;
	arm_matrix_instance_f32 H;
	arm_matrix_instance_f32 P;	
	arm_matrix_instance_f32 Q;	
	arm_matrix_instance_f32 R;
	arm_matrix_instance_f32 y;
	arm_matrix_instance_f32 S;
	arm_matrix_instance_f32 K;		// kalman gain
	float dT;	// update interval
	/* control flag */
	bool identity_h;
//	/* temporary matrix */
//	Mat M_nn_1, M_nn_2, M_nn_3, M_nn_4;
//	Mat M_n1_1, M_n1_2;
//	Mat I; // identity matrix
}EKF_Def;

//void EKF_Create(EKF_Def* kf_t, int x_dim, int u_dim);
//void EKF_Init(EKF_Def* kf_t, float *F_val, float *B_val, float *H_val, float *P_val, float *Q_val, float *R_val, float *x_val, bool identity_h, float dt);
//void EKF_Predict(EKF_Def* kf_t);
//void EKF_Update(EKF_Def* kf_t);

#endif
