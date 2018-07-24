/*
 * File      : kf.h
 *
 *
 * Change Logs:
 * Date           Author       	Notes
 * 2017-08-09     zoujiachi   	the first version
 */
 
#ifndef __KF_H__
#define __KF_H__

#include "light_matrix.h"
#include "pos_estimator.h"

#define USE_OPT_KF_GAIN

typedef struct
{
	Mat x;		// states
	Mat u;		// control vector
	Mat z;		// observation vector
	
	Mat F;		
	Mat B;
	Mat H;
	Mat P;	
	Mat Q;	
	Mat R;
	Mat y;
	Mat S;
	Mat K;		// kalman gain
	float dT;	// update interval
	/* control flag */
	bool identity_h;
	/* temporary matrix */
	Mat M_nn_1, M_nn_2, M_nn_3, M_nn_4;
	Mat M_n1_1, M_n1_2;
	Mat I; // identity matrix
}KF_Def;

void KF_Create(KF_Def* kf_t, int x_dim, int u_dim);
void KF_Init(KF_Def* kf_t, float *F_val, float *B_val, float *H_val, float *P_val, float *Q_val, float *R_val, float *x_val, bool identity_h, float dt);
void KF_Predict(KF_Def* kf_t);
void KF_Update(KF_Def* kf_t);

#endif
