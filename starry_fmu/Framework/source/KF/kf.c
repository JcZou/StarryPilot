/*
 * File      : kf.c
 *
 * Change Logs:
 * Date           Author       Notes
 * 2017-08-09     zoujiachi    first version.
 */

#include <math.h>
#include "global.h"
#include "kf.h"
#include "console.h"
#include "fifo.h"

void KF_Create(KF_Def* kf_t, int x_dim, int u_dim)
{
	MatCreate(&kf_t->x, x_dim, 1);
	MatCreate(&kf_t->u, u_dim, 1);
	MatCreate(&kf_t->z, x_dim, 1);
	
	MatCreate(&kf_t->F, x_dim, x_dim);
	MatCreate(&kf_t->B, x_dim, u_dim);
	MatCreate(&kf_t->H, x_dim, x_dim);
	MatCreate(&kf_t->P, x_dim, x_dim);
	MatCreate(&kf_t->Q, x_dim, x_dim);
	MatCreate(&kf_t->R, x_dim, x_dim);
	
	MatCreate(&kf_t->y, x_dim, 1);
	MatCreate(&kf_t->S, x_dim, x_dim);
	MatCreate(&kf_t->K, x_dim, x_dim);
	
	MatCreate(&kf_t->M_nn_1, x_dim, x_dim);
	MatCreate(&kf_t->M_nn_2, x_dim, x_dim);
	MatCreate(&kf_t->M_nn_3, x_dim, x_dim);
	MatCreate(&kf_t->M_nn_4, x_dim, x_dim);
	MatCreate(&kf_t->M_n1_1, x_dim, 1);
	MatCreate(&kf_t->M_n1_2, x_dim, 1);
	MatCreate(&kf_t->I, x_dim, x_dim);
}

void KF_Init(KF_Def* kf_t, float *F_val, float *B_val, float *H_val, float *P_val, float *Q_val, float *R_val, float *x_val, bool identity_h, float dt)
{
	MatSetVal(&kf_t->F, F_val);
	MatSetVal(&kf_t->B, B_val);
	MatSetVal(&kf_t->H, H_val);
	MatSetVal(&kf_t->P, P_val);
	MatSetVal(&kf_t->Q, Q_val);
	MatSetVal(&kf_t->R, R_val);
	MatSetVal(&kf_t->x, x_val);
	
	MatEye(&kf_t->I);
	MatZeros(&kf_t->u);
	MatZeros(&kf_t->z);
	
	kf_t->dT = dt;
	kf_t->identity_h = identity_h;
}

void KF_Predict(KF_Def* kf_t)
{
	/* Predict Phase */
	// x(k|k-1) = F(k)*x(k-1|k-1)+B(k)u(k)
	MatAdd(MatMul(&kf_t->F, &kf_t->x, &kf_t->M_n1_1), MatMul(&kf_t->B, &kf_t->u, &kf_t->M_n1_2), &kf_t->x);
	// P(k|k-1) = F(k)*P(k-1|k-1)*F(k)' + Q(k)
	MatMul(MatMul(&kf_t->F, &kf_t->P, &kf_t->M_nn_1), MatTrans(&kf_t->F, &kf_t->M_nn_2), &kf_t->M_nn_3);
	MatAdd(&kf_t->M_nn_3, &kf_t->Q, &kf_t->P);
}

void KF_Update(KF_Def* kf_t)
{
	/* Update Phase */
	if(kf_t->identity_h){
		// y(k) = z(k) - H(k)*x(k|k-1)
		MatSub(&kf_t->z, &kf_t->x, &kf_t->y);
		// S(k) = H(k)*P(k|k-1)*H(k)' + R(k)
		MatAdd(&kf_t->P, &kf_t->R, &kf_t->S);
		// K(k) = P(k|k-1)*H(k)'*S(k)^-1
		MatInv(&kf_t->S, &kf_t->M_nn_1);
		MatMul(&kf_t->P, &kf_t->M_nn_1, &kf_t->K);
		// x(k|k) = x(k|k-1) + K(k)*y(k)
		MatAdd(&kf_t->x, MatMul(&kf_t->K, &kf_t->y, &kf_t->M_n1_1), &kf_t->x);
#ifdef USE_OPT_KF_GAIN
		// P(k|k) = (I - K(k)*H(k))*P(k|k-1)
		MatSub(&kf_t->P, MatMul(&kf_t->K, &kf_t->P, &kf_t->M_nn_1), &kf_t->P);
#else
		// P(k|k) = (I - K(k)*H(k))*P(k|k-1)*(I - K(k)*H(k))'+K(k)*R(k)*K(k)'
		MatSub(&kf_t->I, &kf_t->K, &kf_t->M_nn_1);
		MatTrans(&kf_t->M_nn_1, &kf_t->M_nn_2);
		MatMul(MatMul(&kf_t->M_nn_1, &kf_t->P, &kf_t->M_nn_3), &kf_t->M_nn_2, &kf_t->M_nn_1);
		MatMul(MatMul(&kf_t->K, &kf_t->R, &kf_t->M_nn_2), MatTrans(&kf_t->K, &kf_t->M_nn_3), &kf_t->M_nn_4);
		MatAdd(&kf_t->M_nn_1, &kf_t->M_nn_4, &kf_t->P);
#endif
	}else{
		// y(k) = z(k) - H(k)*x(k|k-1)
		MatSub(&kf_t->z, MatMul(&kf_t->H, &kf_t->x, &kf_t->M_n1_1), &kf_t->y);
		// S(k) = H(k)*P(k|k-1)*H(k)' + R(k)
		MatMul(MatMul(&kf_t->H, &kf_t->P, &kf_t->M_nn_1), MatTrans(&kf_t->H, &kf_t->M_nn_2), &kf_t->M_nn_3);
		MatAdd(&kf_t->M_nn_3, &kf_t->R, &kf_t->S);
		// K(k) = P(k|k-1)*H(k)'*S(k)^-1
		MatMul(&kf_t->P, MatTrans(&kf_t->H, &kf_t->M_nn_1), &kf_t->M_nn_2);
		MatMul(&kf_t->M_nn_2, MatInv(&kf_t->S, &kf_t->M_nn_1), &kf_t->K);
		// x(k|k) = x(k|k-1) + K(k)*y(k)
		MatAdd(&kf_t->x, MatMul(&kf_t->K, &kf_t->y, &kf_t->M_n1_1), &kf_t->x);
#ifdef USE_OPT_KF_GAIN
		// P(k|k) = (I - K(k)*H(k))*P(k|k-1)
		MatMul(MatMul(&kf_t->K, &kf_t->H, &kf_t->M_nn_1), &kf_t->P, &kf_t->M_nn_2);
		MatSub(&kf_t->P, &kf_t->M_nn_2, &kf_t->P);
#else
		// P(k|k) = (I - K(k)*H(k))*P(k|k-1)*(I - K(k)*H(k))'+K(k)*R(k)*K(k)'
		MatSub(&kf_t->I, MatMul(&kf_t->K, &kf_t->H, &kf_t->M_nn_1), &kf_t->M_nn_1);
		MatTrans(&kf_t->M_nn_1, &kf_t->M_nn_2);
		MatMul(MatMul(&kf_t->M_nn_1, &kf_t->P, &kf_t->M_nn_3), &kf_t->M_nn_2, &kf_t->M_nn_1);
		MatMul(MatMul(&kf_t->K, &kf_t->R, &kf_t->M_nn_2), MatTrans(&kf_t->K, &kf_t->M_nn_3), &kf_t->M_nn_4);
		MatAdd(&kf_t->M_nn_1, &kf_t->M_nn_4, &kf_t->P);
#endif
	}
}
