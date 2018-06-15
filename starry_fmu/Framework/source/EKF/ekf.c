/*
 * File      : ekf.c
 *
 * Change Logs:
 * Date           Author       Notes
 * 2017-08-09     zoujiachi    first version.
 */
 
//#include <rthw.h>
//#include <rtdevice.h>
//#include <rtthread.h>
#include <math.h>
#include "global.h"
#include "ekf.h"
#include "console.h"

//#define ZETA			0.15f
#define ZETA			0.1f

//static const float PI = 3.141592658;
const float R0 = 6371393.0f;

//#define Deg2Rad(a)		(a*PI/180.0f)

Mat temp_mat1, temp_mat2, temp_mat3;
Mat temp_mat4;
static float Tx = 180.0f*1e7/(3.14159265359f*R0);

/* accel bias of z axis in NED frame */
static float _az_bias = 0.0f;
uint8_t _ekf_stable_flag = 0;

void EKF6_Init(EKF_Def* ekf_t, float dT, HOME_Pos home_pos)
{
	int state_num = 6;
	MatCreate(&ekf_t->x, state_num, 1);
	MatCreate(&ekf_t->pre_x, state_num, 1);
	MatCreate(&ekf_t->u, state_num, 1);
	MatCreate(&ekf_t->F, state_num, state_num);
	MatCreate(&ekf_t->P, state_num, state_num);
	MatCreate(&ekf_t->pre_P, state_num, state_num);
	MatCreate(&ekf_t->Q, state_num, state_num);
	MatCreate(&ekf_t->H, state_num, state_num);
	MatCreate(&ekf_t->R, state_num, state_num);
	MatCreate(&ekf_t->z, state_num, 1);
	MatCreate(&ekf_t->y, state_num, 1);
	MatCreate(&ekf_t->S, state_num, state_num);
	MatCreate(&ekf_t->K, state_num, state_num);
	
	MatCreate(&temp_mat1, state_num, state_num);
	MatCreate(&temp_mat2, state_num, state_num);
	MatCreate(&temp_mat3, state_num, state_num);
	MatCreate(&temp_mat4, state_num, 1);
	
	ekf_t->T = dT;
	
	/* set initial state to home */
	ekf_t->x.element[0][0] = (float)home_pos.lat;
	ekf_t->x.element[1][0] = (float)home_pos.lon;
	ekf_t->x.element[2][0] = home_pos.alt;
	ekf_t->x.element[3][0] = 0.0f;
	ekf_t->x.element[4][0] = 0.0f;
	ekf_t->x.element[5][0] = 0.0f;
	
	/* init F and H */
	MatEye(&ekf_t->F);
	ekf_t->F.element[0][3] = Tx*ekf_t->T;
	ekf_t->F.element[2][5] = -ekf_t->T;
	MatEye(&ekf_t->H);
	
	/* Q = (B*[cov_aN cov_aE cov_aD]') * (B*[cov_aN cov_aE cov_aD]')' */
	MatZeros(&ekf_t->Q);
	float Ty = Tx/arm_cos_f32(Deg2Rad(ekf_t->x.element[0][0]*1e-7));
	ekf_t->Q.element[0][0] = (EKF_QVN_SIGMA*dT*Tx)*(EKF_QVN_SIGMA*dT*Tx);
	ekf_t->Q.element[1][1] = (EKF_QVE_SIGMA*dT*Ty)*(EKF_QVE_SIGMA*dT*Ty);
	ekf_t->Q.element[2][2] = (EKF_QVD_SIGMA*dT)*(EKF_QVD_SIGMA*dT);
	ekf_t->Q.element[3][3] = (EKF_QAN_SIGMA*dT)*(EKF_QAN_SIGMA*dT);
	ekf_t->Q.element[4][4] = (EKF_QAE_SIGMA*dT)*(EKF_QAE_SIGMA*dT);
	ekf_t->Q.element[5][5] = (EKF_QAD_SIGMA*dT)*(EKF_QAD_SIGMA*dT);
	
	/* R = (H*[cov_L cov_l cov_h cov_vN cov_vE cov_vD]')*(H*[cov_L cov_l cov_h cov_vN cov_vE cov_vD]')' */
	float cov_L = 300;
	float cov_l = 300;
	//float cov_h = 0.18;
	float cov_h = 0.3f;
	float cov_vN = 0.1;
	float cov_vE = 0.1;
	//float cov_vD = 0.3f;
	float cov_vD = 0.06f;
	MatZeros(&ekf_t->R);
	ekf_t->R.element[0][0] = EKF_RLA_SIGMA*EKF_RLA_SIGMA;
	ekf_t->R.element[1][1] = EKF_RLO_SIGMA*EKF_RLO_SIGMA;
	ekf_t->R.element[2][2] = EKF_RH_SIGMA*EKF_RH_SIGMA;
	ekf_t->R.element[3][3] = EKF_RVN_SIGMA*EKF_RVN_SIGMA;
	ekf_t->R.element[4][4] = EKF_RVE_SIGMA*EKF_RVE_SIGMA;
	ekf_t->R.element[5][5] = EKF_RVD_SIGMA*EKF_RVD_SIGMA;
	
	/* Init P = R */
	MatCopy(&ekf_t->R, &ekf_t->P);
	
	MatZeros(&ekf_t->u);
	MatZeros(&ekf_t->z);
}

void update_F(EKF_Def* ekf_t)
{
	float rad = Deg2Rad(ekf_t->x.element[0][0]*1e-7);
	float cos_val = arm_cos_f32(rad);
	
	ekf_t->F.element[1][0] = Tx*1e-7*ekf_t->T*ekf_t->x.element[4][0]*arm_sin_f32(rad)/(cos_val*cos_val);
	ekf_t->F.element[1][4] = Tx*ekf_t->T/cos_val;
}

void EKF6_Update(EKF_Def* ekf_t)
{
	float Ty = Tx/arm_cos_f32(Deg2Rad(ekf_t->x.element[0][0]*1e-7));

	/* states: [L l h vn ve vd] */
	
	/* calculate F */
	update_F(ekf_t);
	
	/* Predict */
	/* x_k|k-1 = f(x_k-1|k-1, u_k-1) */
	ekf_t->pre_x.element[0][0] = ekf_t->x.element[0][0] + ekf_t->x.element[3][0]*Tx*ekf_t->T;
	ekf_t->pre_x.element[1][0] = ekf_t->x.element[1][0] + ekf_t->x.element[4][0]*Ty*ekf_t->T;
	ekf_t->pre_x.element[2][0] = ekf_t->x.element[2][0] - ekf_t->x.element[5][0]*ekf_t->T;
	ekf_t->pre_x.element[3][0] = ekf_t->x.element[3][0] + ekf_t->u.element[3][0]*ekf_t->T;
	ekf_t->pre_x.element[4][0] = ekf_t->x.element[4][0] + ekf_t->u.element[4][0]*ekf_t->T;
	ekf_t->pre_x.element[5][0] = ekf_t->x.element[5][0] + ekf_t->u.element[5][0]*ekf_t->T;
	/* P_k|k-1 = F_k-1*P_k-1|k-1*(F_k-1)' + Q_k-1 */
	MatMul(MatMul(&ekf_t->F, &ekf_t->P, &temp_mat1), MatTrans(&ekf_t->F, &temp_mat2), &temp_mat3);
	MatAdd(&temp_mat3, &ekf_t->Q, &ekf_t->pre_P);
	
//	/* Update */
//	/* y_k = z_k - h(x_k_k-1) */
//	MatSub(&ekf_t->z, &ekf_t->pre_x, &ekf_t->y);
//	/* S_k = H_k*P_k|k-1*(H_k)' + R_k */
//	MatMul(MatMul(&ekf_t->H, &ekf_t->pre_P, &temp_mat1), MatTrans(&ekf_t->H, &temp_mat2), &temp_mat3);
//	MatAdd(&temp_mat3, &ekf_t->R, &ekf_t->S);
//	/* K_k = P_k|k-1*(H_k)'*(S_k)^-1 */
//	MatMul(MatMul(&ekf_t->pre_P, MatTrans(&ekf_t->H, &temp_mat3), &temp_mat1), MatInv(&ekf_t->S, &temp_mat2), &ekf_t->K);
//	/* x_k|k = x_k|k-1 + K_k*y_k */
//	MatAdd(&ekf_t->pre_x, MatMul(&ekf_t->K, &ekf_t->y, &temp_mat4), &ekf_t->x);
//	/* P_k|k = (I - K_k*H_k)*P_k|k-1 */
//	MatEye(&temp_mat1);
//	MatMul(MatSub(&temp_mat1, MatMul(&ekf_t->K, &ekf_t->H, &temp_mat2), &temp_mat3), &ekf_t->pre_P, &ekf_t->P);

	/* Update */
	/* H is identity Matrix */
	/* y_k = z_k - h(x_k_k-1) */
	MatSub(&ekf_t->z, &ekf_t->pre_x, &ekf_t->y);
	/* S_k = H_k*P_k|k-1*(H_k)' + R_k */
	MatAdd(&ekf_t->pre_P, &ekf_t->R, &ekf_t->S);
	/* K_k = P_k|k-1*(H_k)'*(S_k)^-1 */
	MatMul(&ekf_t->pre_P, MatInv(&ekf_t->S, &temp_mat2), &ekf_t->K);
	/* x_k|k = x_k|k-1 + K_k*y_k */
	MatAdd(&ekf_t->pre_x, MatMul(&ekf_t->K, &ekf_t->y, &temp_mat4), &ekf_t->x);
	/* P_k|k = (I - K_k*H_k)*P_k|k-1 */
	MatSub(&ekf_t->pre_P, MatMul(&ekf_t->K, &ekf_t->pre_P, &temp_mat1), &ekf_t->P);
}

void EKF2_Init(EKF_Def* ekf_t, float qh, float qv, float rh, float rv, float dT)
{
	int state_num = 2;
	MatCreate(&ekf_t->x, state_num, 1);
	MatCreate(&ekf_t->pre_x, state_num, 1);
	MatCreate(&ekf_t->last_x, state_num, 1);
	MatCreate(&ekf_t->u, state_num, 1);
	MatCreate(&ekf_t->F, state_num, state_num);
	MatCreate(&ekf_t->P, state_num, state_num);
	MatCreate(&ekf_t->pre_P, state_num, state_num);
	MatCreate(&ekf_t->Q, state_num, state_num);
	MatCreate(&ekf_t->H, state_num, state_num);
	MatCreate(&ekf_t->R, state_num, state_num);
	MatCreate(&ekf_t->z, state_num, 1);
	MatCreate(&ekf_t->y, state_num, 1);
	MatCreate(&ekf_t->S, state_num, state_num);
	MatCreate(&ekf_t->K, state_num, state_num);
	
	MatCreate(&temp_mat1, state_num, state_num);
	MatCreate(&temp_mat2, state_num, state_num);
	MatCreate(&temp_mat3, state_num, state_num);
	MatCreate(&temp_mat4, state_num, 1);
	
	ekf_t->T = dT;
	
	/* init F and H */
	MatEye(&ekf_t->F);
	ekf_t->F.element[0][1] = -ekf_t->T;
	MatEye(&ekf_t->H);
	
//	float sigma_pre_v = 0.12f;
//	float sigma_acc = 0.07f;
//	float sigma_alt = 0.05f;	
//	float sigma_v = 0.05f;

//	float sigma_pre_v = 0.1f;
//	float sigma_acc = 0.1f;
//	float sigma_alt = 0.05f;	
//	float sigma_v = 0.05f;

//	float sigma_pre_v = 0.1f;
//	float sigma_acc = 0.1f;
//	float sigma_alt = 0.1f;	
//	float sigma_v = 0.1f;

//	float sigma_pre_v = 0.05f;
//	float sigma_acc = 0.1f;
//	float sigma_alt = 0.15f;	
//	float sigma_v = 0.1f;

	//for barometer
//	float sigma_pre_v = 0.1f;
//	float sigma_acc = 0.15f;
//	float sigma_alt = 0.1f;	
//	float sigma_v = 0.1f;
	//for lidar
//	float sigma_pre_v = 0.5f;
//	float sigma_acc = 0.1f;
//	float sigma_alt = 0.1f;	
//	float sigma_v = 0.1f;
	
	/* Q = (B*[cov_aN cov_aE cov_aD]') * (B*[cov_aN cov_aE cov_aD]')' */
	MatZeros(&ekf_t->Q);
	ekf_t->Q.element[0][0] = dT*dT*qh*qh;
	ekf_t->Q.element[1][1] = dT*dT*qv*qv;
	
	/* R = (H*[cov_L cov_l cov_h cov_vN cov_vE cov_vD]')*(H*[cov_L cov_l cov_h cov_vN cov_vE cov_vD]')' */
	MatZeros(&ekf_t->R);
	ekf_t->R.element[0][0] = rh*rh;
	ekf_t->R.element[1][1] = rv*rv;
	
	/* Init P = R */
	MatCopy(&ekf_t->R, &ekf_t->P);
	
	MatZeros(&ekf_t->u);
	MatZeros(&ekf_t->z);
	
	ekf_t->ite_cnt = 0;
	_ekf_stable_flag = 0;
	_az_bias = 0.0f;
	
//	Console.print("F:\n");
//	MatDump(&ekf_t->F);
//	Console.print("H:\n");
//	MatDump(&ekf_t->H);
//	Console.print("Q:\n");
//	MatDump(&ekf_t->Q);
//	Console.print("R:\n");
//	MatDump(&ekf_t->R);
}

void EKF2_Predict(EKF_Def* ekf_t)
{
	/* add bias */
	ekf_t->u.element[1][0] += _az_bias;
	
	/* Predict */
	/* x_k|k-1 = f(x_k-1|k-1, u_k-1) */
	ekf_t->x.element[0][0] = ekf_t->x.element[0][0] - ekf_t->x.element[1][0]*ekf_t->T;
	ekf_t->x.element[1][0] = ekf_t->x.element[1][0] + ekf_t->u.element[1][0]*ekf_t->T;
	/* P_k|k-1 = F_k-1*P_k-1|k-1*(F_k-1)' + Q_k-1 */
	MatMul(MatMul(&ekf_t->F, &ekf_t->P, &temp_mat1), MatTrans(&ekf_t->F, &temp_mat2), &temp_mat3);
	MatAdd(&temp_mat3, &ekf_t->Q, &ekf_t->P);
	
	ekf_t->ite_cnt++;
}

float EKF2_GetZ_Bias(void)
{
	return _az_bias;
}

void EKF2_Update(EKF_Def* ekf_t)
{
	/* add bias */
	ekf_t->u.element[1][0] += _az_bias;
	
	/* Predict */
	/* x_k|k-1 = f(x_k-1|k-1, u_k-1) */
	ekf_t->pre_x.element[0][0] = ekf_t->x.element[0][0] - ekf_t->x.element[1][0]*ekf_t->T;
	ekf_t->pre_x.element[1][0] = ekf_t->x.element[1][0] + ekf_t->u.element[1][0]*ekf_t->T;
	/* P_k|k-1 = F_k-1*P_k-1|k-1*(F_k-1)' + Q_k-1 */
	MatMul(MatMul(&ekf_t->F, &ekf_t->P, &temp_mat1), MatTrans(&ekf_t->F, &temp_mat2), &temp_mat3);
	MatAdd(&temp_mat3, &ekf_t->Q, &ekf_t->pre_P);

	/* Update */
	/* H is identity Matrix */
	/* y_k = z_k - h(x_k_k-1) */
	//MatSub(&ekf_t->z, &ekf_t->pre_x, &ekf_t->y);
	ekf_t->last_x.element[1][0] = ekf_t->pre_x.element[1][0];
	MatSub(&ekf_t->z, &ekf_t->last_x, &ekf_t->y);
	/* S_k = H_k*P_k|k-1*(H_k)' + R_k */
	MatAdd(&ekf_t->pre_P, &ekf_t->R, &ekf_t->S);
	/* K_k = P_k|k-1*(H_k)'*(S_k)^-1 */
	MatMul(&ekf_t->pre_P, MatInv(&ekf_t->S, &temp_mat2), &ekf_t->K);
	/* x_k|k = x_k|k-1 + K_k*y_k */
	MatAdd(&ekf_t->pre_x, MatMul(&ekf_t->K, &ekf_t->y, &temp_mat4), &ekf_t->x);
	/* P_k|k = (I - K_k*H_k)*P_k|k-1 */
	MatSub(&ekf_t->pre_P, MatMul(&ekf_t->K, &ekf_t->pre_P, &temp_mat1), &ekf_t->P);
	
	MatCopy(&ekf_t->x, &ekf_t->last_x);
	
	/* we only update z accel bias while ekf is stable */
	if(ekf_t->ite_cnt > 100){
		_ekf_stable_flag = 1;
	}
	
	/* calculate z accel bias */
	if(_ekf_stable_flag){
		float da = temp_mat4.element[1][0]/ekf_t->T;	//the update period of barometer is 20ms
		_az_bias += ZETA*da*ekf_t->T;
	}
	/* update iteration count */
	ekf_t->ite_cnt++;
}


