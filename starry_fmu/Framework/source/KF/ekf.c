#include <math.h>
#include <stdlib.h>
#include "ekf.h"
#include "sensor_manager.h"
#include "console.h"
#include "AHRS.h"

#define NUM_X	14
#define NUM_U	9
#define NUM_Z	8
#define NUM_W	10

#define MAX(x,y) (x > y ? x : y)

// estimate covariance
#define q_gx			0.0025
#define q_gy			0.0025
#define q_gz			0.0025
#define q_ax			0.1
#define q_ay			0.1
#define q_az			0.1
//#define q_gx_bias		0.0005
//#define q_gy_bias		0.0005
//#define q_gz_bias		0.0005
//#define q_az_bias		0.0025
#define q_gx_bias		0.002
#define q_gy_bias		0.002
#define q_gz_bias		0.002
#define q_az_bias		0.0025
// observe covariance
#define r_x				0.02
#define r_y				0.02
#define r_z				0.04
//#define r_ax			0.0015
//#define r_ay			0.0015
//#define r_az			0.0015
//#define r_mx			0.0012
//#define r_my			0.0012
#define r_ax			0.15
#define r_ay			0.15
#define r_az			0.15
#define r_mx			0.1
#define r_my			0.1

static float32_t  X_Data[NUM_X];
static float32_t  U_Data[NUM_U];
static float32_t  Z_Data[NUM_Z];

static float32_t  F_Data[NUM_X*NUM_X];
static float32_t  H_Data[NUM_Z*NUM_X];
static float32_t  G_Data[NUM_X*NUM_W];
static float32_t  P_Data[NUM_X*NUM_X];
static float32_t  Q_Data[NUM_W*NUM_W];
static float32_t  R_Data[NUM_Z*NUM_Z];

static float32_t  Y_Data[NUM_Z];
static float32_t  S_Data[NUM_Z*NUM_Z];
static float32_t  K_Data[NUM_X*NUM_Z];

static float32_t IFT_Data[NUM_X*NUM_X];
static float32_t IFTT_Data[NUM_X*NUM_X];
static float32_t IFTP_Data[NUM_X*NUM_X];
static float32_t IFTPIFTT_Data[NUM_X*NUM_X];
static float32_t GQ_Data[NUM_X*NUM_W];
static float32_t GT_Data[NUM_W*NUM_X];
static float32_t GQGT_Data[NUM_X*NUM_X];

static float32_t HT_Data[NUM_X*NUM_Z];
static float32_t PHT_Data[NUM_X*NUM_Z];
static float32_t HPHT_Data[NUM_Z*NUM_Z];
static float32_t INV_S_Data[NUM_Z*NUM_Z];
static float32_t KY_Data[NUM_X];
static float32_t KH_Data[NUM_X*NUM_X];
static float32_t KHP_Data[NUM_X*NUM_X];


void mat_fill_f32(arm_matrix_instance_f32* mat, float32_t val)
{
	for(int n = 0 ; n < mat->numRows*mat->numCols ; n++){
			mat->pData[n] = val;
	}
}

void mat_fill_identity(arm_matrix_instance_f32* mat)
{
	int n = 0;
	for(int r = 0 ; r < mat->numRows ; r++){
		for(int c = 0 ; c < mat->numCols ; c++){
			if(r == c)
				mat->pData[n] = 1.0f;
			else
				mat->pData[n] = 0.0f;
			
			n++;
		}
	}
}

///////////////////////////////////////

uint8_t EKF14_Init(EKF_Def* ekf_t, float32_t dT)
{
	ekf_t->dT = dT;
	
	arm_mat_init_f32(&ekf_t->X, NUM_X, 1, X_Data);
	arm_mat_init_f32(&ekf_t->U, NUM_U, 1, U_Data);
	arm_mat_init_f32(&ekf_t->Z, NUM_Z, 1, Z_Data);
	
	arm_mat_init_f32(&ekf_t->F, NUM_X, NUM_X, F_Data);
	arm_mat_init_f32(&ekf_t->H, NUM_Z, NUM_X, H_Data);
	arm_mat_init_f32(&ekf_t->G, NUM_X, NUM_W, G_Data);
	arm_mat_init_f32(&ekf_t->P, NUM_X, NUM_X, P_Data);
	arm_mat_init_f32(&ekf_t->Q, NUM_W, NUM_W, Q_Data);
	arm_mat_init_f32(&ekf_t->R, NUM_Z, NUM_Z, R_Data);
	mat_fill_f32(&ekf_t->F, 0.0f);
	mat_fill_f32(&ekf_t->H, 0.0f);
	mat_fill_f32(&ekf_t->G, 0.0f);
	mat_fill_f32(&ekf_t->P, 0.0f);
	mat_fill_f32(&ekf_t->Q, 0.0f);
	mat_fill_f32(&ekf_t->R, 0.0f);
	
	arm_mat_init_f32(&ekf_t->Y, NUM_Z, 1, Y_Data);
	arm_mat_init_f32(&ekf_t->S, NUM_Z, NUM_Z, S_Data);
	arm_mat_init_f32(&ekf_t->K, NUM_X, NUM_Z, K_Data);
	
	MAT_ELEMENT(ekf_t->Q, 0, 0) = q_gx*q_gx;
	MAT_ELEMENT(ekf_t->Q, 1, 1) = q_gy*q_gy;
	MAT_ELEMENT(ekf_t->Q, 2, 2) = q_gz*q_gz;
	MAT_ELEMENT(ekf_t->Q, 3, 3) = q_ax*q_ax;
	MAT_ELEMENT(ekf_t->Q, 4, 4) = q_ay*q_ay;
	MAT_ELEMENT(ekf_t->Q, 5, 5) = q_az*q_az;
	MAT_ELEMENT(ekf_t->Q, 6, 6) = q_gx_bias*q_gx_bias;
	MAT_ELEMENT(ekf_t->Q, 7, 7) = q_gy_bias*q_gy_bias;
	MAT_ELEMENT(ekf_t->Q, 8, 8) = q_gz_bias*q_gz_bias;
	MAT_ELEMENT(ekf_t->Q, 9, 9) = q_az_bias*q_az_bias;
	
	MAT_ELEMENT(ekf_t->R, 0, 0) = r_x*r_x;
	MAT_ELEMENT(ekf_t->R, 1, 1) = r_y*r_y;
	MAT_ELEMENT(ekf_t->R, 2, 2) = r_z*r_z;
	MAT_ELEMENT(ekf_t->R, 3, 3) = r_ax*r_ax;
	MAT_ELEMENT(ekf_t->R, 4, 4) = r_ay*r_ay;
	MAT_ELEMENT(ekf_t->R, 5, 5) = r_az*r_az;
	MAT_ELEMENT(ekf_t->R, 6, 6) = r_mx*r_mx;
	MAT_ELEMENT(ekf_t->R, 7, 7) = r_my*r_my;
	
	arm_mat_init_f32(&ekf_t->IFT, NUM_X, NUM_X, IFT_Data);
	arm_mat_init_f32(&ekf_t->IFTT, NUM_X, NUM_X, IFTT_Data);
	arm_mat_init_f32(&ekf_t->IFTP, NUM_X, NUM_X, IFTP_Data);
	arm_mat_init_f32(&ekf_t->IFTPIFTT, NUM_X, NUM_X, IFTPIFTT_Data);
	arm_mat_init_f32(&ekf_t->GQ, NUM_X, NUM_W, GQ_Data);
	arm_mat_init_f32(&ekf_t->GT, NUM_W, NUM_X, GT_Data);
	arm_mat_init_f32(&ekf_t->GQGT, NUM_X, NUM_X, GQGT_Data);
	
	arm_mat_init_f32(&ekf_t->HT, NUM_X, NUM_Z, HT_Data);
	arm_mat_init_f32(&ekf_t->PHT, NUM_X, NUM_Z, PHT_Data);
	arm_mat_init_f32(&ekf_t->HPHT, NUM_Z, NUM_Z, HPHT_Data);
	arm_mat_init_f32(&ekf_t->INV_S, NUM_Z, NUM_Z, INV_S_Data);
	arm_mat_init_f32(&ekf_t->KY, NUM_X, 1, KY_Data);
	arm_mat_init_f32(&ekf_t->KH, NUM_X, NUM_X, KH_Data);
	arm_mat_init_f32(&ekf_t->KHP, NUM_X, NUM_X, KHP_Data);

	EKF14_Reset(ekf_t);
	
	return 0;
}

void EKF14_Reset(EKF_Def* ekf_t)
{
	mat_fill_f32(&ekf_t->P, 0.0f);
	MAT_ELEMENT(ekf_t->P, 0, 0) = 1.0f;
	MAT_ELEMENT(ekf_t->P, 1, 1) = 1.0f;
	MAT_ELEMENT(ekf_t->P, 2, 2) = 1.0f;
	MAT_ELEMENT(ekf_t->P, 3, 3) = 0.2f;
	MAT_ELEMENT(ekf_t->P, 4, 4) = 0.2f;
	MAT_ELEMENT(ekf_t->P, 5, 5) = 0.2f;
	MAT_ELEMENT(ekf_t->P, 6, 6) = 1e-5;
	MAT_ELEMENT(ekf_t->P, 7, 7) = 1e-5;
	MAT_ELEMENT(ekf_t->P, 8, 8) = 1e-5;
	MAT_ELEMENT(ekf_t->P, 9, 9) = 1e-5;
	MAT_ELEMENT(ekf_t->P, 10, 10) = 1e-9;
	MAT_ELEMENT(ekf_t->P, 11, 11) = 1e-9;
	MAT_ELEMENT(ekf_t->P, 12, 12) = 1e-9;
	MAT_ELEMENT(ekf_t->P, 13, 13) = 1e-8;
	
#ifdef HIL_SIMULATION
	float acc[3] = {0.0, 0.0, -9.8};
	float mag[3] = {1.0, 0.0, 0.0};
#else
	float acc[3], mag[3];
	sensor_acc_get_calibrated_data(acc);
	sensor_mag_get_calibrated_data(mag);
#endif
	quaternion att_q;
	AHRS_reset(&att_q, acc, mag);
	
	//TOOD, fill reset value
	MAT_ELEMENT(ekf_t->X, STATE_X, 0) = 0.0f;
	MAT_ELEMENT(ekf_t->X, STATE_Y, 0) = 0.0f;
	MAT_ELEMENT(ekf_t->X, STATE_Z, 0) = 0.0f;
	MAT_ELEMENT(ekf_t->X, STATE_VX, 0) = 0.0f;
	MAT_ELEMENT(ekf_t->X, STATE_VY, 0) = 0.0f;
	MAT_ELEMENT(ekf_t->X, STATE_VZ, 0) = 0.0f;
	MAT_ELEMENT(ekf_t->X, STATE_Q0, 0) = att_q.w;
	MAT_ELEMENT(ekf_t->X, STATE_Q1, 0) = att_q.x;
	MAT_ELEMENT(ekf_t->X, STATE_Q2, 0) = att_q.y;
	MAT_ELEMENT(ekf_t->X, STATE_Q3, 0) = att_q.z;
	MAT_ELEMENT(ekf_t->X, STATE_GX_BIAS, 0) = 0.0f;
	MAT_ELEMENT(ekf_t->X, STATE_GY_BIAS, 0) = 0.0f;
	MAT_ELEMENT(ekf_t->X, STATE_GZ_BIAS, 0) = 0.0f;
	MAT_ELEMENT(ekf_t->X, STATE_AZ_BIAS, 0) = 0.0f;
}

uint8_t EKF14_Prediction(EKF_Def* ekf_t)
{
	float32_t q0 = MAT_ELEMENT(ekf_t->X, STATE_Q0, 0);
	float32_t q1 = MAT_ELEMENT(ekf_t->X, STATE_Q1, 0);
	float32_t q2 = MAT_ELEMENT(ekf_t->X, STATE_Q2, 0);
	float32_t q3 = MAT_ELEMENT(ekf_t->X, STATE_Q3, 0);
	float32_t gx = MAT_ELEMENT(ekf_t->U, 0, 0) - MAT_ELEMENT(ekf_t->X, STATE_GX_BIAS, 0);
	float32_t gy = MAT_ELEMENT(ekf_t->U, 1, 0) - MAT_ELEMENT(ekf_t->X, STATE_GY_BIAS, 0);
	float32_t gz = MAT_ELEMENT(ekf_t->U, 2, 0) - MAT_ELEMENT(ekf_t->X, STATE_GZ_BIAS, 0);
	float32_t ax = MAT_ELEMENT(ekf_t->U, 3, 0);
	float32_t ay = MAT_ELEMENT(ekf_t->U, 4, 0);
	float32_t az = MAT_ELEMENT(ekf_t->U, 5, 0) - MAT_ELEMENT(ekf_t->X, STATE_AZ_BIAS, 0);
//	float32_t gx = MAT_ELEMENT(ekf_t->U, 0, 0);
//	float32_t gy = MAT_ELEMENT(ekf_t->U, 1, 0);
//	float32_t gz = MAT_ELEMENT(ekf_t->U, 2, 0);
//	float32_t ax = MAT_ELEMENT(ekf_t->U, 3, 0);
//	float32_t ay = MAT_ELEMENT(ekf_t->U, 4, 0);
//	float32_t az = MAT_ELEMENT(ekf_t->U, 5, 0);
	
	/* calculate jocobbians of f(x,u) */
	// d(Xdot)/d(V)
	MAT_ELEMENT(ekf_t->F, 0, 3) = 1.0f;
	MAT_ELEMENT(ekf_t->F, 1, 4) = 1.0f;
	MAT_ELEMENT(ekf_t->F, 2, 5) = 1.0f;
	// d(Vdoot)/d(q)
	MAT_ELEMENT(ekf_t->F, 3, 6) = 2.0f * (q0 * ax - q3 * ay + q2 * az);
    MAT_ELEMENT(ekf_t->F, 3, 7) = 2.0f * (q1 * ax + q2 * ay + q3 * az);
    MAT_ELEMENT(ekf_t->F, 3, 8) = 2.0f * (-q2 * ax + q1 * ay + q0 * az);
    MAT_ELEMENT(ekf_t->F, 3, 9) = 2.0f * (-q3 * ax - q0 * ay + q1 * az);
    MAT_ELEMENT(ekf_t->F, 4, 6) = 2.0f * (q3 * ax + q0 * ay - q1 * az);
    MAT_ELEMENT(ekf_t->F, 4, 7) = 2.0f * (q2 * ax - q1 * ay - q0 * az);
    MAT_ELEMENT(ekf_t->F, 4, 8) = 2.0f * (q1 * ax + q2 * ay + q3 * az);
    MAT_ELEMENT(ekf_t->F, 4, 9) = 2.0f * (q0 * ax - q3 * ay + q2 * az);
    MAT_ELEMENT(ekf_t->F, 5, 6) = 2.0f * (-q2 * ax + q1 * ay + q0 * az);
    MAT_ELEMENT(ekf_t->F, 5, 7) = 2.0f * (q3 * ax + q0 * ay - q1 * az);
    MAT_ELEMENT(ekf_t->F, 5, 8) = 2.0f * (-q0 * ax + q3 * ay - q2 * az);
    MAT_ELEMENT(ekf_t->F, 5, 9) = 2.0f * (q1 * ax + q2 * ay + q3 * az);
	// d(Vdot)/d(acc_bias)
    MAT_ELEMENT(ekf_t->F, 3, 13) =  -2.0f * (q1 * q3 + q0 * q2);
	MAT_ELEMENT(ekf_t->F, 4, 13) =  2.0f * (-q2 * q3 + q0 * q1);
	MAT_ELEMENT(ekf_t->F, 5, 13) =  -q0 * q0 + q1 * q1 + q2 * q2 - q3 * q3;
	// d(qdot)/d(q)
	MAT_ELEMENT(ekf_t->F, 6, 6)  = 0.0f;
    MAT_ELEMENT(ekf_t->F, 6, 7)  = -gx / 2.0f;
    MAT_ELEMENT(ekf_t->F, 6, 8)  = -gy / 2.0f;
    MAT_ELEMENT(ekf_t->F, 6, 9)  = -gz / 2.0f;
    MAT_ELEMENT(ekf_t->F, 7, 6)  = gx / 2.0f;
	MAT_ELEMENT(ekf_t->F, 7, 7)  = 0.0f;
    MAT_ELEMENT(ekf_t->F, 7, 8)  = gz / 2.0f;
    MAT_ELEMENT(ekf_t->F, 7, 9)  = -gy / 2.0f;
    MAT_ELEMENT(ekf_t->F, 8, 6)  = gy / 2.0f;
    MAT_ELEMENT(ekf_t->F, 8, 7)  = -gz / 2.0f;
	MAT_ELEMENT(ekf_t->F, 8, 8)  = 0.0f;
    MAT_ELEMENT(ekf_t->F, 8, 9)  = gx / 2.0f;
    MAT_ELEMENT(ekf_t->F, 9, 6)  = gz / 2.0f;
    MAT_ELEMENT(ekf_t->F, 9, 7)  = gy / 2.0f;
    MAT_ELEMENT(ekf_t->F, 9, 8)  = -gx / 2.0f;
	MAT_ELEMENT(ekf_t->F, 9, 9)  = 0.0f;
	// d(qdot)/d(gyr_bias)
    MAT_ELEMENT(ekf_t->F, 6, 10) = q1 / 2.0f;
    MAT_ELEMENT(ekf_t->F, 6, 11) = q2 / 2.0f;
    MAT_ELEMENT(ekf_t->F, 6, 12) = q3 / 2.0f;
    MAT_ELEMENT(ekf_t->F, 7, 10) = -q0 / 2.0f;
    MAT_ELEMENT(ekf_t->F, 7, 11) = q3 / 2.0f;
    MAT_ELEMENT(ekf_t->F, 7, 12) = -q2 / 2.0f;
    MAT_ELEMENT(ekf_t->F, 8, 10) = -q3 / 2.0f;
    MAT_ELEMENT(ekf_t->F, 8, 11) = -q0 / 2.0f;
    MAT_ELEMENT(ekf_t->F, 8, 12) = q1 / 2.0f;
    MAT_ELEMENT(ekf_t->F, 9, 10) = q2 / 2.0f;
    MAT_ELEMENT(ekf_t->F, 9, 11) = -q1 / 2.0f;
    MAT_ELEMENT(ekf_t->F, 9, 12) = -q0 / 2.0f;
	
	// d(Vdot)/d(acc)
    MAT_ELEMENT(ekf_t->G, 3, 3)  = q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3;
    MAT_ELEMENT(ekf_t->G, 3, 4)  = 2.0f * (q1 * q2 - q0 * q3);
    MAT_ELEMENT(ekf_t->G, 3, 5)  = 2.0f * (q1 * q3 + q0 * q2);
    MAT_ELEMENT(ekf_t->G, 4, 3)  = 2.0f * (q1 * q2 + q0 * q3);
    MAT_ELEMENT(ekf_t->G, 4, 4)  = q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3;
    MAT_ELEMENT(ekf_t->G, 4, 5)  = 2.0f * (q2 * q3 - q0 * q1);
    MAT_ELEMENT(ekf_t->G, 5, 3)  = 2.0f * (q1 * q3 - q0 * q2);
    MAT_ELEMENT(ekf_t->G, 5, 4)  = 2.0f * (q2 * q3 + q0 * q1);
    MAT_ELEMENT(ekf_t->G, 5, 5)  = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
	// d(qdot)/d(gyr)
    MAT_ELEMENT(ekf_t->G, 6, 0)  = -q1 / 2.0f;
    MAT_ELEMENT(ekf_t->G, 6, 1)  = -q2 / 2.0f;
    MAT_ELEMENT(ekf_t->G, 6, 2)  = -q3 / 2.0f;
    MAT_ELEMENT(ekf_t->G, 7, 0)  = q0 / 2.0f;
    MAT_ELEMENT(ekf_t->G, 7, 1)  = -q3 / 2.0f;
    MAT_ELEMENT(ekf_t->G, 7, 2)  = q2 / 2.0f;
    MAT_ELEMENT(ekf_t->G, 8, 0)  = q3 / 2.0f;
    MAT_ELEMENT(ekf_t->G, 8, 1)  = q0 / 2.0f;
    MAT_ELEMENT(ekf_t->G, 8, 2)  = -q1 / 2.0f;
    MAT_ELEMENT(ekf_t->G, 9, 0)  = -q2 / 2.0f;
    MAT_ELEMENT(ekf_t->G, 9, 1)  = q1 / 2.0f;
    MAT_ELEMENT(ekf_t->G, 9, 2)  = q0 / 2.0f;
    // d(bias)/d(bias)
	MAT_ELEMENT(ekf_t->G, 10, 6) = 1.0f;
	MAT_ELEMENT(ekf_t->G, 11, 7) = 1.0f;
	MAT_ELEMENT(ekf_t->G, 12, 8) = 1.0f;
    MAT_ELEMENT(ekf_t->G, 13, 9) = 1.0f;
	
	/* X(k|k-1) = f(X(k-1|k-1), U(k-1)) */
	float32_t dotX[NUM_X];
	dotX[STATE_X]	= MAT_ELEMENT(ekf_t->X, STATE_VX, 0);
	dotX[STATE_Y]	= MAT_ELEMENT(ekf_t->X, STATE_VY, 0);
	dotX[STATE_Z]	= MAT_ELEMENT(ekf_t->X, STATE_VZ, 0);
	dotX[STATE_VX]	= (q0*q0+q1*q1-q2*q2-q3*q3)*ax+2.0f*(q1*q2-q0*q3)*ay+2.0f*(q1*q3+q0*q2)*az;
	dotX[STATE_VY]	= 2.0f*(q1*q2+q0*q3)*ax+(q0*q0-q1*q1+q2*q2-q3*q3)*ay+2.0f*(q2*q3-q0*q1)*az;
	dotX[STATE_VZ]	= 2.0f*(q1*q3-q0*q2)*ax+2.0f*(q2*q3+q0*q1)*ay+(q0*q0-q1*q1-q2*q2+q3*q3)*az + GRAVITY_MSS;
	dotX[STATE_Q0]	= (-q1*gx-q2*gy-q3*gz)*0.5f;
	dotX[STATE_Q1]	= (q0*gx-q3*gy+q2*gz)*0.5f;
	dotX[STATE_Q2]	= (q3*gx+q0*gy-q1*gz)*0.5f;
	dotX[STATE_Q3]	= (-q2*gx+q1*gy+q0*gz)*0.5f;
	dotX[STATE_GX_BIAS] = dotX[STATE_GY_BIAS] = dotX[STATE_GZ_BIAS] = dotX[STATE_AZ_BIAS] = 0.0f;
	
	if(ax == 0.0f && ay == 0.0f && az == 0.0f){
		// accel is not available, do not let vz to increase
		dotX[STATE_VZ] = 0.0f;
	}
	
	for(uint8_t n = 0 ; n < NUM_X ; n++){
		MAT_ELEMENT(ekf_t->X, n, 0) += ekf_t->dT * dotX[n];
	}
	
	// normalize quaternion
	q0 = MAT_ELEMENT(ekf_t->X, STATE_Q0, 0);
	q1 = MAT_ELEMENT(ekf_t->X, STATE_Q1, 0);
	q2 = MAT_ELEMENT(ekf_t->X, STATE_Q2, 0);
	q3 = MAT_ELEMENT(ekf_t->X, STATE_Q3, 0);
	float32_t inv_norm = 1.0f/sqrtf(q0*q0+q1*q1+q2*q2+q3*q3);
	MAT_ELEMENT(ekf_t->X, STATE_Q0, 0) *= inv_norm;
	MAT_ELEMENT(ekf_t->X, STATE_Q1, 0) *= inv_norm;
	MAT_ELEMENT(ekf_t->X, STATE_Q2, 0) *= inv_norm;
	MAT_ELEMENT(ekf_t->X, STATE_Q3, 0) *= inv_norm;
	
	arm_status res = ARM_MATH_SUCCESS;
	
	/* P(k|k-1) = (I+F(k)*T)*P(k-1|k-1)*(I+F(k)*T)' + T^2*G*Q(k)*G' */
	res |= arm_mat_scale_f32(&ekf_t->F, ekf_t->dT, &ekf_t->IFT);
	for(uint8_t n = 0 ; n < NUM_X ; n++){
		MAT_ELEMENT(ekf_t->IFT, n, n) += 1.0f;
	}
	res |= arm_mat_trans_f32(&ekf_t->IFT, &ekf_t->IFTT);
	res |= arm_mat_mult_f32(&ekf_t->IFT, &ekf_t->P, &ekf_t->IFTP);
	res |= arm_mat_mult_f32(&ekf_t->IFTP, &ekf_t->IFTT, &ekf_t->IFTPIFTT);
	
	res |= arm_mat_mult_f32(&ekf_t->G, &ekf_t->Q, &ekf_t->GQ);
	res |= arm_mat_trans_f32(&ekf_t->G, &ekf_t->GT);
	res |= arm_mat_mult_f32(&ekf_t->GQ, &ekf_t->GT, &ekf_t->GQGT);
	res |= arm_mat_scale_f32(&ekf_t->GQGT, ekf_t->dT*ekf_t->dT, &ekf_t->GQGT);
	
	res |= arm_mat_add_f32(&ekf_t->IFTPIFTT, &ekf_t->GQGT, &ekf_t->P);
	
//	if(res != ARM_MATH_SUCCESS){
//		Console.print("predict err:%d\n", res);
//	}
	
	return res==ARM_MATH_SUCCESS ? 1 : 0;
}

uint8_t EKF14_SerialPrediction(EKF_Def* ekf_t, uint32_t enable_bitmask)
{
	float32_t q0 = MAT_ELEMENT(ekf_t->X, STATE_Q0, 0);
	float32_t q1 = MAT_ELEMENT(ekf_t->X, STATE_Q1, 0);
	float32_t q2 = MAT_ELEMENT(ekf_t->X, STATE_Q2, 0);
	float32_t q3 = MAT_ELEMENT(ekf_t->X, STATE_Q3, 0);
	float32_t gx = MAT_ELEMENT(ekf_t->U, 0, 0) - MAT_ELEMENT(ekf_t->X, STATE_GX_BIAS, 0);
	float32_t gy = MAT_ELEMENT(ekf_t->U, 1, 0) - MAT_ELEMENT(ekf_t->X, STATE_GY_BIAS, 0);
	float32_t gz = MAT_ELEMENT(ekf_t->U, 2, 0) - MAT_ELEMENT(ekf_t->X, STATE_GZ_BIAS, 0);
	float32_t ax = MAT_ELEMENT(ekf_t->U, 3, 0);
	float32_t ay = MAT_ELEMENT(ekf_t->U, 4, 0);
	float32_t az = MAT_ELEMENT(ekf_t->U, 5, 0) - MAT_ELEMENT(ekf_t->X, STATE_AZ_BIAS, 0);
	
	/* calculate jocobbians of f(x,u) */
	if( (enable_bitmask & 0x03) == 0x03 ){
		// d(Xdot)/d(V)
		MAT_ELEMENT(ekf_t->F, 0, 3) = 1.0f;
		MAT_ELEMENT(ekf_t->F, 1, 4) = 1.0f;
		// d(Vdoot)/d(q)
		MAT_ELEMENT(ekf_t->F, 3, 6) = 2.0f * (q0 * ax - q3 * ay + q2 * az);
		MAT_ELEMENT(ekf_t->F, 3, 7) = 2.0f * (q1 * ax + q2 * ay + q3 * az);
		MAT_ELEMENT(ekf_t->F, 3, 8) = 2.0f * (-q2 * ax + q1 * ay + q0 * az);
		MAT_ELEMENT(ekf_t->F, 3, 9) = 2.0f * (-q3 * ax - q0 * ay + q1 * az);
		MAT_ELEMENT(ekf_t->F, 4, 6) = 2.0f * (q3 * ax + q0 * ay - q1 * az);
		MAT_ELEMENT(ekf_t->F, 4, 7) = 2.0f * (q2 * ax - q1 * ay - q0 * az);
		MAT_ELEMENT(ekf_t->F, 4, 8) = 2.0f * (q1 * ax + q2 * ay + q3 * az);
		MAT_ELEMENT(ekf_t->F, 4, 9) = 2.0f * (q0 * ax - q3 * ay + q2 * az);
		// d(Vdot)/d(acc_bias)
		MAT_ELEMENT(ekf_t->F, 3, 13) =  -2.0f * (q1 * q3 + q0 * q2);
		MAT_ELEMENT(ekf_t->F, 4, 13) =  2.0f * (-q2 * q3 + q0 * q1);
	}else{
		// d(Xdot)/d(V)
		MAT_ELEMENT(ekf_t->F, 0, 3) = 0.0f;
		MAT_ELEMENT(ekf_t->F, 1, 4) = 0.0f;
		// d(Vdoot)/d(q)
		MAT_ELEMENT(ekf_t->F, 3, 6) = 0.0f;
		MAT_ELEMENT(ekf_t->F, 3, 7) = 0.0f;
		MAT_ELEMENT(ekf_t->F, 3, 8) = 0.0f;
		MAT_ELEMENT(ekf_t->F, 3, 9) = 0.0f;
		MAT_ELEMENT(ekf_t->F, 4, 6) = 0.0f;
		MAT_ELEMENT(ekf_t->F, 4, 7) = 0.0f;
		MAT_ELEMENT(ekf_t->F, 4, 8) = 0.0f;
		MAT_ELEMENT(ekf_t->F, 4, 9) = 0.0f;
		// d(Vdot)/d(acc_bias)
		MAT_ELEMENT(ekf_t->F, 3, 13) =  0.0f;
		MAT_ELEMENT(ekf_t->F, 4, 13) =  0.0f;
	}
	if( (enable_bitmask & 0x04) == 0x04 ){
		// d(Xdot)/d(V)
		MAT_ELEMENT(ekf_t->F, 2, 5) = 1.0f;
		// d(Vdoot)/d(q)
		MAT_ELEMENT(ekf_t->F, 5, 6) = 2.0f * (-q2 * ax + q1 * ay + q0 * az);
		MAT_ELEMENT(ekf_t->F, 5, 7) = 2.0f * (q3 * ax + q0 * ay - q1 * az);
		MAT_ELEMENT(ekf_t->F, 5, 8) = 2.0f * (-q0 * ax + q3 * ay - q2 * az);
		MAT_ELEMENT(ekf_t->F, 5, 9) = 2.0f * (q1 * ax + q2 * ay + q3 * az);
		// d(Vdot)/d(acc_bias)
		MAT_ELEMENT(ekf_t->F, 5, 13) =  -q0 * q0 + q1 * q1 + q2 * q2 - q3 * q3;
	}else{
		// d(Xdot)/d(V)
		MAT_ELEMENT(ekf_t->F, 2, 5) = 0.0f;
		// d(Vdoot)/d(q)
		MAT_ELEMENT(ekf_t->F, 5, 6) = 0.0f;
		MAT_ELEMENT(ekf_t->F, 5, 7) = 0.0f;
		MAT_ELEMENT(ekf_t->F, 5, 8) = 0.0f;
		MAT_ELEMENT(ekf_t->F, 5, 9) = 0.0f;
		// d(Vdot)/d(acc_bias)
		MAT_ELEMENT(ekf_t->F, 5, 13) =  0.0f;
	}

	// d(qdot)/d(q)
	MAT_ELEMENT(ekf_t->F, 6, 6)  = 0.0f;
    MAT_ELEMENT(ekf_t->F, 6, 7)  = -gx / 2.0f;
    MAT_ELEMENT(ekf_t->F, 6, 8)  = -gy / 2.0f;
    MAT_ELEMENT(ekf_t->F, 6, 9)  = -gz / 2.0f;
    MAT_ELEMENT(ekf_t->F, 7, 6)  = gx / 2.0f;
	MAT_ELEMENT(ekf_t->F, 7, 7)  = 0.0f;
    MAT_ELEMENT(ekf_t->F, 7, 8)  = gz / 2.0f;
    MAT_ELEMENT(ekf_t->F, 7, 9)  = -gy / 2.0f;
    MAT_ELEMENT(ekf_t->F, 8, 6)  = gy / 2.0f;
    MAT_ELEMENT(ekf_t->F, 8, 7)  = -gz / 2.0f;
	MAT_ELEMENT(ekf_t->F, 8, 8)  = 0.0f;
    MAT_ELEMENT(ekf_t->F, 8, 9)  = gx / 2.0f;
    MAT_ELEMENT(ekf_t->F, 9, 6)  = gz / 2.0f;
    MAT_ELEMENT(ekf_t->F, 9, 7)  = gy / 2.0f;
    MAT_ELEMENT(ekf_t->F, 9, 8)  = -gx / 2.0f;
	MAT_ELEMENT(ekf_t->F, 9, 9)  = 0.0f;
	// d(qdot)/d(gyr_bias)
    MAT_ELEMENT(ekf_t->F, 6, 10) = q1 / 2.0f;
    MAT_ELEMENT(ekf_t->F, 6, 11) = q2 / 2.0f;
    MAT_ELEMENT(ekf_t->F, 6, 12) = q3 / 2.0f;
    MAT_ELEMENT(ekf_t->F, 7, 10) = -q0 / 2.0f;
    MAT_ELEMENT(ekf_t->F, 7, 11) = q3 / 2.0f;
    MAT_ELEMENT(ekf_t->F, 7, 12) = -q2 / 2.0f;
    MAT_ELEMENT(ekf_t->F, 8, 10) = -q3 / 2.0f;
    MAT_ELEMENT(ekf_t->F, 8, 11) = -q0 / 2.0f;
    MAT_ELEMENT(ekf_t->F, 8, 12) = q1 / 2.0f;
    MAT_ELEMENT(ekf_t->F, 9, 10) = q2 / 2.0f;
    MAT_ELEMENT(ekf_t->F, 9, 11) = -q1 / 2.0f;
    MAT_ELEMENT(ekf_t->F, 9, 12) = -q0 / 2.0f;
	
	// d(Vdot)/d(acc)
	if( (enable_bitmask & 0x03) == 0x03 ){
		MAT_ELEMENT(ekf_t->G, 3, 3)  = q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3;
		MAT_ELEMENT(ekf_t->G, 3, 4)  = 2.0f * (q1 * q2 - q0 * q3);
		MAT_ELEMENT(ekf_t->G, 3, 5)  = 2.0f * (q1 * q3 + q0 * q2);
		MAT_ELEMENT(ekf_t->G, 4, 3)  = 2.0f * (q1 * q2 + q0 * q3);
		MAT_ELEMENT(ekf_t->G, 4, 4)  = q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3;
		MAT_ELEMENT(ekf_t->G, 4, 5)  = 2.0f * (q2 * q3 - q0 * q1);
	}else{
		MAT_ELEMENT(ekf_t->G, 3, 3)  = 0.0f;
		MAT_ELEMENT(ekf_t->G, 3, 4)  = 0.0f;
		MAT_ELEMENT(ekf_t->G, 3, 5)  = 0.0f;
		MAT_ELEMENT(ekf_t->G, 4, 3)  = 0.0f;
		MAT_ELEMENT(ekf_t->G, 4, 4)  = 0.0f;
		MAT_ELEMENT(ekf_t->G, 4, 5)  = 0.0f;
	}
	if( (enable_bitmask & 0x04) == 0x04 ){
		MAT_ELEMENT(ekf_t->G, 5, 3)  = 2.0f * (q1 * q3 - q0 * q2);
		MAT_ELEMENT(ekf_t->G, 5, 4)  = 2.0f * (q2 * q3 + q0 * q1);
		MAT_ELEMENT(ekf_t->G, 5, 5)  = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
	}else{
		MAT_ELEMENT(ekf_t->G, 5, 3)  = 0.0f;
		MAT_ELEMENT(ekf_t->G, 5, 4)  = 0.0f;
		MAT_ELEMENT(ekf_t->G, 5, 5)  = 0.0f;
	}
	// d(qdot)/d(gyr)
    MAT_ELEMENT(ekf_t->G, 6, 0)  = -q1 / 2.0f;
    MAT_ELEMENT(ekf_t->G, 6, 1)  = -q2 / 2.0f;
    MAT_ELEMENT(ekf_t->G, 6, 2)  = -q3 / 2.0f;
    MAT_ELEMENT(ekf_t->G, 7, 0)  = q0 / 2.0f;
    MAT_ELEMENT(ekf_t->G, 7, 1)  = -q3 / 2.0f;
    MAT_ELEMENT(ekf_t->G, 7, 2)  = q2 / 2.0f;
    MAT_ELEMENT(ekf_t->G, 8, 0)  = q3 / 2.0f;
    MAT_ELEMENT(ekf_t->G, 8, 1)  = q0 / 2.0f;
    MAT_ELEMENT(ekf_t->G, 8, 2)  = -q1 / 2.0f;
    MAT_ELEMENT(ekf_t->G, 9, 0)  = -q2 / 2.0f;
    MAT_ELEMENT(ekf_t->G, 9, 1)  = q1 / 2.0f;
    MAT_ELEMENT(ekf_t->G, 9, 2)  = q0 / 2.0f;
    // d(bias)/d(bias)
	MAT_ELEMENT(ekf_t->G, 10, 6) = 1.0f;
	MAT_ELEMENT(ekf_t->G, 11, 7) = 1.0f;
	MAT_ELEMENT(ekf_t->G, 12, 8) = 1.0f;
    MAT_ELEMENT(ekf_t->G, 13, 9) = 1.0f;
	
	/* X(k|k-1) = f(X(k-1|k-1), U(k-1)) */
	float32_t dotX[NUM_X];
	if( (enable_bitmask & 0x03) == 0x03 ){
		dotX[STATE_X]	= MAT_ELEMENT(ekf_t->X, STATE_VX, 0);
		dotX[STATE_Y]	= MAT_ELEMENT(ekf_t->X, STATE_VY, 0);
		dotX[STATE_VX]	= (q0*q0+q1*q1-q2*q2-q3*q3)*ax+2.0f*(q1*q2-q0*q3)*ay+2.0f*(q1*q3+q0*q2)*az;
		dotX[STATE_VY]	= 2.0f*(q1*q2+q0*q3)*ax+(q0*q0-q1*q1+q2*q2-q3*q3)*ay+2.0f*(q2*q3-q0*q1)*az;
	}else{
		dotX[STATE_X]	= 0.0f;
		dotX[STATE_Y]	= 0.0f;
		dotX[STATE_VX]	= 0.0f;
		dotX[STATE_VY]	= 0.0f;
	}
	if( (enable_bitmask & 0x04) == 0x04 ){
		dotX[STATE_Z]	= MAT_ELEMENT(ekf_t->X, STATE_VZ, 0);
		dotX[STATE_VZ]	= 2.0f*(q1*q3-q0*q2)*ax+2.0f*(q2*q3+q0*q1)*ay+(q0*q0-q1*q1-q2*q2+q3*q3)*az + GRAVITY_MSS;
	}else{
		dotX[STATE_Z]	= 0.0f;
		dotX[STATE_VZ]	= 0.0f;
	}
	dotX[STATE_Q0]	= (-q1*gx-q2*gy-q3*gz)*0.5f;
	dotX[STATE_Q1]	= (q0*gx-q3*gy+q2*gz)*0.5f;
	dotX[STATE_Q2]	= (q3*gx+q0*gy-q1*gz)*0.5f;
	dotX[STATE_Q3]	= (-q2*gx+q1*gy+q0*gz)*0.5f;
	dotX[STATE_GX_BIAS] = dotX[STATE_GY_BIAS] = dotX[STATE_GZ_BIAS] = dotX[STATE_AZ_BIAS] = 0.0f;
	
	if(ax == 0.0f && ay == 0.0f && az == 0.0f){
		// accel is not available, do not let vz to increase
		dotX[STATE_VZ] = 0.0f;
	}

//	// for test
//	MAT_ELEMENT(ekf_t->Q, 3, 3) = MAT_ELEMENT(ekf_t->Q, 4, 4) = 0;
//	dotX[STATE_X] = 0;
//	dotX[STATE_Y] = 0;
//	dotX[STATE_VX] = 0;
//	dotX[STATE_VY] = 0;
	
	for(uint8_t n = 0 ; n < NUM_X ; n++){
		MAT_ELEMENT(ekf_t->X, n, 0) += ekf_t->dT * dotX[n];
	}
	
	// normalize quaternion
	q0 = MAT_ELEMENT(ekf_t->X, STATE_Q0, 0);
	q1 = MAT_ELEMENT(ekf_t->X, STATE_Q1, 0);
	q2 = MAT_ELEMENT(ekf_t->X, STATE_Q2, 0);
	q3 = MAT_ELEMENT(ekf_t->X, STATE_Q3, 0);
	float32_t inv_norm = 1.0f/sqrtf(q0*q0+q1*q1+q2*q2+q3*q3);
	MAT_ELEMENT(ekf_t->X, STATE_Q0, 0) *= inv_norm;
	MAT_ELEMENT(ekf_t->X, STATE_Q1, 0) *= inv_norm;
	MAT_ELEMENT(ekf_t->X, STATE_Q2, 0) *= inv_norm;
	MAT_ELEMENT(ekf_t->X, STATE_Q3, 0) *= inv_norm;
	
	arm_status res = ARM_MATH_SUCCESS;
	
	/* P(k|k-1) = (I+F(k)*T)*P(k-1|k-1)*(I+F(k)*T)' + T^2*G*Q(k)*G' */
	res |= arm_mat_scale_f32(&ekf_t->F, ekf_t->dT, &ekf_t->IFT);
	for(uint8_t n = 0 ; n < NUM_X ; n++){
		MAT_ELEMENT(ekf_t->IFT, n, n) += 1.0f;
	}
	res |= arm_mat_trans_f32(&ekf_t->IFT, &ekf_t->IFTT);
	res |= arm_mat_mult_f32(&ekf_t->IFT, &ekf_t->P, &ekf_t->IFTP);
	res |= arm_mat_mult_f32(&ekf_t->IFTP, &ekf_t->IFTT, &ekf_t->IFTPIFTT);
	
	res |= arm_mat_mult_f32(&ekf_t->G, &ekf_t->Q, &ekf_t->GQ);
	res |= arm_mat_trans_f32(&ekf_t->G, &ekf_t->GT);
	res |= arm_mat_mult_f32(&ekf_t->GQ, &ekf_t->GT, &ekf_t->GQGT);
	res |= arm_mat_scale_f32(&ekf_t->GQGT, ekf_t->dT*ekf_t->dT, &ekf_t->GQGT);
	
	res |= arm_mat_add_f32(&ekf_t->IFTPIFTT, &ekf_t->GQGT, &ekf_t->P);
	
//	if(res != ARM_MATH_SUCCESS){
//		Console.print("predict err:%d\n", res);
//	}
	
	return res==ARM_MATH_SUCCESS ? 1 : 0;
}

uint8_t EKF14_Correct(EKF_Def* ekf_t)
{
	float32_t q0 = MAT_ELEMENT(ekf_t->X, STATE_Q0, 0);
	float32_t q1 = MAT_ELEMENT(ekf_t->X, STATE_Q1, 0);
	float32_t q2 = MAT_ELEMENT(ekf_t->X, STATE_Q2, 0);
	float32_t q3 = MAT_ELEMENT(ekf_t->X, STATE_Q3, 0);
	
	//TODO: remove acc bias?
	float32_t ax = MAT_ELEMENT(ekf_t->U, 3, 0);
	float32_t ay = MAT_ELEMENT(ekf_t->U, 4, 0);
	float32_t az = MAT_ELEMENT(ekf_t->U, 5, 0);
	
	float32_t mx = MAT_ELEMENT(ekf_t->U, 6, 0);
	float32_t my = MAT_ELEMENT(ekf_t->U, 7, 0);
	float32_t mz = MAT_ELEMENT(ekf_t->U, 8, 0);

	//normalize acc and mag
	float32_t inv_norm;
	inv_norm = 1.0f/sqrtf(ax*ax+ay*ay+az*az);
	ax = ax*inv_norm;
	ay = ay*inv_norm;
	az = az*inv_norm;
	inv_norm = 1.0f/sqrtf(mx*mx+my*my+mz*mz);
	mx = mx*inv_norm;
	my = my*inv_norm;
	mz = mz*inv_norm;
	
	/* calculate jocobbians of h(x) */
	// d(X)/d(X)
	MAT_ELEMENT(ekf_t->H, 0, 0) = 1.0f;
	MAT_ELEMENT(ekf_t->H, 1, 1) = 1.0f;
	MAT_ELEMENT(ekf_t->H, 2, 2) = 1.0f;
//    // d(V)/d(V)
//	MAT_ELEMENT(ekf_t->H, 3, 3) = 1.0f;
//	MAT_ELEMENT(ekf_t->H, 4, 4) = 1.0f;
//	MAT_ELEMENT(ekf_t->H, 5, 5) = 1.0f;
	// d(acc)/d(q)
    MAT_ELEMENT(ekf_t->H, 3, 6) = 2.0f * (q0 * ax - q3 * ay + q2 * az);
    MAT_ELEMENT(ekf_t->H, 3, 7) = 2.0f * (q1 * ax + q2 * ay + q3 * az);
    MAT_ELEMENT(ekf_t->H, 3, 8) = 2.0f * (-q2 * ax + q1 * ay + q0 * az);
    MAT_ELEMENT(ekf_t->H, 3, 9) = 2.0f * (-q3 * ax - q0 * ay + q1 * az);
    MAT_ELEMENT(ekf_t->H, 4, 6) = 2.0f * (q3 * ax + q0 * ay - q1 * az);
    MAT_ELEMENT(ekf_t->H, 4, 7) = 2.0f * (q2 * ax - q1 * ay - q0 * az);
    MAT_ELEMENT(ekf_t->H, 4, 8) = 2.0f * (q1 * ax + q2 * ay + q3 * az);
    MAT_ELEMENT(ekf_t->H, 4, 9) = 2.0f * (q0 * ax - q3 * ay + q2 * az);
    MAT_ELEMENT(ekf_t->H, 5, 6) = 2.0f * (-q2 * ax + q1 * ay + q0 * az);
    MAT_ELEMENT(ekf_t->H, 5, 7) = 2.0f * (q3 * ax + q0 * ay - q1 * az);
    MAT_ELEMENT(ekf_t->H, 5, 8) = 2.0f * (-q0 * ax + q3 * ay - q2 * az);
    MAT_ELEMENT(ekf_t->H, 5, 9) = 2.0f * (q1 * ax + q2 * ay + q3 * az);
	// d(mag)/d(q)
    MAT_ELEMENT(ekf_t->H, 6, 6) = 2.0f * (q0 * mx - q3 * my + q2 * mz);
    MAT_ELEMENT(ekf_t->H, 6, 7) = 2.0f * (q1 * mx + q2 * my + q3 * mz);
    MAT_ELEMENT(ekf_t->H, 6, 8) = 2.0f * (-q2 * mx + q1 * my + q0 * mz);
    MAT_ELEMENT(ekf_t->H, 6, 9) = 2.0f * (-q3 * mx - q0 * my + q1 * mz);
    MAT_ELEMENT(ekf_t->H, 7, 6) = 2.0f * (q3 * mx + q0 * my - q1 * mz);
    MAT_ELEMENT(ekf_t->H, 7, 7) = 2.0f * (q2 * mx - q1 * my - q0 * mz);
    MAT_ELEMENT(ekf_t->H, 7, 8) = 2.0f * (q1 * mx + q2 * my + q3 * mz);
    MAT_ELEMENT(ekf_t->H, 7, 9) = 2.0f * (q0 * mx - q3 * my + q2 * mz);
	
	// rotate acc and mag from body frame to navigation frame
	float32_t accN[3], magN[2];
	
	accN[0] = (q0*q0+q1*q1-q2*q2-q3*q3)*ax+2.0f*(q1*q2-q0*q3)*ay+2.0f*(q1*q3+q0*q2)*az;
	accN[1] = 2.0f*(q1*q2+q0*q3)*ax+(q0*q0-q1*q1+q2*q2-q3*q3)*ay+2.0f*(q2*q3-q0*q1)*az;
	accN[2] = 2.0f*(q1*q3-q0*q2)*ax+2.0f*(q2*q3+q0*q1)*ay+(q0*q0-q1*q1-q2*q2+q3*q3)*az;
	
	magN[0] = (q0*q0+q1*q1-q2*q2-q3*q3)*mx+2.0f*(q1*q2-q0*q3)*my+2.0f*(q1*q3+q0*q2)*mz;
	magN[1] = 2.0f*(q1*q2+q0*q3)*mx+(q0*q0-q1*q1+q2*q2-q3*q3)*my+2.0f*(q2*q3-q0*q1)*mz;
	inv_norm = 1.0f/sqrtf(magN[0]*magN[0]+magN[1]*magN[1]);
	magN[0] *= inv_norm;
	magN[1] *= inv_norm;
	
	/* Y(k) = Z(k) - h(X(k|k-1)) */
	float32_t hx[NUM_Z] = 
		{ MAT_ELEMENT(ekf_t->X, STATE_X, 0), MAT_ELEMENT(ekf_t->X, STATE_Y, 0), MAT_ELEMENT(ekf_t->X, STATE_Z, 0),
		  accN[0], accN[1], accN[2], magN[0], magN[1] };
		
	for(uint8_t n = 0 ; n < NUM_Z ; n++){
		MAT_ELEMENT(ekf_t->Y, n, 0) = MAT_ELEMENT(ekf_t->Z, n, 0) - hx[n];
	}
	
	arm_status res = ARM_MATH_SUCCESS;
	
	/* S(k) = H(k)*P(k|k-1)*H(k)' + R(k) */
	res |= arm_mat_trans_f32(&ekf_t->H, &ekf_t->HT);
	res |= arm_mat_mult_f32(&ekf_t->P, &ekf_t->HT, &ekf_t->PHT);
	res |= arm_mat_mult_f32(&ekf_t->H, &ekf_t->PHT, &ekf_t->HPHT);
	res |= arm_mat_add_f32(&ekf_t->HPHT, &ekf_t->R, &ekf_t->S);
	
	/* K(k) = P(k|k-1)*H(k)'*S(k)^-1 */
	res |= arm_mat_inverse_f32(&ekf_t->S, &ekf_t->INV_S);
	res |= arm_mat_mult_f32(&ekf_t->PHT, &ekf_t->INV_S, &ekf_t->K);
	
	/* X(k|k) = X(k|k-1) + K(k)*Y(k) */
	res |= arm_mat_mult_f32(&ekf_t->K, &ekf_t->Y, &ekf_t->KY);
	res |= arm_mat_add_f32(&ekf_t->X, &ekf_t->KY, &ekf_t->X);
	
	// normalize quaternion
	q0 = MAT_ELEMENT(ekf_t->X, STATE_Q0, 0);
	q1 = MAT_ELEMENT(ekf_t->X, STATE_Q1, 0);
	q2 = MAT_ELEMENT(ekf_t->X, STATE_Q2, 0);
	q3 = MAT_ELEMENT(ekf_t->X, STATE_Q3, 0);
	inv_norm = 1.0f/sqrtf(q0*q0+q1*q1+q2*q2+q3*q3);
	MAT_ELEMENT(ekf_t->X, STATE_Q0, 0) *= inv_norm;
	MAT_ELEMENT(ekf_t->X, STATE_Q1, 0) *= inv_norm;
	MAT_ELEMENT(ekf_t->X, STATE_Q2, 0) *= inv_norm;
	MAT_ELEMENT(ekf_t->X, STATE_Q3, 0) *= inv_norm;
	
	/* P(k|k) = P(k|k-1) - K(k)*H(k)*P(k|k-1); */
	res |= arm_mat_mult_f32(&ekf_t->K, &ekf_t->H, &ekf_t->KH);
	res |= arm_mat_mult_f32(&ekf_t->KH, &ekf_t->P, &ekf_t->KHP);
	res |= arm_mat_sub_f32(&ekf_t->P, &ekf_t->KHP, &ekf_t->P);
	
	//static uint32_t time = 0;
	//Console.print_eachtime(&time, 300, "new P:%f %f %f\n", MAT_ELEMENT(ekf_t->P, 0, 0), MAT_ELEMENT(ekf_t->P, 3, 3), MAT_ELEMENT(ekf_t->P, 6, 6));
	
	
//	if(res != ARM_MATH_SUCCESS){
//		Console.print("correct err:%d\n", res);
//	}
	
	return res==ARM_MATH_SUCCESS ? 1 : 0;
}

uint8_t EKF14_SerialCorrect(EKF_Def* ekf_t, uint32_t enable_bitmask)
{
	float32_t HP[NUM_X], Y[NUM_Z], HPHR;
    uint8_t i, j, k, m;
	
	float32_t q0 = MAT_ELEMENT(ekf_t->X, STATE_Q0, 0);
	float32_t q1 = MAT_ELEMENT(ekf_t->X, STATE_Q1, 0);
	float32_t q2 = MAT_ELEMENT(ekf_t->X, STATE_Q2, 0);
	float32_t q3 = MAT_ELEMENT(ekf_t->X, STATE_Q3, 0);
	
	/* calculate jocobbians of h(x) */
	// d(X)/d(X)
	if(enable_bitmask & (0x01 << 0)){
		MAT_ELEMENT(ekf_t->H, 0, 0) = 1.0f;
		Y[0] = MAT_ELEMENT(ekf_t->X, STATE_X, 0);
	}
	if(enable_bitmask & (0x01 << 1)){
		MAT_ELEMENT(ekf_t->H, 1, 1) = 1.0f;
		Y[1] = MAT_ELEMENT(ekf_t->X, STATE_Y, 0);
	}
	if(enable_bitmask & (0x01 << 2)){
		MAT_ELEMENT(ekf_t->H, 2, 2) = 1.0f;
		Y[2] = MAT_ELEMENT(ekf_t->X, STATE_Z, 0);
	}
	// d(acc)/d(q)
	if( (enable_bitmask & 0x38) == 0x38 ){
		//TODO: remove acc bias?
		float32_t ax = MAT_ELEMENT(ekf_t->U, 3, 0);
		float32_t ay = MAT_ELEMENT(ekf_t->U, 4, 0);
		float32_t az = MAT_ELEMENT(ekf_t->U, 5, 0);
		
		float32_t inv_norm;
		inv_norm = 1.0f/MAX(sqrtf(ax*ax+ay*ay+az*az), 1e-6);
		ax = ax*inv_norm;
		ay = ay*inv_norm;
		az = az*inv_norm;
		
		MAT_ELEMENT(ekf_t->H, 3, 6) = 2.0f * (q0 * ax - q3 * ay + q2 * az);
		MAT_ELEMENT(ekf_t->H, 3, 7) = 2.0f * (q1 * ax + q2 * ay + q3 * az);
		MAT_ELEMENT(ekf_t->H, 3, 8) = 2.0f * (-q2 * ax + q1 * ay + q0 * az);
		MAT_ELEMENT(ekf_t->H, 3, 9) = 2.0f * (-q3 * ax - q0 * ay + q1 * az);
		MAT_ELEMENT(ekf_t->H, 4, 6) = 2.0f * (q3 * ax + q0 * ay - q1 * az);
		MAT_ELEMENT(ekf_t->H, 4, 7) = 2.0f * (q2 * ax - q1 * ay - q0 * az);
		MAT_ELEMENT(ekf_t->H, 4, 8) = 2.0f * (q1 * ax + q2 * ay + q3 * az);
		MAT_ELEMENT(ekf_t->H, 4, 9) = 2.0f * (q0 * ax - q3 * ay + q2 * az);
		MAT_ELEMENT(ekf_t->H, 5, 6) = 2.0f * (-q2 * ax + q1 * ay + q0 * az);
		MAT_ELEMENT(ekf_t->H, 5, 7) = 2.0f * (q3 * ax + q0 * ay - q1 * az);
		MAT_ELEMENT(ekf_t->H, 5, 8) = 2.0f * (-q0 * ax + q3 * ay - q2 * az);
		MAT_ELEMENT(ekf_t->H, 5, 9) = 2.0f * (q1 * ax + q2 * ay + q3 * az);
		
		float32_t accN[3];
		accN[0] = (q0*q0+q1*q1-q2*q2-q3*q3)*ax+2.0f*(q1*q2-q0*q3)*ay+2.0f*(q1*q3+q0*q2)*az;
		accN[1] = 2.0f*(q1*q2+q0*q3)*ax+(q0*q0-q1*q1+q2*q2-q3*q3)*ay+2.0f*(q2*q3-q0*q1)*az;
		accN[2] = 2.0f*(q1*q3-q0*q2)*ax+2.0f*(q2*q3+q0*q1)*ay+(q0*q0-q1*q1-q2*q2+q3*q3)*az;
		
		Y[3] = accN[0];
		Y[4] = accN[1];
		Y[5] = accN[2];
	}
	// d(mag)/d(q)
	if( (enable_bitmask & 0xC0) == 0xC0 ){
		float32_t mx = MAT_ELEMENT(ekf_t->U, 6, 0);
		float32_t my = MAT_ELEMENT(ekf_t->U, 7, 0);
		float32_t mz = MAT_ELEMENT(ekf_t->U, 8, 0);

		float32_t inv_norm;
		inv_norm = 1.0f/MAX(sqrtf(mx*mx+my*my+mz*mz), 1e-6);
		mx = mx*inv_norm;
		my = my*inv_norm;
		mz = mz*inv_norm;
		
		MAT_ELEMENT(ekf_t->H, 6, 6) = 2.0f * (q0 * mx - q3 * my + q2 * mz);
		MAT_ELEMENT(ekf_t->H, 6, 7) = 2.0f * (q1 * mx + q2 * my + q3 * mz);
		MAT_ELEMENT(ekf_t->H, 6, 8) = 2.0f * (-q2 * mx + q1 * my + q0 * mz);
		MAT_ELEMENT(ekf_t->H, 6, 9) = 2.0f * (-q3 * mx - q0 * my + q1 * mz);
		MAT_ELEMENT(ekf_t->H, 7, 6) = 2.0f * (q3 * mx + q0 * my - q1 * mz);
		MAT_ELEMENT(ekf_t->H, 7, 7) = 2.0f * (q2 * mx - q1 * my - q0 * mz);
		MAT_ELEMENT(ekf_t->H, 7, 8) = 2.0f * (q1 * mx + q2 * my + q3 * mz);
		MAT_ELEMENT(ekf_t->H, 7, 9) = 2.0f * (q0 * mx - q3 * my + q2 * mz);
		
		float32_t magN[2];
		magN[0] = (q0*q0+q1*q1-q2*q2-q3*q3)*mx+2.0f*(q1*q2-q0*q3)*my+2.0f*(q1*q3+q0*q2)*mz;
		magN[1] = 2.0f*(q1*q2+q0*q3)*mx+(q0*q0-q1*q1+q2*q2-q3*q3)*my+2.0f*(q2*q3-q0*q1)*mz;
		inv_norm = 1.0f/MAX(sqrtf(magN[0]*magN[0]+magN[1]*magN[1]), 1e-6);
		magN[0] *= inv_norm;
		magN[1] *= inv_norm;
		
		Y[6] = magN[0];
		Y[7] = magN[1];
	}

    for (m = 0; m < NUM_Z; m++) {
        if (enable_bitmask & (0x01 << m)) { // use this sensor for update

            for (j = 0; j < NUM_X; j++) { // Find Hp = H*P
                HP[j] = 0.0f;
                for (k = 0; k < NUM_X; k++) {
                    HP[j] += MAT_ELEMENT(ekf_t->H, m, k) * MAT_ELEMENT(ekf_t->P, k, j);
                }
            }
            HPHR = MAT_ELEMENT(ekf_t->R, m, m); // Find  HPHR = H*P*H' + R
            for (k = 0; k < NUM_X; k++) {
                HPHR += HP[k] * MAT_ELEMENT(ekf_t->H, m, k);
            }

            for (k = 0; k < NUM_X; k++) {
                MAT_ELEMENT(ekf_t->K, k, m) = HP[k] / HPHR; // find K = HP/HPHR
            }
            for (i = 0; i < NUM_X; i++) { // Find P(m)= P(m-1) + K*HP
                for (j = i; j < NUM_X; j++) {
                    MAT_ELEMENT(ekf_t->P, i, j) = MAT_ELEMENT(ekf_t->P, j, i) =
                                  MAT_ELEMENT(ekf_t->P, i, j) - MAT_ELEMENT(ekf_t->K, i, m) * HP[j];
                }
            }

			MAT_ELEMENT(ekf_t->Y, m, 0) =  MAT_ELEMENT(ekf_t->Z, m, 0) - Y[m];
            for (i = 0; i < NUM_X; i++) { // Find X(m)= X(m-1) + K*Error
                MAT_ELEMENT(ekf_t->X, i, 0) += MAT_ELEMENT(ekf_t->K, i, m) * MAT_ELEMENT(ekf_t->Y, m, 0);
            }
        }
    }	

	// normalize quaternion
	if( (enable_bitmask & 0x38) == 0x38 || (enable_bitmask & 0xC0) == 0xC0 ){
		q0 = MAT_ELEMENT(ekf_t->X, STATE_Q0, 0);
		q1 = MAT_ELEMENT(ekf_t->X, STATE_Q1, 0);
		q2 = MAT_ELEMENT(ekf_t->X, STATE_Q2, 0);
		q3 = MAT_ELEMENT(ekf_t->X, STATE_Q3, 0);
		float32_t inv_norm = 1.0f/MAX(sqrtf(q0*q0+q1*q1+q2*q2+q3*q3), 1e-6);
		MAT_ELEMENT(ekf_t->X, STATE_Q0, 0) *= inv_norm;
		MAT_ELEMENT(ekf_t->X, STATE_Q1, 0) *= inv_norm;
		MAT_ELEMENT(ekf_t->X, STATE_Q2, 0) *= inv_norm;
		MAT_ELEMENT(ekf_t->X, STATE_Q3, 0) *= inv_norm;
	}
	
	//static uint32_t time = 0;
	//Console.print_eachtime(&time, 300, "new P:%f %f %f enable:%x\n", MAT_ELEMENT(ekf_t->P, 0, 0), MAT_ELEMENT(ekf_t->P, 3, 3), MAT_ELEMENT(ekf_t->P, 6, 6),enable_bitmask);
	
	return 1;
}

float32_t EKF14_Get_State(const EKF_Def* ekf_t, uint8_t state)
{
	return MAT_ELEMENT(ekf_t->X, state, 0);
}
