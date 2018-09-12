/*
 * File      : AHRS.c
 *
 * Change Logs:
 * Date           Author       Notes
 * 2016-07-01     zoujiachi    first version.
 * 2016-12-29     zoujiachi    rewrite version.
 */

#include <math.h>
#include "AHRS.h"
#include "global.h"
#include "console.h"
#include "uMCN.h"
#include "pos_estimator.h"

// System constants
//#define deltat 0.004f // sampling period in seconds (shown as 1 ms)
//#define gyroMeasError 3.14159265358979 * (5.0f / 180.0f) // gyroscope measurement error in rad/s (shown as 5 deg/s)
//#define gyroMeasDrift 3.14159265358979 * (0.2f / 180.0f) // gyroscope measurement error in rad/s/s (shown as 0.2f deg/s/s)
//#define gyroMeasError 3.14159265358979 * (4.0f / 180.0f) // gyroscope measurement error in rad/s (shown as 5 deg/s)
//#define gyroMeasDrift 3.14159265358979 * (0.15f / 180.0f) // gyroscope measurement error in rad/s/s (shown as 0.2f deg/s/s)
#define gyroMeasError 3.14159265358979 * (1.5f / 180.0f) // gyroscope measurement error in rad/s (shown as 5 deg/s)
#define gyroMeasDrift 3.14159265358979 * (0.4f / 180.0f) // gyroscope measurement error in rad/s/s (shown as 0.2f deg/s/s)
//#define beta sqrt(3.0f / 4.0f) * gyroMeasError // compute beta
//#define zeta sqrt(3.0f / 4.0f) * gyroMeasDrift // compute zeta
static float beta = 0.8660254f * gyroMeasError; // compute beta
static float zeta = 0.8660254f * gyroMeasDrift; // compute zeta
// Global system variables
//float a_x, a_y, a_z; // accelerometer measurements
//float w_x, w_y, w_z; // gyroscope measurements in rad/s
//float m_x, m_y, m_z; // magnetometer measurements
//float SEq_1 = 1, SEq_2 = 0, SEq_3 = 0, SEq_4 = 0; // estimated orientation quaternion elements with initial conditions
float b_x = 1, b_z = 0; // reference direction of flux in earth frame
float w_bx = 0, w_by = 0, w_bz = 0; // estimate gyroscope biases error

static float accU[3], magU[3];
static float delta[3];
//static float FACTOR_P = 2.0f;
//static float FACTOR_I = 0.005f;
static float FACTOR_P = 0.25f;
static float FACTOR_I = 0.1f;

static float acc_cross[3];
static float mag_cross[3];
static float acc_const[3] = {0.0f, 0.0f, -1.0f};
static float mag_const[3] = {1.0f, 0.0f, 0.0f};
static float gyr_bias[3];
static float errX_Int = 0.0f;
static float errY_Int = 0.0f;
static float errZ_Int = 0.0f;

float comp_gain = 0.05f;

extern McnNode_t _home_node_t;
MCN_DECLARE(HOME_POS);

void Runge_Kutta_1st(quaternion* attitude, quaternion q, float g[3], float dT)
{
	float halfT = 0.5f*dT;
	
	OS_ENTER_CRITICAL;
	attitude->w += halfT * (-g[0] * q.x - g[1] * q.y - g[2] * q.z);
	attitude->x += halfT * ( g[0] * q.w - g[1] * q.z + g[2] * q.y);
	attitude->y += halfT * ( g[0] * q.z + g[1] * q.w - g[2] * q.x);
	attitude->z += halfT * (-g[0] * q.y + g[1] * q.x + g[2] * q.w);
	OS_EXIT_CRITICAL;
}

/* use acc and mag data to calculate the current attitude */
void AHRS_reset(quaternion * q, const float acc[3],const float mag[3])
{
	quaternion q1,q2;
	float to[3];
	float from[3];
	
	/* pitch-row modification */
	/* calculate rotation from current acc to acc constant [0,0,-1] */
	to[0] = to[1] = 0.0f;
	to[2] = -1.0f;
	quaternion_fromTwoVectorRotation(&q1, acc, to);
	
	/* yaw modification */
	/* first rotate mag by q1 */
	quaternion_rotateVector(&q1, mag, from);
	from[2] = 0;
	to[0] = 1;
	to[1] = to[2] = 0;
	/* calculate rotation from mag to mag constant [1,0,0] */
	quaternion_fromTwoVectorRotation(&q2, from, to);
	
	/* combine two rotations to get current attitude */
	quaternion_mult(q, &q2, &q1);
	quaternion_normalize(q);
	
	gyr_bias[0] = 0.0f;
	gyr_bias[1] = 0.0f;
	gyr_bias[2] = 0.0f;
}

void AHRS_update(quaternion * q, const float gyr[3], const float acc[3], const float mag[3], float dT)
{	
	Vector3_Normalize(accU, acc);
	Vector3_Normalize(magU, mag);
	
	/* transfer acc and mag from body frame to nav frame */
	float acc_N[3], mag_N[3];
	quaternion_rotateVector(q, accU, acc_N);
	quaternion_rotateVector(q, magU, mag_N);
	
//	/* correct magnetic declination */
//	if(mcn_poll(_home_node_t)){
//		HOME_Pos home;
//		mcn_copy(MCN_ID(HOME_POS), _home_node_t, &home);
//		
//		float axis[3] = {0,0,-1};
//		float mag_axis[3] = {1,0,0};
//		quaternion qr;
//		
//		quaternion_create(&qr, Deg2Rad(home.mag_decl), axis);
//		quaternion_rotateVector(&qr, mag_axis, mag_const);
//	}
		
	/* we only need x and y value for mag to calculate error of yaw */
	mag_N[2] = 0.0f;
	Vector3_Normalize(mag_N, mag_N);
	
	/* cross product to calculate diffirence */
	Vector3_CrossProduct(acc_cross, acc_N, acc_const);
	Vector3_CrossProduct(mag_cross, mag_N, mag_const);
	
	/* calculate error in navigation frame */
	/* error of roll and pitch come from acc, error of yaw come from mag */
	float err_N[3], err_B[3];

	err_N[0] = acc_cross[0] + mag_cross[0];
	err_N[1] = acc_cross[1] + mag_cross[1];
	err_N[2] = acc_cross[2] + mag_cross[2];
	
	/* transfer error from navigation frame to body frame */
	quaternion_inv_rotateVector(q, err_N, err_B);
	
	/* integrate error to estimate gyr bias */
	gyr_bias[0] += err_B[0]*FACTOR_I*dT;  
	gyr_bias[1] += err_B[1]*FACTOR_I*dT;  
	gyr_bias[2] += err_B[2]*FACTOR_I*dT; 
 
	/* calculate delta value */
	delta[0] = gyr[0] + FACTOR_P*err_B[0] + gyr_bias[0];  
	delta[1] = gyr[1] + FACTOR_P*err_B[1] + gyr_bias[1];  
	delta[2] = gyr[2] + FACTOR_P*err_B[2] + gyr_bias[2];
	
	/* first order runge-kutta to create quaternion */
	Runge_Kutta_1st(q, *q, delta, dT);
	quaternion_normalize(q);
}

void MahonyAHRS_update(quaternion * q,const float gyr[3],const float acc[3],const float mag[3],float dT)
{ 
	float hx, hy, hz, bx, bz;  
	float vx, vy, vz, wx, wy, wz;   
	float ex, ey, ez;  
	static float exInt = 0.0f;
	static float eyInt = 0.0f;
	static float ezInt = 0.0f;

	// auxiliary variables to reduce number of repeated operations   
	float qw_qw_2 = 2.0f*q->w*q->w;  
	float qw_qx_2 = 2.0f*q->w*q->x;  
	float qw_qy_2 = 2.0f*q->w*q->y;  
	float qw_qz_2 = 2.0f*q->w*q->z;  
	float qx_qx_2 = 2.0f*q->x*q->x;  
	float qx_qy_2 = 2.0f*q->x*q->y;  
	float qx_qz_2 = 2.0f*q->x*q->z;  
	float qy_qy_2 = 2.0f*q->y*q->y;  
	float qy_qz_2 = 2.0f*q->y*q->z;  
	float qz_qz_2 = 2.0f*q->z*q->z;
	

	// normalise the measurements  
	Vector3_Normalize(accU, acc);
	Vector3_Normalize(magU, mag);

	// compute reference direction of magnetic field 
    // transfer magU from body frame to nav frame 	
	hx = magU[0]*(1.0f - qy_qy_2 - qz_qz_2) + magU[1]*(qx_qy_2 - qw_qz_2) 		 + magU[2]*(qx_qz_2 + qw_qy_2);  
	hy = magU[0]*(qx_qy_2 + qw_qz_2) 		+ magU[1]*(1.0f - qx_qx_2 - qz_qz_2) + magU[2]*(qy_qz_2 - qw_qx_2);  
	hz = magU[0]*(qx_qz_2 - qw_qy_2) 		+ magU[1]*(qy_qz_2 + qw_qx_2) 		 + magU[2]*(1.0f - qx_qx_2 - qy_qy_2);
	
    // [bx,0,bz] points to north direction and has the same z value with magU	
	bx = sqrtf((hx*hx) + (hy*hy));  
	bz = hz; 

	// estimated direction of gravity and magnetic field (v and w) 
	// the constant gravity is (0,0,-1), because accelerator measure the relative acceleration
	// transfer acc constant [0,0,-1] from nav frame to body frame 
	vx = qw_qy_2 - qx_qz_2;  
	vy = -qw_qx_2 - qy_qz_2;  
	vz = -1.0f + qx_qx_2 + qy_qy_2;  
	// transfer [bx,0,bz] from nav frame to body frame
	wx = bx*(1.0f - qy_qy_2 - qz_qz_2) + bz*(qx_qz_2 - qw_qy_2);  
	wy = bx*(qx_qy_2 - qw_qz_2) + bz*(qw_qx_2 + qy_qz_2);  
	wz = bx*(qw_qy_2 + qx_qz_2) + bz*(1.0f - qx_qx_2 - qy_qy_2); 

	// error is sum ofcross product between reference direction of fields and directionmeasured by sensors  
	ex = (accU[1]*vz - accU[2]*vy) + (magU[1]*wz - magU[2]*wy);  
	ey = (accU[2]*vx - accU[0]*vz) + (magU[2]*wx - magU[0]*wz);  
	ez = (accU[0]*vy - accU[1]*vx) + (magU[0]*wy - magU[1]*wx);  

	// integral error scaled integral gain  
	exInt += ex*FACTOR_I* dT;  
	eyInt += ey*FACTOR_I* dT;  
	ezInt += ez*FACTOR_I* dT;  
	// adjusted gyroscope measurements  
	delta[0] = gyr[0] + FACTOR_P*ex + exInt;  
	delta[1] = gyr[1] + FACTOR_P*ey + eyInt;  
	delta[2] = gyr[2] + FACTOR_P*ez + ezInt;  

	// integrate quaternion rate and normalize   
	Runge_Kutta_1st(q, *q, delta, dT);

	// normalise quaternion   	
	quaternion_normalize(q);
}

void MARG_AHRS_IMU_Update(quaternion* q, float g_x, float g_y, float g_z, float a_x, float a_y, float a_z, float dT)
{
	// Local system variables
	float norm; // vector norm
	float SEqDot_omega_1, SEqDot_omega_2, SEqDot_omega_3, SEqDot_omega_4; // quaternion derrivative from gyroscopes elements
	float f_1, f_2, f_3; // objective function elements
	float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33; // objective function Jacobian elements
	float SEqHatDot_1, SEqHatDot_2, SEqHatDot_3, SEqHatDot_4; // estimated direction of the gyroscope error
	// Axulirary variables to avoid reapeated calcualtions
	float halfSEq_1 = 0.5f * q->w;
	float halfSEq_2 = 0.5f * q->x;
	float halfSEq_3 = 0.5f * q->y;
	float halfSEq_4 = 0.5f * q->z;
	float twoSEq_1 = 2.0f * q->w;
	float twoSEq_2 = 2.0f * q->x;
	float twoSEq_3 = 2.0f * q->y;
	
	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if((a_x == 0.0f) && (a_y == 0.0f) && (a_z == 0.0f)) {
		return ;
	}
		
	// Normalise the accelerometer measurement
	norm = sqrt(a_x * a_x + a_y * a_y + a_z * a_z);
	a_x /= norm;
	a_y /= norm;
	a_z /= norm;
	// Compute the objective function and Jacobian
	f_1 = twoSEq_2 * q->z - twoSEq_1 * q->y - a_x;
	f_2 = twoSEq_1 * q->x + twoSEq_3 * q->z - a_y;
	f_3 = 1.0f - twoSEq_2 * q->x - twoSEq_3 * q->y - a_z;
	J_11or24 = twoSEq_3; // J_11 negated in matrix multiplication
	J_12or23 = 2.0f * q->z;
	J_13or22 = twoSEq_1; // J_12 negated in matrix multiplication
	J_14or21 = twoSEq_2;
	J_32 = 2.0f * J_14or21; // negated in matrix multiplication
	J_33 = 2.0f * J_11or24; // negated in matrix multiplication
	// Compute the gradient (matrix multiplication)
	SEqHatDot_1 = J_14or21 * f_2 - J_11or24 * f_1;
	SEqHatDot_2 = J_12or23 * f_1 + J_13or22 * f_2 - J_32 * f_3;
	SEqHatDot_3 = J_12or23 * f_2 - J_33 * f_3 - J_13or22 * f_1;
	SEqHatDot_4 = J_14or21 * f_1 + J_11or24 * f_2;
	// Normalise the gradient
	norm = sqrt(SEqHatDot_1 * SEqHatDot_1 + SEqHatDot_2 * SEqHatDot_2 + SEqHatDot_3 * SEqHatDot_3 + SEqHatDot_4 * SEqHatDot_4);
	SEqHatDot_1 /= norm;
	SEqHatDot_2 /= norm;
	SEqHatDot_3 /= norm;
	SEqHatDot_4 /= norm;
	// Compute the quaternion derrivative measured by gyroscopes
	SEqDot_omega_1 = -halfSEq_2 * g_x - halfSEq_3 * g_y - halfSEq_4 * g_z;
	SEqDot_omega_2 = halfSEq_1 * g_x + halfSEq_3 * g_z - halfSEq_4 * g_y;
	SEqDot_omega_3 = halfSEq_1 * g_y - halfSEq_2 * g_z + halfSEq_4 * g_x;
	SEqDot_omega_4 = halfSEq_1 * g_z + halfSEq_2 * g_y - halfSEq_3 * g_x;
	// Compute then integrate the estimated quaternion derrivative
	q->w += (SEqDot_omega_1 - (beta * SEqHatDot_1)) * dT;
	q->x += (SEqDot_omega_2 - (beta * SEqHatDot_2)) * dT;
	q->y += (SEqDot_omega_3 - (beta * SEqHatDot_3)) * dT;
	q->z += (SEqDot_omega_4 - (beta * SEqHatDot_4)) * dT;
	// Normalise quaternion
	norm = sqrt(q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z);
	q->w /= norm;
	q->x /= norm;
	q->y /= norm;
	q->z /= norm;
}

// Function to compute one filter iteration
void MARG_AHRS_update(quaternion* q, float g_x, float g_y, float g_z, float a_x, float a_y, float a_z, float m_x, float m_y, float m_z, float dT)
{
	// local system variables
	float norm; // vector norm
	float SEqDot_omega_1, SEqDot_omega_2, SEqDot_omega_3, SEqDot_omega_4; // quaternion rate from gyroscopes elements
	float f_1, f_2, f_3, f_4, f_5, f_6; // objective function elements
	float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33, // objective function Jacobian elements
	J_41, J_42, J_43, J_44, J_51, J_52, J_53, J_54, J_61, J_62, J_63, J_64; //
	float SEqHatDot_1, SEqHatDot_2, SEqHatDot_3, SEqHatDot_4; // estimated direction of the gyroscope error
	float w_err_x, w_err_y, w_err_z; // estimated direction of the gyroscope error (angular)
	float h_x, h_y, h_z; // computed flux in the earth frame
	// axulirary variables to avoid reapeated calcualtions
	float halfSEq_1 = 0.5f * q->w;
	float halfSEq_2 = 0.5f * q->x;
	float halfSEq_3 = 0.5f * q->y;
	float halfSEq_4 = 0.5f * q->z;
	float twoSEq_1 = 2.0f * q->w;
	float twoSEq_2 = 2.0f * q->x;
	float twoSEq_3 = 2.0f * q->y;
	float twoSEq_4 = 2.0f * q->z;
	float twob_x = 2.0f * b_x;
	float twob_z = 2.0f * b_z;
	float twob_xSEq_1 = 2.0f * b_x * q->w;
	float twob_xSEq_2 = 2.0f * b_x * q->x;
	float twob_xSEq_3 = 2.0f * b_x * q->y;
	float twob_xSEq_4 = 2.0f * b_x * q->z;
	float twob_zSEq_1 = 2.0f * b_z * q->w;
	float twob_zSEq_2 = 2.0f * b_z * q->x;
	float twob_zSEq_3 = 2.0f * b_z * q->y;
	float twob_zSEq_4 = 2.0f * b_z * q->z;
	float SEq_1SEq_2;
	float SEq_1SEq_3 = q->w * q->y;
	float SEq_1SEq_4;
	float SEq_2SEq_3;
	float SEq_2SEq_4 = q->x * q->z;
	float SEq_3SEq_4;
	float twom_x = 2.0f * m_x;
	float twom_y = 2.0f * m_y;
	float twom_z = 2.0f * m_z;
	
	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if((a_x == 0.0f) && (a_y == 0.0f) && (a_z == 0.0f)) {
		return ;
	}
	
	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if((m_x == 0.0f) && (m_y == 0.0f) && (m_z == 0.0f)) {
		MARG_AHRS_IMU_Update(q, g_x, g_y, g_z, a_x, a_y, a_z, dT);
		return;
	}

	// normalise the accelerometer measurement
	norm = sqrt(a_x * a_x + a_y * a_y + a_z * a_z);
	a_x /= norm;
	a_y /= norm;
	a_z /= norm;

	// normalise the magnetometer measurement
	norm = sqrt(m_x * m_x + m_y * m_y + m_z * m_z);
	m_x /= norm;
	m_y /= norm;
	m_z /= norm;
	// compute the objective function and Jacobian
	f_1 = twoSEq_2 * q->z - twoSEq_1 * q->y - a_x;
	f_2 = twoSEq_1 * q->x + twoSEq_3 * q->z - a_y;
	f_3 = 1.0f - twoSEq_2 * q->x - twoSEq_3 * q->y - a_z;
	f_4 = twob_x * (0.5f - q->y * q->y - q->z * q->z) + twob_z * (SEq_2SEq_4 - SEq_1SEq_3) - m_x;
	f_5 = twob_x * (q->x * q->y - q->w * q->z) + twob_z * (q->w * q->x + q->y * q->z) - m_y;
	f_6 = twob_x * (SEq_1SEq_3 + SEq_2SEq_4) + twob_z * (0.5f - q->x * q->x - q->y * q->y) - m_z;
	J_11or24 = twoSEq_3; // J_11 negated in matrix multiplication
	J_12or23 = 2.0f * q->z;
	J_13or22 = twoSEq_1; // J_12 negated in matrix multiplication
	J_14or21 = twoSEq_2;
	J_32 = 2.0f * J_14or21; // negated in matrix multiplication
	J_33 = 2.0f * J_11or24; // negated in matrix multiplication
	J_41 = twob_zSEq_3; // negated in matrix multiplication
	J_42 = twob_zSEq_4;
	J_43 = 2.0f * twob_xSEq_3 + twob_zSEq_1; // negated in matrix multiplication
	J_44 = 2.0f * twob_xSEq_4 - twob_zSEq_2; // negated in matrix multiplication
	J_51 = twob_xSEq_4 - twob_zSEq_2; // negated in matrix multiplication
	J_52 = twob_xSEq_3 + twob_zSEq_1;
	J_53 = twob_xSEq_2 + twob_zSEq_4;
	J_54 = twob_xSEq_1 - twob_zSEq_3; // negated in matrix multiplication
	J_61 = twob_xSEq_3;
	J_62 = twob_xSEq_4 - 2.0f * twob_zSEq_2;
	J_63 = twob_xSEq_1 - 2.0f * twob_zSEq_3;
	J_64 = twob_xSEq_2;
	// compute the gradient (matrix multiplication)
	SEqHatDot_1 = J_14or21 * f_2 - J_11or24 * f_1 - J_41 * f_4 - J_51 * f_5 + J_61 * f_6;
	SEqHatDot_2 = J_12or23 * f_1 + J_13or22 * f_2 - J_32 * f_3 + J_42 * f_4 + J_52 * f_5 + J_62 * f_6;
	SEqHatDot_3 = J_12or23 * f_2 - J_33 * f_3 - J_13or22 * f_1 - J_43 * f_4 + J_53 * f_5 + J_63 * f_6;
	SEqHatDot_4 = J_14or21 * f_1 + J_11or24 * f_2 - J_44 * f_4 - J_54 * f_5 + J_64 * f_6;
	// normalise the gradient to estimate direction of the gyroscope error
	norm = sqrt(SEqHatDot_1 * SEqHatDot_1 + SEqHatDot_2 * SEqHatDot_2 + SEqHatDot_3 * SEqHatDot_3 + SEqHatDot_4 * SEqHatDot_4);
	SEqHatDot_1 = SEqHatDot_1 / norm;
	SEqHatDot_2 = SEqHatDot_2 / norm;
	SEqHatDot_3 = SEqHatDot_3 / norm;
	SEqHatDot_4 = SEqHatDot_4 / norm;
	// compute angular estimated direction of the gyroscope error
	w_err_x = twoSEq_1 * SEqHatDot_2 - twoSEq_2 * SEqHatDot_1 - twoSEq_3 * SEqHatDot_4 + twoSEq_4 * SEqHatDot_3;
	w_err_y = twoSEq_1 * SEqHatDot_3 + twoSEq_2 * SEqHatDot_4 - twoSEq_3 * SEqHatDot_1 - twoSEq_4 * SEqHatDot_2;
	w_err_z = twoSEq_1 * SEqHatDot_4 - twoSEq_2 * SEqHatDot_3 + twoSEq_3 * SEqHatDot_2 - twoSEq_4 * SEqHatDot_1;
	// compute and remove the gyroscope baises
	w_bx += w_err_x * dT * zeta;
	w_by += w_err_y * dT * zeta;
	w_bz += w_err_z * dT * zeta;
	/* TODO: is bias estimation accurate? */
//	g_x -= w_bx;
//	g_y -= w_by;
//	g_z -= w_bz;
	// compute the quaternion rate measured by gyroscopes
	SEqDot_omega_1 = -halfSEq_2 * g_x - halfSEq_3 * g_y - halfSEq_4 * g_z;
	SEqDot_omega_2 = halfSEq_1 * g_x + halfSEq_3 * g_z - halfSEq_4 * g_y;
	SEqDot_omega_3 = halfSEq_1 * g_y - halfSEq_2 * g_z + halfSEq_4 * g_x;
	SEqDot_omega_4 = halfSEq_1 * g_z + halfSEq_2 * g_y - halfSEq_3 * g_x;
	// compute then integrate the estimated quaternion rate
	q->w += (SEqDot_omega_1 - (beta * SEqHatDot_1)) * dT;
	q->x += (SEqDot_omega_2 - (beta * SEqHatDot_2)) * dT;
	q->y += (SEqDot_omega_3 - (beta * SEqHatDot_3)) * dT;
	q->z += (SEqDot_omega_4 - (beta * SEqHatDot_4)) * dT;

	// normalise quaternion
	norm = sqrt(q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z);
	q->w /= norm;
	q->x /= norm;
	q->y /= norm;
	q->z /= norm;
	// compute flux in the earth frame
	SEq_1SEq_2 = q->w * q->x; // recompute axulirary variables
	SEq_1SEq_3 = q->w * q->y;
	SEq_1SEq_4 = q->w * q->z;
	SEq_3SEq_4 = q->y * q->z;
	SEq_2SEq_3 = q->x * q->y;
	SEq_2SEq_4 = q->x * q->z;
	h_x = twom_x * (0.5f - q->y * q->y - q->z * q->z) + twom_y * (SEq_2SEq_3 - SEq_1SEq_4) + twom_z * (SEq_2SEq_4 + SEq_1SEq_3);
	h_y = twom_x * (SEq_2SEq_3 + SEq_1SEq_4) + twom_y * (0.5f - q->x * q->x - q->z * q->z) + twom_z * (SEq_3SEq_4 - SEq_1SEq_2);
	h_z = twom_x * (SEq_2SEq_4 - SEq_1SEq_3) + twom_y * (SEq_3SEq_4 + SEq_1SEq_2) + twom_z * (0.5f - q->x * q->x - q->y * q->y);
	// normalise the flux vector to have only components in the x and z
	b_x = sqrt((h_x * h_x) + (h_y * h_y));
	b_z = h_z;
}

// TODO
void AHRS_gyr_acc_fusion(quaternion * q, const float gyr[3], const float acc[3], float dT)
{	
	Vector3_Normalize(accU, acc);
	
	/* transfer acc and mag from body frame to nav frame */
	float acc_N[3];
	quaternion_rotateVector(q, accU, acc_N);

	/* cross product to calculate diffirence */
	Vector3_CrossProduct(acc_cross, acc_N, acc_const);
	
	/* calculate error in navigation frame */
	/* error of roll and pitch come from acc, error of yaw come from mag */
	float err_N[3], err_B[3];
	err_N[0] = acc_cross[0];
	err_N[1] = acc_cross[1];
	err_N[2] = 0.0f;
	
	/* transfer error from navigation frame to body frame */
	quaternion_inv_rotateVector(q, err_N, err_B);
	
	errX_Int += err_B[0]*FACTOR_I*dT;  
	errY_Int += err_B[1]*FACTOR_I*dT;  
	errZ_Int += err_B[2]*FACTOR_I*dT; 
	
	delta[0] = gyr[0] + FACTOR_P*err_B[0] + errX_Int;  
	delta[1] = gyr[1] + FACTOR_P*err_B[1] + errY_Int;  
	delta[2] = gyr[2] + FACTOR_P*err_B[2] + errZ_Int;

	/* first order runge-kutta to create quaternion */
	Runge_Kutta_1st(q, *q, delta, dT);
	quaternion_normalize(q);
}

// TODO
void AHRS_mag_fusion(quaternion * q, const float mag[3], float dT)
{	
	Vector3_Normalize(magU, mag);
	
	/* transfer acc and mag from body frame to nav frame */
	float mag_N[3];
	quaternion_rotateVector(q, magU, mag_N);
	
	/* we only need x and y value for mag to calculate error of yaw */
	mag_N[2] = 0.0f;
	Vector3_Normalize(mag_N, mag_N);
	
	/* cross product to calculate diffirence */
	Vector3_CrossProduct(mag_cross, mag_N, mag_const);
	
	/* calculate error in navigation frame */
	/* error of roll and pitch come from acc, error of yaw come from mag */
	float err_N[3], err_B[3];
	err_N[0] = 0.0f;
	err_N[1] = 0.0f;
	err_N[2] = mag_cross[2];
	
	/* transfer error from navigation frame to body frame */
	quaternion_inv_rotateVector(q, err_N, err_B);
	
	/* do not let intergrate so fast */
	errX_Int += err_B[0]*FACTOR_I*dT;  
	errY_Int += err_B[1]*FACTOR_I*dT;  
	errZ_Int += err_B[2]*FACTOR_I*dT; 
	
	delta[0] = FACTOR_P*err_B[0] + errX_Int;  
	delta[1] = FACTOR_P*err_B[1] + errY_Int;  
	delta[2] = FACTOR_P*err_B[2] + errZ_Int;

	/* first order runge-kutta to create quaternion */
	Runge_Kutta_1st(q, *q, delta, dT);
	quaternion_normalize(q);
}

