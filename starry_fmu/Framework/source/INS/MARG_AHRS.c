/*
 * File      : MARG_AHRS.c
 *
 * Change Logs:
 * Date           Author       Notes
 * 2017-11-06     zoujiachi    first version.
 */

#include <math.h>
#include "MARG_AHRS.h"
#include "global.h"
#include "console.h"

// System constants
#define deltat 0.004f // sampling period in seconds (shown as 1 ms)
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
void MARG_AHRS_Update(quaternion* q, float g_x, float g_y, float g_z, float a_x, float a_y, float a_z, float m_x, float m_y, float m_z, float dT)
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