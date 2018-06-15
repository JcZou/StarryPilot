/*
 * File      : ekf.h
 *
 *
 * Change Logs:
 * Date           Author       	Notes
 * 2017-08-09     zoujiachi   	the first version
 */
 
#ifndef __EKF_H__
#define __EKF_H__

//#include <rtthread.h>
//#include <rtdevice.h>
#include "light_matrix.h"
#include "pos_estimator.h"

#define EKF_QVN_SIGMA		(0.1f)
#define EKF_QVE_SIGMA		(0.1f)
#define EKF_QVD_SIGMA		(0.1f)
#define EKF_QAN_SIGMA		(0.0513f)
#define EKF_QAE_SIGMA		(0.0471f)
#define EKF_QAD_SIGMA		(0.0859f)
                            
#define EKF_RLA_SIGMA		(316.693f)
#define EKF_RLO_SIGMA		(1051.9f)
#define EKF_RH_SIGMA		(0.1313f)
#define EKF_RVN_SIGMA		(0.2178f)
#define EKF_RVE_SIGMA		(0.3969f)
#define EKF_RVD_SIGMA		(0.1859f)

typedef struct
{
	Mat x;		// states
	Mat pre_x;	// predicted states
	Mat last_x;	// last x status
	Mat u;		// control vector
	//Mat f;		// state function
	Mat F;		// jacobian matrix of f
	Mat P;	
	Mat pre_P;
	Mat Q;
	//Mat h;		// observation function
	Mat H;		// jacobian matrix of h
	Mat R;
	Mat z;		// observation vector
	Mat y;
	Mat S;
	Mat K;		// kalman gain
	float T;	// update interval
	uint32_t ite_cnt;	//iteration count
}EKF_Def;

void EKF6_Init(EKF_Def* ekf_t, float dT, HOME_Pos home_pos);
void EKF6_Update(EKF_Def* ekf_t);
void EKF2_Init(EKF_Def* ekf_t, float qh, float qv, float rh, float rv, float dT);
void EKF2_Predict(EKF_Def* ekf_t);
void EKF2_Update(EKF_Def* ekf_t);
float EKF2_GetZ_Bias(void);

#endif
