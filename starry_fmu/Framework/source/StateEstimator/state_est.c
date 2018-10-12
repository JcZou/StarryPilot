/*****************************************************************************
Copyright (c) 2018, StarryPilot Development Team. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of StarryPilot nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

#include "state_est.h"
#include "ekf.h"
#include "uMCN.h"
#include "console.h"
#include "sensor_manager.h"
#include "pos_estimator.h"
#include "sensor_manager.h"
#include "gps.h"
#include "fifo.h"

#define EKF_MAX_DELAY_OFFFSET		20
#define EKF_STATE_X_DELAY			100
#define EKF_STATE_Y_DELAY			100
#define EKF_STATE_Z_DELAY			100
#define EKF_STATE_VX_DELAY			100
#define EKF_STATE_VY_DELAY			100
#define EKF_STATE_VZ_DELAY			100
#define EKF_STATE_Q0_DELAY			4
#define EKF_STATE_Q1_DELAY			4
#define EKF_STATE_Q2_DELAY			4
#define EKF_STATE_Q3_DELAY			4
#define EKF_STATE_GX_BIAS_DELAY		0
#define EKF_STATE_GY_BIAS_DELAY		0
#define EKF_STATE_GZ_BIAS_DELAY		0
#define EKF_STATE_AZ_BIAS_DELAY		0
	
static EKF_Def ekf_14;
static quaternion _est_att_q;
static Euler _est_att_e;
static FIFO _hist_x[14];

MCN_DECLARE(ATT_QUATERNION);
MCN_DECLARE(ATT_EULER);
MCN_DECLARE(BARO_POSITION);
MCN_DECLARE(ALT_INFO);
MCN_DECLARE(POS_INFO);
MCN_DECLARE(SENSOR_FILTER_ACC);
MCN_DECLARE(SENSOR_FILTER_MAG);
MCN_DECLARE(SENSOR_FILTER_GYR);

static char *TAG = "State_EST";

void state_est_get_quaternion(quaternion* q)
{
	q->w = MAT_ELEMENT(ekf_14.X, STATE_Q0, 0);
	q->x = MAT_ELEMENT(ekf_14.X, STATE_Q1, 0);
	q->y = MAT_ELEMENT(ekf_14.X, STATE_Q2, 0);
	q->z = MAT_ELEMENT(ekf_14.X, STATE_Q3, 0);
}

void state_est_get_position(Vector3f_t *pos)
{
	pos->x = MAT_ELEMENT(ekf_14.X, STATE_X, 0);
	pos->y = MAT_ELEMENT(ekf_14.X, STATE_Y, 0);
	pos->z = MAT_ELEMENT(ekf_14.X, STATE_Z, 0);
}

void state_est_get_velocity(Vector3f_t *vel)
{
	vel->x = MAT_ELEMENT(ekf_14.X, STATE_VX, 0);
	vel->y = MAT_ELEMENT(ekf_14.X, STATE_VY, 0);
	vel->z = MAT_ELEMENT(ekf_14.X, STATE_VZ, 0);
}

uint8_t state_est_init(float dT)
{
	EKF14_Init(&ekf_14, dT);
	
	uint32_t interval = dT*1e3;
	uint32_t hist_offset[14] = {
		EKF_STATE_X_DELAY/interval, EKF_STATE_Y_DELAY/interval, EKF_STATE_Z_DELAY/interval,
		EKF_STATE_VX_DELAY/interval, EKF_STATE_VY_DELAY/interval, EKF_STATE_VZ_DELAY/interval,
		EKF_STATE_Q0_DELAY/interval, EKF_STATE_Q1_DELAY/interval, EKF_STATE_Q2_DELAY/interval, EKF_STATE_Q3_DELAY/interval,
		EKF_STATE_GX_BIAS_DELAY/interval, EKF_STATE_GY_BIAS_DELAY/interval, EKF_STATE_GZ_BIAS_DELAY/interval, 
		EKF_STATE_AZ_BIAS_DELAY/interval
	};
	
	for(uint8_t i = 0 ; i < 14 ; i++){
		fifo_create(&_hist_x[i], hist_offset[i]+1);
		fifo_flush(&_hist_x[i]);
	}
	
	int mcn_res = mcn_advertise(MCN_ID(ATT_QUATERNION));
	if(mcn_res != 0){
		Console.e(TAG, "err:%d, ATT_QUATERNION advertise fail!\n", mcn_res);
	}
	mcn_res = mcn_advertise(MCN_ID(ATT_EULER));
	if(mcn_res != 0){
		Console.e(TAG, "err:%d, ATT_EULER advertise fail!\n", mcn_res);
	}
	
	return 0;
}

uint8_t state_est_reset(void)
{
	EKF14_Reset(&ekf_14);
	
	return 0;
}

uint8_t state_est_update(void)
{
	float acc[3], gyr[3], mag[3];
	uint32_t enable = 0xFFFF;
	
	pos_try_sethome();
	
	//sensor_get_acc(acc);
	mcn_copy_from_hub(MCN_ID(SENSOR_FILTER_ACC), acc);
	//sensor_get_mag(mag);
	mcn_copy_from_hub(MCN_ID(SENSOR_FILTER_MAG), mag);
	//sensor_get_gyr(gyr);
	mcn_copy_from_hub(MCN_ID(SENSOR_FILTER_GYR), gyr);
	
	if(acc[0] == 0.0f && acc[1] == 0.0f && acc[2] == 0.0f){
		enable &= 0xC7;
	}
	if(mag[0] == 0.0f && mag[1] == 0.0f && mag[2] == 0.0f){
		enable &= 0x3F;
	}
	
	Vector3f_t pos = {0,0,0};
	Vector3f_t vel = {0,0,0};

	struct vehicle_gps_position_s gps_report = gps_get_report();

	GPS_Status gps_status;
	gps_get_status(&gps_status);
	
	HOME_Pos home_pos;
	pos_get_home(&home_pos);
	
	if(gps_status.status == GPS_AVAILABLE && home_pos.gps_coordinate_set){
		gps_get_position(&pos, gps_report);
		gps_get_velocity(&vel, gps_report);
	}else{
		enable &= 0xFC;
	}
	
	BaroPosition baro_pos;
	mcn_copy_from_hub(MCN_ID(BARO_POSITION), &baro_pos);
	
	if(home_pos.baro_altitude_set){
		pos.z = baro_pos.altitude - home_pos.alt;
		vel.z = baro_pos.velocity;
	}else{
		enable &= 0xFB;
	}
	
	MAT_ELEMENT(ekf_14.U, 0, 0) = gyr[0];
	MAT_ELEMENT(ekf_14.U, 1, 0) = gyr[1];
	MAT_ELEMENT(ekf_14.U, 2, 0) = gyr[2];
	MAT_ELEMENT(ekf_14.U, 3, 0) = acc[0];
	MAT_ELEMENT(ekf_14.U, 4, 0) = acc[1];
	MAT_ELEMENT(ekf_14.U, 5, 0) = acc[2];
	MAT_ELEMENT(ekf_14.U, 6, 0) = mag[0];
	MAT_ELEMENT(ekf_14.U, 7, 0) = mag[1];
	MAT_ELEMENT(ekf_14.U, 8, 0) = mag[2];
	
	// MAT_ELEMENT(ekf_14.Z, 0, 0) = pos.x;
	// MAT_ELEMENT(ekf_14.Z, 1, 0) = pos.y;
	// MAT_ELEMENT(ekf_14.Z, 2, 0) = pos.z;
	MAT_ELEMENT(ekf_14.Z, 0, 0) = 0.0f;
	MAT_ELEMENT(ekf_14.Z, 1, 0) = 0.0f;
	MAT_ELEMENT(ekf_14.Z, 2, 0) = pos.z;
	MAT_ELEMENT(ekf_14.Z, 3, 0) = 0.0f;		// acc constant: [0, 0, -1]
	MAT_ELEMENT(ekf_14.Z, 4, 0) = 0.0f;
	MAT_ELEMENT(ekf_14.Z, 5, 0) = -1.0f;
	MAT_ELEMENT(ekf_14.Z, 6, 0) = 1.0f;		// mag constant: [1, 0]
	MAT_ELEMENT(ekf_14.Z, 7, 0) = 0.0f;
	
	//EKF14_Prediction(&ekf_14);
	
	EKF14_SerialPrediction(&ekf_14, 0xFFFF);
	
	// store history state
	// for(uint8_t n = 0 ; n < 14 ; n++){
	// 	fifo_push(&_hist_x[n], MAT_ELEMENT(ekf_14.X, n, 0)); 
	// }
	
	// calculate delta state
//	float delta_x[14];
//	for(uint8_t n = 0 ; n < 14 ; n++){
//		// read history state
//		float hist_val;
//		if(_hist_x[n].cnt == _hist_x[n].size){ // read history data only fifo is full
//			hist_val = fifo_pop(&_hist_x[n]);
//		}else{
//			hist_val = MAT_ELEMENT(ekf_14.X, n, 0);
//		}
//		
//		delta_x[n] = MAT_ELEMENT(ekf_14.X, n, 0) - hist_val;
//		
//		// set current state to history value
//		MAT_ELEMENT(ekf_14.X, n, 0) = hist_val;
//	}
	
	if((acc[0] == 0.0f && acc[1] == 0.0f && acc[2] == 0.0f) || (mag[0] == 0.0f && mag[1] == 0.0f && mag[2] == 0.0f)){
		//EKF14_SerialCorrect(&ekf_14, enable);
		state_est_reset();
	}
	else{
		//EKF14_SerialCorrect(&ekf_14, enable);
		EKF14_Correct(&ekf_14);
	}
//	EKF14_SerialCorrect(&ekf_14, enable);
	
	// add delta state back
//	for(uint8_t n = 0 ; n < 14 ; n++){
//		MAT_ELEMENT(ekf_14.X, n, 0) += delta_x[n];
//	}
	
	state_est_get_quaternion(&_est_att_q);
	mcn_publish(MCN_ID(ATT_QUATERNION), &_est_att_q);

	quaternion_toEuler(&_est_att_q, &_est_att_e);
	mcn_publish(MCN_ID(ATT_EULER), &_est_att_e);

	Vector3f_t ned_pos, ned_vel;
	state_est_get_position(&ned_pos);
	state_est_get_velocity(&ned_vel);
	
	float accE[3];
	/* transfer acceleration from body frame to navigation frame */
	quaternion_rotateVector(&_est_att_q, acc, accE);	
	/* remove gravity */
	accE[2] += GRAVITY_MSS;
	
	Altitude_Info alt_info;
	alt_info.alt = -(ned_pos.z+home_pos.alt);
	alt_info.relative_alt = -ned_pos.z;
	alt_info.vz = -ned_vel.z;
	alt_info.az = -accE[2];
	alt_info.az_bias = -EKF14_Get_State(&ekf_14, STATE_AZ_BIAS);
	mcn_publish(MCN_ID(ALT_INFO), &alt_info);
	
	Position_Info pos_info;
	pos_info.x = ned_pos.x;
	pos_info.y = ned_pos.y;
	pos_info.vx = ned_vel.x;
	pos_info.vy = ned_vel.y;
	pos_info.ax = accE[0];
	pos_info.ay = accE[1];
	pos_info.ax_bias = 0.0f;	// TODO, no estimation bias for ax and ay
	pos_info.ay_bias = 0.0f;
	mcn_publish(MCN_ID(POS_INFO), &pos_info);

	return 0;
}
