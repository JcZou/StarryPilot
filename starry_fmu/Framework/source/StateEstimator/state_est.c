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

static EKF_Def ekf_14;

MCN_DECLARE(ATT_QUATERNION);
MCN_DECLARE(ATT_EULER);
MCN_DECLARE(BARO_POSITION);
MCN_DECLARE(ALT_INFO);

uint8_t state_est_init(float dT)
{
	EKF14_Init(&ekf_14, dT);
	
	return 0;
}

uint8_t state_est_reset(void)
{
	EKF14_Reset(&ekf_14);
	
	return 0;
}

uint8_t state_est_update(void)
{
	float acc[3], gyr[3];
	
	pos_try_sethome();
	
	sensor_get_acc(acc);
	sensor_get_gyr(gyr);
	
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
	}
	
	BaroPosition baro_pos;
	mcn_copy_from_hub(MCN_ID(BARO_POSITION), &baro_pos);
	
	if(home_pos.baro_altitude_set){
		pos.z = baro_pos.altitude - home_pos.alt;
		vel.z = baro_pos.velocity;
	}
	
	MAT_GET_ELEMENT(ekf_14.U, 0, 0) = gyr[0];
	MAT_GET_ELEMENT(ekf_14.U, 1, 0) = gyr[1];
	MAT_GET_ELEMENT(ekf_14.U, 2, 0) = gyr[2];
	MAT_GET_ELEMENT(ekf_14.U, 3, 0) = acc[0];
	MAT_GET_ELEMENT(ekf_14.U, 4, 0) = acc[1];
	MAT_GET_ELEMENT(ekf_14.U, 5, 0) = acc[2];
	
	MAT_GET_ELEMENT(ekf_14.Z, 0, 0) = pos.x;
	MAT_GET_ELEMENT(ekf_14.Z, 1, 0) = pos.y;
	MAT_GET_ELEMENT(ekf_14.Z, 2, 0) = pos.z;
	MAT_GET_ELEMENT(ekf_14.Z, 3, 0) = vel.x;
	MAT_GET_ELEMENT(ekf_14.Z, 4, 0) = vel.y;
	MAT_GET_ELEMENT(ekf_14.Z, 5, 0) = vel.z;
	MAT_GET_ELEMENT(ekf_14.Z, 6, 0) = 0.0f;		// acc constant: [0, 0, -1]
	MAT_GET_ELEMENT(ekf_14.Z, 7, 0) = 0.0f;
	MAT_GET_ELEMENT(ekf_14.Z, 8, 0) = -1.0f;
	MAT_GET_ELEMENT(ekf_14.Z, 9, 0) = 1.0f;		// mag constant: [1, 0]
	MAT_GET_ELEMENT(ekf_14.Z, 10, 0) = 0.0f;
	
	EKF14_Prediction(&ekf_14);
	EKF14_Correct(&ekf_14);
	
	quaternion att_q;
	state_est_get_quaternion(&att_q);
	mcn_publish(MCN_ID(ATT_QUATERNION), &att_q);
	
	Euler euler;
	quaternion_toEuler(att_q, &euler);
	mcn_publish(MCN_ID(ATT_EULER), &euler);
	
	Vector3f_t ned_pos, ned_vel;
	state_est_get_position(&ned_pos);
	state_est_get_velocity(&ned_vel);
	
	float accE[3];
	/* transfer acceleration from body frame to navigation frame */
	quaternion_rotateVector(att_q, acc, accE);	
	/* remove gravity */
	accE[2] += 9.8f;
	
	AltInfo alt_info;
	alt_info.alt = -(ned_pos.z+home_pos.alt);
	alt_info.relative_alt = -ned_pos.z;
	alt_info.vz = -ned_vel.z;
	alt_info.az = -accE[2];
	alt_info.az_bias = -EKF14_Get_State(&ekf_14, STATE_AZ_BIAS);
	mcn_publish(MCN_ID(ALT_INFO), &alt_info);

	return 0;
}

void state_est_get_quaternion(quaternion* q)
{
	q->w = EKF14_Get_State(&ekf_14, STATE_Q0);
	q->x = EKF14_Get_State(&ekf_14, STATE_Q1);
	q->y = EKF14_Get_State(&ekf_14, STATE_Q2);
	q->z = EKF14_Get_State(&ekf_14, STATE_Q3);
}

void state_est_get_position(Vector3f_t *pos)
{
	pos->x = EKF14_Get_State(&ekf_14, STATE_X);
	pos->y = EKF14_Get_State(&ekf_14, STATE_Y);
	pos->z = EKF14_Get_State(&ekf_14, STATE_Z);
}

void state_est_get_velocity(Vector3f_t *vel)
{
	vel->x = EKF14_Get_State(&ekf_14, STATE_VX);
	vel->y = EKF14_Get_State(&ekf_14, STATE_VY);
	vel->z = EKF14_Get_State(&ekf_14, STATE_VZ);
}
