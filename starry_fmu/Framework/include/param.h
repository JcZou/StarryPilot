/*
 * File      : param.h
 *
 *
 * Change Logs:
 * Date           Author       	Notes
 * 2016-07-01     zoujiachi   	the first version
 */
 
#ifndef __PARAM_H__
#define __PARAM_H__

#include <stm32f4xx.h>
#include "quaternion.h"

#define PARAM_MAX_NAME_LEN

typedef struct
{
	/* firmware version */
	uint16_t version;
	
	/* sensor compensate param */
	float accOffset[3];
	float accTransMat[3][3];
	float magOffset[3];
	float magTransMat[3][3];
	float gyr_offset[3];
	float gyr_gain[3];

	/* attitude control pid param */
	float att_angle_p[3];
	float att_angle_i[3];
	float att_angle_d[3];
	float att_rate_p[3];
	float att_rate_i[3];
	float att_rate_d[3];
	/* attitude integrate limit */
	float att_i_limit[3];
	/* attitude rate integrate limit */
	float att_rate_i_limit[3];
	/* attitude control output limit */
	float att_control_limit[3];
	
	/* altitude control pid param */
	float alt_p;
	float alt_i;
	float alt_d;
	float alt_vel_p;
	float alt_vel_i;
	float alt_vel_d;
	float alt_acc_p;
	float alt_acc_i;
	float alt_acc_d;
}PARAM_Def;

typedef union {
	int32_t		i;
	uint32_t	u;
	float		f;
}param_value_t;

typedef enum{
	PARAM_TYPE_INT32 = 0,
	PARAM_TYPE_UINT32,
	PARAM_TYPE_FLOAT,
	PARAM_TYPE_UNKNOWN = 0xffff
} param_type_t;

typedef struct{
	const char* name;
	const param_type_t type;
	param_value_t val;
}param_info_t;

#define PARAM_DECLARE(_name)					param_info_t _name
#define PARAM_DEFINE_INT32(_name, _default) \
			{ \
				.name = #_name, \
				.type = PARAM_TYPE_INT32, \
				.val.i = _default \
			}
			
#define PARAM_DEFINE_UINT32(_name, _default) \
			{ \
				.name = #_name, \
				.type = PARAM_TYPE_UINT32, \
				.val.u = _default \
			}
			
#define PARAM_DEFINE_FLOAT(_name, _default) \
			{ \
				.name = #_name, \
				.type = PARAM_TYPE_FLOAT, \
				.val.f = _default \
			}

#define PARAM_GROUP(_group)						_param_##_group
#define PARAM_DECLARE_GROUP(_group)				_param_##_group##_t
#define PARAM_DEFINE_GROUP(_group) \
			{ \
				.name = #_group, \
				.param_num = sizeof(_param_##_group##_t)/sizeof(param_info_t), \
				.content = (param_info_t*)&_param_##_group##_t \
			}

/* step 1: Parameter Declare */	
typedef struct
{
	PARAM_DECLARE(GYR_X_OFFSET);
	PARAM_DECLARE(GYR_Y_OFFSET);
	PARAM_DECLARE(GYR_Z_OFFSET);
	PARAM_DECLARE(GYR_X_GAIN);
	PARAM_DECLARE(GYR_Y_GAIN);
	PARAM_DECLARE(GYR_Z_GAIN);
	PARAM_DECLARE(GYR_CALIB);
	PARAM_DECLARE(ACC_X_OFFSET);
	PARAM_DECLARE(ACC_Y_OFFSET);
	PARAM_DECLARE(ACC_Z_OFFSET);
	PARAM_DECLARE(ACC_TRANS_MAT00);
	PARAM_DECLARE(ACC_TRANS_MAT01);
	PARAM_DECLARE(ACC_TRANS_MAT02);
	PARAM_DECLARE(ACC_TRANS_MAT10);
	PARAM_DECLARE(ACC_TRANS_MAT11);
	PARAM_DECLARE(ACC_TRANS_MAT12);
	PARAM_DECLARE(ACC_TRANS_MAT20);
	PARAM_DECLARE(ACC_TRANS_MAT21);
	PARAM_DECLARE(ACC_TRANS_MAT22);
	PARAM_DECLARE(ACC_CALIB);
	PARAM_DECLARE(MAG_X_OFFSET);
	PARAM_DECLARE(MAG_Y_OFFSET);
	PARAM_DECLARE(MAG_Z_OFFSET);
	PARAM_DECLARE(MAG_TRANS_MAT00);
	PARAM_DECLARE(MAG_TRANS_MAT01);
	PARAM_DECLARE(MAG_TRANS_MAT02);
	PARAM_DECLARE(MAG_TRANS_MAT10);
	PARAM_DECLARE(MAG_TRANS_MAT11);
	PARAM_DECLARE(MAG_TRANS_MAT12);
	PARAM_DECLARE(MAG_TRANS_MAT20);
	PARAM_DECLARE(MAG_TRANS_MAT21);
	PARAM_DECLARE(MAG_TRANS_MAT22);
	PARAM_DECLARE(MAG_CALIB);
}PARAM_GROUP(CALIBRATION);
			
typedef struct
{
	PARAM_DECLARE(ATT_ROLL_P);
	PARAM_DECLARE(ATT_ROLL_RATE_P);
	PARAM_DECLARE(ATT_ROLL_RATE_I);
	PARAM_DECLARE(ATT_ROLL_RATE_D);
	PARAM_DECLARE(ATT_PITCH_P);
	PARAM_DECLARE(ATT_PITCH_RATE_P);
	PARAM_DECLARE(ATT_PITCH_RATE_I);
	PARAM_DECLARE(ATT_PITCH_RATE_D);
	PARAM_DECLARE(ATT_YAW_P);
	PARAM_DECLARE(ATT_YAW_RATE_P);
	PARAM_DECLARE(ATT_YAW_RATE_I);
	PARAM_DECLARE(ATT_YAW_RATE_D);
	PARAM_DECLARE(ATT_ROLLOUT_LIM);
	PARAM_DECLARE(ATT_PITCHOUT_LIM);
	PARAM_DECLARE(ATT_YAWOUT_LIM);
	PARAM_DECLARE(ATT_ROLLR_I_LIM);
	PARAM_DECLARE(ATT_PITCHR_I_LIM);
	PARAM_DECLARE(ATT_YAWR_I_LIM);
}PARAM_GROUP(ATT_CONTROLLER);

typedef struct
{
	PARAM_DECLARE(ALT_P);
	PARAM_DECLARE(ALT_RATE_P);
	PARAM_DECLARE(ALT_ACC_P);
	PARAM_DECLARE(ALT_ACC_I);
	PARAM_DECLARE(ALT_ACC_D);
	PARAM_DECLARE(ALT_ERR_MIN);
	PARAM_DECLARE(ALT_ERR_MAX);
	PARAM_DECLARE(VEL_ERR_MIN);
	PARAM_DECLARE(VEL_ERR_MAX);
	PARAM_DECLARE(ACC_ERR_MIN);
	PARAM_DECLARE(ACC_ERR_MAX);
	PARAM_DECLARE(ALT_OUTPUT_MIN);
	PARAM_DECLARE(ALT_OUTPUT_MAX);
	PARAM_DECLARE(VEL_OUTPUT_MIN);
	PARAM_DECLARE(VEL_OUTPUT_MAX);
	PARAM_DECLARE(ACC_OUTPUT_MIN);
	PARAM_DECLARE(ACC_OUTPUT_MAX);
	PARAM_DECLARE(ACC_I_MIN);
	PARAM_DECLARE(ACC_I_MAX);
	PARAM_DECLARE(FEEDFORWARD_EN);
	PARAM_DECLARE(ACC_ERR_LPF_EN);
	PARAM_DECLARE(ACC_ERR_LPF_FREQ);
}PARAM_GROUP(ALT_CONTROLLER);

typedef struct
{
	PARAM_DECLARE(ADRC_ENABLE);
	PARAM_DECLARE(ADRC_MODE);
	PARAM_DECLARE(TD_CONTROL_R2);
	PARAM_DECLARE(TD_CONTROL_H2F);
	PARAM_DECLARE(TD_R0);
	PARAM_DECLARE(NLSEF_R1);
	PARAM_DECLARE(NLSEF_H1F);
	PARAM_DECLARE(NLSEF_C);
	PARAM_DECLARE(NLSEF_KI);
	PARAM_DECLARE(LESO_W);
//	PARAM_DECLARE(ADRC_BETA1);
//	PARAM_DECLARE(ADRC_BETA2);
//	PARAM_DECLARE(ADRC_ALPHA);
//	PARAM_DECLARE(ADRC_DELTA);
	PARAM_DECLARE(T_UP);
	PARAM_DECLARE(T_DOWN);
	PARAM_DECLARE(GAMMA);
	PARAM_DECLARE(B0);
}PARAM_GROUP(ADRC_ATT);

typedef struct
{
	PARAM_DECLARE(HIL_ATT_EST_PRD);
	PARAM_DECLARE(HIL_POS_EST_PRD);
	PARAM_DECLARE(HIL_CONTROL_PRD);
}PARAM_GROUP(HIL_SIM);
/* Parameter Declare End */		

#define PARAM_GET(_group, _name)				((_param_##_group *)(param_list._param_##_group.content))->_name
#define PARAM_GET_INT32(_group, _name)			((_param_##_group *)(param_list._param_##_group.content))->_name.val.i
#define PARAM_GET_UINT32(_group, _name)			((_param_##_group *)(param_list._param_##_group.content))->_name.val.u
#define PARAM_GET_FLOAT(_group, _name)			((_param_##_group *)(param_list._param_##_group.content))->_name.val.f

#define PARAM_SET_INT32(_group, _name, _val)	((_param_##_group *)(param_list._param_##_group.content))->_name.val.i = _val
#define PARAM_SET_UINT32(_group, _name, _val)	((_param_##_group *)(param_list._param_##_group.content))->_name.val.u = _val
#define PARAM_SET_FLOAT(_group, _name, _val)	((_param_##_group *)(param_list._param_##_group.content))->_name.val.f = _val

typedef struct{
	const char* name;
	const uint32_t param_num;
	param_info_t* content;
}param_group_info;

/* step 2: param list declare */
typedef struct{
	param_group_info	PARAM_GROUP(CALIBRATION);
	param_group_info	PARAM_GROUP(ATT_CONTROLLER);
	param_group_info	PARAM_GROUP(ALT_CONTROLLER);
	param_group_info	PARAM_GROUP(ADRC_ATT);
	param_group_info	PARAM_GROUP(HIL_SIM);
}param_list_t;

extern param_list_t param_list;

uint8_t param_init(void);
const PARAM_Def * get_param(void);
void param_release(void);

param_info_t* param_get(char* group_name, char* param_name);
param_info_t* param_get_by_name(char* param_name);
void param_traverse(void (*param_ops)(param_info_t* param));
uint32_t param_get_info_count(void);
uint32_t param_get_info_index(char* param_name);
int param_set_by_info(param_info_t* param, float val);
int param_get_by_info(param_info_t* param, float *val);

#endif
