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

typedef struct {
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
} PARAM_Def;

typedef union {
	int8_t		i8;
	uint8_t		u8;
	int16_t		i16;
	uint16_t	u16;
	int32_t		i32;
	uint32_t	u32;
	float		f;
	double		lf;
} param_value_t;

typedef enum {
	PARAM_TYPE_INT8 = 0,
	PARAM_TYPE_UINT8,
	PARAM_TYPE_INT16,
	PARAM_TYPE_UINT16,
	PARAM_TYPE_INT32,
	PARAM_TYPE_UINT32,
	PARAM_TYPE_FLOAT,
	PARAM_TYPE_DOUBLE,
	PARAM_TYPE_UNKNOWN = 0xffff
} param_type_t;

typedef struct {
	const char* name;
	const param_type_t type;
	param_value_t val;
} param_info_t;

#define PARAM_DECLARE(_name)					param_info_t _name

#define PARAM_DEFINE_INT8(_name, _default) \
			{ \
				.name = #_name, \
				.type = PARAM_TYPE_INT8, \
				.val.i8 = _default \
			}

#define PARAM_DEFINE_UINT8(_name, _default) \
			{ \
				.name = #_name, \
				.type = PARAM_TYPE_UINT8, \
				.val.u8 = _default \
			}

#define PARAM_DEFINE_INT16(_name, _default) \
			{ \
				.name = #_name, \
				.type = PARAM_TYPE_INT16, \
				.val.i16 = _default \
			}

#define PARAM_DEFINE_UINT16(_name, _default) \
			{ \
				.name = #_name, \
				.type = PARAM_TYPE_UINT16, \
				.val.u16 = _default \
			}

#define PARAM_DEFINE_INT32(_name, _default) \
			{ \
				.name = #_name, \
				.type = PARAM_TYPE_INT32, \
				.val.i32 = _default \
			}

#define PARAM_DEFINE_UINT32(_name, _default) \
			{ \
				.name = #_name, \
				.type = PARAM_TYPE_UINT32, \
				.val.u32 = _default \
			}

#define PARAM_DEFINE_FLOAT(_name, _default) \
			{ \
				.name = #_name, \
				.type = PARAM_TYPE_FLOAT, \
				.val.f = _default \
			}

#define PARAM_DEFINE_DOUBLE(_name, _default) \
			{ \
				.name = #_name, \
				.type = PARAM_TYPE_DOUBLE, \
				.val.lf = _default \
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
typedef struct {
	PARAM_DECLARE(GYR_BIAS_X);
	PARAM_DECLARE(GYR_BIAS_Y);
	PARAM_DECLARE(GYR_BIAS_Z);
	PARAM_DECLARE(GYR_X_GAIN);
	PARAM_DECLARE(GYR_Y_GAIN);
	PARAM_DECLARE(GYR_Z_GAIN);
	PARAM_DECLARE(GYR_CALIB);
	PARAM_DECLARE(ACC_BIAS_X);
	PARAM_DECLARE(ACC_BIAS_Y);
	PARAM_DECLARE(ACC_BIAS_Z);
	PARAM_DECLARE(ACC_ROT_MAT_1);
	PARAM_DECLARE(ACC_ROT_MAT_2);
	PARAM_DECLARE(ACC_ROT_MAT_3);
	PARAM_DECLARE(ACC_ROT_MAT_4);
	PARAM_DECLARE(ACC_ROT_MAT_5);
	PARAM_DECLARE(ACC_ROT_MAT_6);
	PARAM_DECLARE(ACC_ROT_MAT_7);
	PARAM_DECLARE(ACC_ROT_MAT_8);
	PARAM_DECLARE(ACC_ROT_MAT_9);
	PARAM_DECLARE(ACC_CALIB);
	PARAM_DECLARE(MAG_BIAS_X);
	PARAM_DECLARE(MAG_BIAS_Y);
	PARAM_DECLARE(MAG_BIAS_Z);
	PARAM_DECLARE(MAG_ROT_MAT_1);
	PARAM_DECLARE(MAG_ROT_MAT_2);
	PARAM_DECLARE(MAG_ROT_MAT_3);
	PARAM_DECLARE(MAG_ROT_MAT_4);
	PARAM_DECLARE(MAG_ROT_MAT_5);
	PARAM_DECLARE(MAG_ROT_MAT_6);
	PARAM_DECLARE(MAG_ROT_MAT_7);
	PARAM_DECLARE(MAG_ROT_MAT_8);
	PARAM_DECLARE(MAG_ROT_MAT_9);
	PARAM_DECLARE(MAG_CALIB);
} PARAM_GROUP(CALIBRATION);

typedef struct {
	PARAM_DECLARE(LOG_AUTO_START);
} PARAM_GROUP(SYSTEM);
/* Parameter Declare End */

#define PARAM_GET(_group, _name)				((_param_##_group *)(param_list._param_##_group.content))->_name
#define PARAM_GET_INT8(_group, _name)			((_param_##_group *)(param_list._param_##_group.content))->_name.val.i8
#define PARAM_GET_UINT8(_group, _name)			((_param_##_group *)(param_list._param_##_group.content))->_name.val.u8
#define PARAM_GET_INT16(_group, _name)			((_param_##_group *)(param_list._param_##_group.content))->_name.val.i16
#define PARAM_GET_UINT16(_group, _name)			((_param_##_group *)(param_list._param_##_group.content))->_name.val.u16
#define PARAM_GET_INT32(_group, _name)			((_param_##_group *)(param_list._param_##_group.content))->_name.val.i32
#define PARAM_GET_UINT32(_group, _name)			((_param_##_group *)(param_list._param_##_group.content))->_name.val.u32
#define PARAM_GET_FLOAT(_group, _name)			((_param_##_group *)(param_list._param_##_group.content))->_name.val.f
#define PARAM_GET_DOUBLE(_group, _name)			((_param_##_group *)(param_list._param_##_group.content))->_name.val.lf

#define PARAM_SET_INT8(_group, _name, _val)		((_param_##_group *)(param_list._param_##_group.content))->_name.val.i8 = _val
#define PARAM_SET_UINT8(_group, _name, _val)	((_param_##_group *)(param_list._param_##_group.content))->_name.val.u8 = _val
#define PARAM_SET_INT16(_group, _name, _val)	((_param_##_group *)(param_list._param_##_group.content))->_name.val.i16 = _val
#define PARAM_SET_UINT16(_group, _name, _val)	((_param_##_group *)(param_list._param_##_group.content))->_name.val.u16 = _val
#define PARAM_SET_INT32(_group, _name, _val)	((_param_##_group *)(param_list._param_##_group.content))->_name.val.i32 = _val
#define PARAM_SET_UINT32(_group, _name, _val)	((_param_##_group *)(param_list._param_##_group.content))->_name.val.u32 = _val
#define PARAM_SET_FLOAT(_group, _name, _val)	((_param_##_group *)(param_list._param_##_group.content))->_name.val.f = _val
#define PARAM_SET_DOUBLE(_group, _name, _val)	((_param_##_group *)(param_list._param_##_group.content))->_name.val.lf = _val

typedef struct {
	const char* name;
	const uint32_t param_num;
	param_info_t* content;
} param_group_info;

/* step 2: param list declare */
typedef struct {
	param_group_info	PARAM_GROUP(CALIBRATION);
	param_group_info	PARAM_GROUP(SYSTEM);
} param_list_t;

extern param_list_t param_list;

uint8_t param_init(void);
const PARAM_Def* get_param(void);
void param_release(void);

param_info_t* param_get(char* group_name, char* param_name);
param_info_t* param_get_by_name(char* param_name);
void param_traverse(void (*param_ops)(param_info_t* param));
uint32_t param_get_info_count(void);
uint32_t param_get_info_index(char* param_name);
int param_set_by_info(param_info_t* param, float val);
int param_get_by_info(param_info_t* param, float* val);

#endif
