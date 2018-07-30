/*
 * File      : adrc_att.c
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-04-09     zoujiachi    first version.
 */
 
#include "adrc_att.h"
#include "param.h"
#include "filter.h"
#include "att_pid.h"
#include "global.h"
#include "console.h"
#include "uMCN.h"
#include "ap_math.h"

MCN_DECLARE(ADRC);

#define sin_45 0.70711f
#define cR     718.078f
#define cT     1.23884e-5
#define L      0.255f
#define d      88.448f
#define Ixx_yy 0.016f
static float b_const = 8.0f*cR*cT*L*sin_45;
static float int_i[2] = {0.0f, 0.0f};
static TD_Controller_Def _roll_td_controller;
static TD_Controller_Def _pitch_td_controller;
static ADRC_TD_Def _roll_td;
static ADRC_TD_Def _pitch_td;
static ADRC_NLSEF_Def _roll_nlsef;
static ADRC_NLSEF_Def _pitch_nlsef;
static uint8_t _outerloop_update = 1;
ADRC_LESO_Def _roll_leso;
ADRC_LESO_Def _pitch_leso;

Delay_Block roll_leso_delay;
Delay_Block pitch_leso_delay;

uint8_t _delay_block_create(Delay_Block *block, uint16_t size)
{
	block->data = (float*)OS_MALLOC(size*sizeof(float));
	if(block->data == NULL){
		Console.print("delay block create fail\n");
		return 1;
	}
	block->size = size;
	block->head = 0;
	for(int i = 0 ; i < size ; i++){
		block->data[i] = 0.0f;
	}
	
	return 0;
}

void _delay_block_flush(Delay_Block *block)
{
	if(block == NULL)
		return;
	block->head = 0;
	for(int i = 0 ; i < block->size ; i++){
		block->data[i] = 0.0f;
	}
}

void _delay_block_push(Delay_Block *block, float val)
{
	block->head = (block->head+1) % block->size;
	block->data[block->head] = val;
}

float _delay_block_pop(Delay_Block *block)
{
	uint16_t tail = (block->head+1) % block->size;
	return block->data[tail];
}

void adrc_att_init(float h)
{
//	_derivative[0] = _derivative[1] = 0.0f;
//	pre_err[0] = pre_err[1] = 0.0f;
//	d_alpha = lpf_get_alpha(50, dt);
	
	adrc_td_control_init(&_roll_td_controller, h, PARAM_GET_FLOAT(ADRC_ATT, TD_CONTROL_R2), PARAM_GET_FLOAT(ADRC_ATT, TD_CONTROL_H2F)*h);
	adrc_td_control_init(&_pitch_td_controller, h, PARAM_GET_FLOAT(ADRC_ATT, TD_CONTROL_R2), PARAM_GET_FLOAT(ADRC_ATT, TD_CONTROL_H2F)*h);

	adrc_td_init(&_roll_td, h, PARAM_GET_FLOAT(ADRC_ATT, TD_R0), h);
	adrc_td_init(&_pitch_td, h, PARAM_GET_FLOAT(ADRC_ATT, TD_R0), h);
	
	/* LESO run faster than other components */
	adrc_leso_init(&_roll_leso, 0.001f, PARAM_GET_FLOAT(ADRC_ATT, LESO_W), 400);
	adrc_leso_init(&_pitch_leso, 0.001f, PARAM_GET_FLOAT(ADRC_ATT, LESO_W), 400);

	adrc_nlsef_init(&_roll_nlsef, h, PARAM_GET_FLOAT(ADRC_ATT, NLSEF_R1), PARAM_GET_FLOAT(ADRC_ATT, NLSEF_H1F)*h,
						PARAM_GET_FLOAT(ADRC_ATT, NLSEF_C));
	adrc_nlsef_init(&_pitch_nlsef, h, PARAM_GET_FLOAT(ADRC_ATT, NLSEF_R1), PARAM_GET_FLOAT(ADRC_ATT, NLSEF_H1F)*h,
						PARAM_GET_FLOAT(ADRC_ATT, NLSEF_C));
	
	// delay time = (size-1)*sample_time
#ifdef HIL_SIMULATION
	// there is no filtering for HIL simulation
	_delay_block_create(&roll_leso_delay, 1);
	_delay_block_create(&pitch_leso_delay, 1);
#else
//	_delay_block_create(&roll_leso_delay, 5);
//	_delay_block_create(&pitch_leso_delay, 5);
	_delay_block_create(&roll_leso_delay, 3);
	_delay_block_create(&pitch_leso_delay, 3);
#endif
	
	_outerloop_update = 1;
	
	int_i[0] = int_i[1] = 0.0f;
}

void adrc_att_reset(float h)
{
	adrc_td_control_init(&_roll_td_controller, h, PARAM_GET_FLOAT(ADRC_ATT, TD_CONTROL_R2), PARAM_GET_FLOAT(ADRC_ATT, TD_CONTROL_H2F)*h);
	adrc_td_control_init(&_pitch_td_controller, h, PARAM_GET_FLOAT(ADRC_ATT, TD_CONTROL_R2), PARAM_GET_FLOAT(ADRC_ATT, TD_CONTROL_H2F)*h);

	adrc_td_init(&_roll_td, h, PARAM_GET_FLOAT(ADRC_ATT, TD_R0), h);
	adrc_td_init(&_pitch_td, h, PARAM_GET_FLOAT(ADRC_ATT, TD_R0), h);
	
	/* LESO run faster than other components */
	adrc_leso_init(&_roll_leso, 0.001f, PARAM_GET_FLOAT(ADRC_ATT, LESO_W), PARAM_GET_FLOAT(ADRC_ATT, B0));
	adrc_leso_init(&_pitch_leso, 0.001f, PARAM_GET_FLOAT(ADRC_ATT, LESO_W), PARAM_GET_FLOAT(ADRC_ATT, B0));

	adrc_nlsef_init(&_roll_nlsef, h, PARAM_GET_FLOAT(ADRC_ATT, NLSEF_R1), PARAM_GET_FLOAT(ADRC_ATT, NLSEF_H1F)*h,
						PARAM_GET_FLOAT(ADRC_ATT, NLSEF_C));
	adrc_nlsef_init(&_pitch_nlsef, h, PARAM_GET_FLOAT(ADRC_ATT, NLSEF_R1), PARAM_GET_FLOAT(ADRC_ATT, NLSEF_H1F)*h,
						PARAM_GET_FLOAT(ADRC_ATT, NLSEF_C));
	
	_delay_block_flush(&roll_leso_delay);
	_delay_block_flush(&pitch_leso_delay);
	
	_outerloop_update = 1;
	
	int_i[0] = int_i[1] = 0.0f;
}

void adrc_att_dis_comp(float* in, float* out)
{
	float gamma = PARAM_GET_FLOAT(ADRC_ATT, GAMMA);
	
	out[0] = in[0] - gamma*_roll_leso.z2/_roll_leso.b0;
	out[1] = in[1] - gamma*_pitch_leso.z2/_pitch_leso.b0;
	
	// constrain output
	constrain(&out[0], -0.5f, 0.5f);
	constrain(&out[1], -0.5f, 0.5f);
	
	// delay control signal
	_delay_block_push(&roll_leso_delay, out[0]);
	_roll_leso.u = _delay_block_pop(&roll_leso_delay);
	_delay_block_push(&pitch_leso_delay, out[1]);
	_pitch_leso.u = _delay_block_pop(&pitch_leso_delay);
}

extern ADRC_Log adrc_log;
void adrc_att_control(float err[3], const float gyr[3], float out[3], float bth)
{
	float sp_rate[3];
	float rate_err[3];
	float u0[2];
	//static int flag = 1;
	
	/* TD control generates target rotational velocity */
	sp_rate[0] = adrc_td_control(&_roll_td_controller, err[0]);
	sp_rate[1] = adrc_td_control(&_pitch_td_controller, err[1]);
	rate_err[0] = sp_rate[0] - gyr[0];
	rate_err[1] = sp_rate[1] - gyr[1];
	
	/* control law */
	// TD extracts derivative of error
	adrc_td(&_roll_td, rate_err[0]);
	adrc_td(&_pitch_td, rate_err[1]);
	// NLSEF control
	u0[0] = adrc_nlsef(&_roll_nlsef, rate_err[0], _roll_td.v2)/_roll_leso.b0;
	u0[1] = adrc_nlsef(&_pitch_nlsef, rate_err[1], _pitch_td.v2)/_pitch_leso.b0;
	// integral action
	float ki = PARAM_GET_FLOAT(ADRC_ATT, NLSEF_KI);
	if(IN_RANGE(int_i[0], -0.1f, 0.1f))
		int_i[0] += rate_err[0] * ki * _roll_nlsef.h;
	if(IN_RANGE(int_i[1], -0.1f, 0.1f))
		int_i[1] += rate_err[1] * ki * _pitch_nlsef.h;
	u0[0] += int_i[0];
	u0[1] += int_i[1];
	// constrain output
	constrain(&u0[0], -0.5f, 0.5f);
	constrain(&u0[1], -0.5f, 0.5f);
	
	/* disturbance rejection */
	adrc_att_dis_comp(u0, out);

	/* yaw axis control uses pid */
	att_yaw_pid_control(Rad2Deg(err[2]), &out[2], gyr[2], _pitch_nlsef.h, bth);
	
	//ADRC_Log adrc_log;
	adrc_log.sp_rate = sp_rate[1];
//	adrc_log.v = rate_err[1];
	adrc_log.v1 = Rad2Deg(_pitch_td_controller.v1);
	adrc_log.v2 = _pitch_td_controller.v2;
	adrc_log.z1 = _pitch_leso.z1;
	adrc_log.z2 = _pitch_leso.z2;
	mcn_publish(MCN_ID(ADRC), &adrc_log);
}

void adrc_att_observer_update(const float gyr[3], float bth)
{
	/* update b0 */
	float bt = bth;
	// do not let base_throttle to be too small or too large
	constrain(&bt, 0.3f, 0.7f);
	float b0 = b_const*(cR*bt+d)/Ixx_yy;
	//float b0 = PARAM_GET_FLOAT(ADRC_ATT, B0);
	_pitch_leso.b0 = b0;
	_roll_leso.b0 = b0;
	
	/* observer update */
	adrc_leso(&_roll_leso, gyr[0]);
	adrc_leso(&_pitch_leso, gyr[1]);
}