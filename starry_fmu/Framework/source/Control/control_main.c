/*
 * File      : control.c
 *
 * Change Logs:
 * Date           Author       Notes
 * 2017-02-25     zoujiachi    first version.
 */
 
//#include <rthw.h>
//#include <rtdevice.h>
#include <math.h>
#include "rc.h"
#include "pwm.h"
#include "param.h"
#include "control_main.h"
#include "quaternion.h"
#include "att_estimator.h"
#include "motor.h"
#include "global.h"
#include "console.h"
#include "filter.h"
#include "att_pid.h"
#include "starryio_manager.h"
#include "control_alt.h"
#include "pos_estimator.h"
#include "ap_math.h"
#include "sensor_manager.h"
#include "delay.h"
#include "uMCN.h"
#include "copter_main.h"
#include "mavproxy.h"
#include "adrc.h"
#include "copter_main.h"
#include "adrc_att.h"
#include "gps.h"

#define EVENT_CONTROL			(1<<0)

/* Definition */
#define ATT_CTRL_INTERVAL		10
#define ALT_CTRL_INTERVAL		10
/* the control angle for rc(degree) */
#define ANGLE_CONTROL_SCALE		(20.0f)
/* the maximal yaw rotation speed 20 deg/s */
#define YAW_CONTROL_SPEED		(90.0f)
#define THROTTLE_MIN			0.0f
#define THROTTLE_MAX			0.9f
#define THROTTLE_DEAD_ZONE		0.1f
/* maximal altitude rate, 2 m/s */
#define MAX_ALT_RATE			2.0f
/* maximal altitude 2 m */
#ifdef HIL_SIMULATION
	#define MAX_ALTITUDE			100.0f
#else
	#define MAX_ALTITUDE			10.0f
#endif
/* attitude control dead zone */
#define ATT_CTRL_DEADZONE		0.05f

struct	Control_Req_Param
{
	float base_throttle;	/* 0.0~1.0 */
	float roll_sp;			/* deg */
	float pitch_sp;			/* deg */
}_ctrl_req_param;

static bool _request_control = false;
static rt_device_t motor_device_t;
static float yaw_target;
static Euler _att_et;
/* 0:lock	1:unlock*/
static float _vehicle_status = 0;
static float _throttle_out[MOTOR_NUM];
/* in attitude control mode, the yaw is locked by default */
static uint8_t att_mode_lock_yaw = 0;
/* control mode. 1:attitude control mode	2:altitude hold mode */
static uint8_t _control_mode = 1;
static float _baseThrottle = 0.0f;
static float _throttle_alpha = 0.0f;	//throttle LPF
static float _throttle_lpf = 0.0f;
static uint8_t alt_hold_mode = 0;
static float alt_setpoint = 0.0f;
static uint8_t _att_outerloop_update = 1;
Euler _ec;	//current euler angle
HomePosition _home = {0.0f, 0.0f, 0};	// home position

#ifdef BLUEJAY
FrameType _frame_type = frame_type_4;
#else
FrameType _frame_type = frame_type_1;	/* the default frame type is X frame */
#endif

static char* TAG = "Control";

MCN_DEFINE(MOTOR_THROTTLE, MOTOR_NUM*sizeof(float));
MCN_DEFINE(ADRC, sizeof(ADRC_Log));

MCN_DECLARE(ATT_EULER);
MCN_DECLARE(ALT_INFO);
McnNode_t alt_node_t;

ADRC_TD_Def throttle_td[MOTOR_NUM];

void ctrl_throttle_set_lpf(float cutoff_freq, float dt)
{
	if(cutoff_freq > 0.0f && dt > 0.0f){
		_throttle_alpha = lpf_get_alpha(cutoff_freq, dt);
	}
}

float ctrl_get_boosted_throttle(float throttle_in)
{
	// inverted_factor is 1 for tilt angles below 60 degrees
    // inverted_factor reduces from 1 to 0 for tilt angles between 60 and 90 degrees
	
	//quaternion att = attitude_est_get_quaternion();
	Euler e = _ec;
	//quaternion_toEuler(att, &e);
	
	float cos_tilt = arm_cos_f32(e.roll)*arm_cos_f32(e.pitch);
    float inverted_factor = constrain_float(2.0f*cos_tilt, 0.0f, 1.0f);
    float boost_factor = 1.0f/constrain_float(cos_tilt, 0.5f, 1.0f);

    float throttle_out = throttle_in*inverted_factor*boost_factor;
	
	//TODO: constrain the throttle_out here?
    return throttle_out;
}

void ctrl_set_basethrottle(float throttle)
{
	/* throttle low pass filter */
	if(_throttle_alpha > 0.0f){
		_throttle_lpf = _throttle_lpf + (throttle - _throttle_lpf) * _throttle_alpha;
		_baseThrottle = _throttle_lpf;
	}
	
	_baseThrottle = throttle;
}

float ctrl_get_basethrottle(void)
{
	float out_throttle = _baseThrottle;
	/* altitude mode or position mode */
	if(_control_mode == 2 || _control_mode ==3){
		/* throttle tilt compensation */
		out_throttle = ctrl_get_boosted_throttle(_baseThrottle);
	}
	
	return out_throttle;
}

int _switch_control_mode(float mode_chan_val)
{
	int mode_change = 0;
	
	/* attitude control mode */
	if(mode_chan_val < 0.1 && _control_mode != 1){
		_control_mode = 1;
		mode_change = 1;
		Console.print("change to attitude mode\n");
	}
	
	/* altitude hold mode */
	if(mode_chan_val > 0.4 && mode_chan_val < 0.6 && _control_mode != 2){
		_control_mode = 2;
		mode_change = 1;
		Console.print("change to altitude hold mode\n");
	}
	
	/* position control mode */
	if(mode_chan_val > 0.9 && _control_mode != 3){
		_control_mode = 3;
		mode_change = 1;
		Console.print("change to position mode\n");
	}
	
	return mode_change;
}

void ctrl_constrain_throttle(float* throttle, uint8_t throttle_num)
{
	for(int i = 0 ; i < throttle_num ; i++){
		if(throttle[i] < 0.0f)
			throttle[i] = 0.0f;
		if(throttle[i] > 0.9f)
			throttle[i] = 0.9f;
	}
}

void _ctrl_mix_throttle_out(float *out, float* in, float base_throttle)
{
	if(_frame_type == frame_type_1){
		/* X frame */
		/* -1	 1	 1 */
		/*  1	-1	 1 */
		/*  1	 1	-1 */
		/* -1	-1	-1 */
		out[0] = base_throttle - in[0] + in[1] + in[2];
		out[1] = base_throttle + in[0] - in[1] + in[2];
		out[2] = base_throttle + in[0] + in[1] - in[2];
		out[3] = base_throttle - in[0] - in[1] - in[2];		
	}else if(_frame_type == frame_type_2){
		/* + frame */
		/* -sqrt(2)	  0	 		sqrt(2) */
		/*  sqrt(2)	  0	 		sqrt(2) */
		/*  0	 	sqrt(2)		-sqrt(2) */
		/*  0		-sqrt(2)	-sqrt(2) */
		out[0] = base_throttle - 1.414f*in[0]          			+ 1.414f*in[2];
		out[1] = base_throttle + 1.414f*in[0]          			+ 1.414f*in[2];
		out[2] = base_throttle          		+ 1.414f*in[1] 	- 1.414f*in[2];
		out[3] = base_throttle          		- 1.414f*in[1] 	- 1.414f*in[2];	
	}else if(_frame_type == frame_type_3){
		/* hexrotor frame */
		/*      0	 	  1	 	-1 */
		/*      0	 	 -1		 1 */
		/*  sqrt(3)/2	-1/2	-1 */
		/* -sqrt(3)/2	 1/2	 1 */
		/*  sqrt(3)/2	 1/2	 1 */
		/* -sqrt(3)/2	-1/2	-1 */
		out[0] = base_throttle                 +      in[1]  - in[2];
		out[1] = base_throttle                 -      in[1]  + in[2];
		out[2] = base_throttle + 0.866f*in[0]  - 0.5f*in[1]  - in[2];
		out[3] = base_throttle - 0.866f*in[0]  + 0.5f*in[1]  + in[2];
		out[4] = base_throttle + 0.866f*in[0]  + 0.5f*in[1]  + in[2];
		out[5] = base_throttle - 0.866f*in[0]  - 0.5f*in[1]  - in[2];		
	}else if(_frame_type == frame_type_4){
		/* blue jay's frame */
		/*     -1	 	  0	 	  -1 */
		/*      1	 	  0		   1 */
		/*     1/2    sqrt(3)/2	  -1 */
		/*    -1/2	 -sqrt(3)/2	   1 */
		/*    -1/2	  sqrt(3)/2	   1 */
		/*     1/2	 -sqrt(3)/2	  -1 */
		out[0] = base_throttle+ 1.414f*(-      in[0]                )  - in[2];
		out[1] = base_throttle+ 1.414f*(+      in[0]                )  + in[2];
		out[2] = base_throttle+ 1.414f*(+ 0.5f*in[0]  + 0.866f*in[1])  - in[2];
		out[3] = base_throttle+ 1.414f*(- 0.5f*in[0]  - 0.866f*in[1])  + in[2];
		out[4] = base_throttle+ 1.414f*(- 0.5f*in[0]  + 0.866f*in[1])  + in[2];
		out[5] = base_throttle+ 1.414f*(+ 0.5f*in[0]  - 0.866f*in[1])  - in[2];		
	}else{
		Console.e(TAG, "err, unknow frame type:%d\n", _frame_type);
		rc_enter_status(RC_LOCK_STATUS);
	}
}

quaternion _calc_target_quaternion(int ctrl_mode, float dT)
{
	quaternion qt;	// target quaternion

	if(ctrl_mode == 1){	/* attitude control */
		float toRad = PI/180.0f;
		Euler e;
		
		e.roll = (rc_get_chanval(CHAN_ROLL)-0.5f)*2.0f*toRad*ANGLE_CONTROL_SCALE;
        e.pitch = -(rc_get_chanval(CHAN_PITCH)-0.5f)*2.0f*toRad*ANGLE_CONTROL_SCALE;
		if(!att_mode_lock_yaw){
			if(rc_get_chanval(CHAN_YAW)>0.55 || rc_get_chanval(CHAN_YAW)<0.45){
				yaw_target += (rc_get_chanval(CHAN_YAW)-0.5f)*2.0f*dT*toRad*YAW_CONTROL_SPEED;
				if(yaw_target >= PI){
					yaw_target -= 2*PI;
				}
				if(yaw_target < -PI){
					yaw_target += 2*PI;
				}
			}
		}
		e.yaw = yaw_target;
		
		quaternion_fromEuler(e, &qt);
	}
	
	return qt;
}

Euler _calc_target_euler(int ctrl_mode, float dT)
{
	/* init target euler angle */
	Euler et = {0.0f, 0.0f, yaw_target};

	if(ctrl_mode == 1){	/* attitude control */
		if(_request_control == false){
			float toRad = PI/180.0f;
			float chan_roll = rc_get_chanval(CHAN_ROLL);
			float chan_pitch = rc_get_chanval(CHAN_PITCH);
			float chan_yaw = rc_get_chanval(CHAN_YAW);
			
			if( !IN_RANGE(chan_roll, 0.5f-ATT_CTRL_DEADZONE, 0.5f+ATT_CTRL_DEADZONE) )
				et.roll = (chan_roll-0.5f)*2.0f*toRad*ANGLE_CONTROL_SCALE;
			if( !IN_RANGE(chan_pitch, 0.5f-ATT_CTRL_DEADZONE, 0.5f+ATT_CTRL_DEADZONE) )
				et.pitch = -(chan_pitch-0.5f)*2.0f*toRad*ANGLE_CONTROL_SCALE;
			if(!att_mode_lock_yaw){
				if( !IN_RANGE(chan_yaw, 0.5f-ATT_CTRL_DEADZONE, 0.5f+ATT_CTRL_DEADZONE) ){
					yaw_target += (chan_yaw-0.5f)*2.0f*dT*toRad*YAW_CONTROL_SPEED;
					if(yaw_target > PI){
						yaw_target -= 2*PI;
					}
					if(yaw_target <= -PI){
						yaw_target += 2*PI;
					}
				}
			}
			et.yaw = yaw_target;
		}else{
			et.roll = Deg2Rad(_ctrl_req_param.roll_sp);
			et.pitch = Deg2Rad(_ctrl_req_param.pitch_sp);
			et.yaw = yaw_target;
		}
	}
	
	return et;
}

void throttle_compensate(float* throttle)
{
//	const float cR = 718.078;
//	const float b = 88.448;
	float T_up = PARAM_GET_FLOAT(ADRC_ATT, T_UP);
	float T_down = PARAM_GET_FLOAT(ADRC_ATT, T_DOWN);
	

	// T' = T + t*\dot{T}, where T is throttle, \dot{T} is derivative of T, and t = T_up or T_down
	for(uint8_t i = 0 ; i < MOTOR_NUM ; i++){
		adrc_td(&throttle_td[i], throttle[i]);
		if(throttle_td[i].v2 > 0){
			throttle[i] += T_up*throttle_td[i].v2;
		}else{
			throttle[i] += T_down*throttle_td[i].v2;
		}
		constrain(&throttle[i], THROTTLE_MIN, THROTTLE_MAX);
	}
}

ADRC_Log adrc_log;
void _ctrl_att_with_baseThrottle(float baseThrottle, float dT)
{
	float out[3];
	float err[3];
	float gyr_t[3];
	sensor_get_gyr(gyr_t);
	
	/* only control attitude when throttle is bigger than threshold */
	if(baseThrottle > 0.05f){

		/* tilt protection */
#ifndef HIL_SIMULATION
		if(fabs(Rad2Deg(_ec.roll))>45.0f || fabs(Rad2Deg(_ec.pitch))>45.0f){
			Console.print("tilt protection\n");
			rc_enter_status(RC_LOCK_STATUS);
			return ;
		}
#endif
		
		/* calculate target attitude according to rc value */
		Euler et = _calc_target_euler(1, dT);
		
		/* calculate attitude error */
		err[0] = et.roll - _ec.roll;
		err[1] = et.pitch - _ec.pitch;
		err[2] = et.yaw - _ec.yaw;
		//TODO: would cause rotation for some case?
		if(err[2] > PI){
			err[2] -= 2.0*PI;
		}
		if(err[2] < -PI){
			err[2] += 2.0*PI;
		}
		
		if(PARAM_GET_INT32(ADRC_ATT, ADRC_ENABLE)){
			/* adrc control */
			if(PARAM_GET_INT32(ADRC_ATT, ADRC_MODE) == 1 || PARAM_GET_INT32(ADRC_ATT, ADRC_ENABLE) > 2 ){
				/* full adrc mode */
				adrc_log.v = Rad2Deg(et.pitch);
				adrc_att_control(err, gyr_t, out, baseThrottle);
			}else{
				/* pid + leso */
				err[0] = Rad2Deg(err[0]);
				err[1] = Rad2Deg(err[1]);
				err[2] = Rad2Deg(err[2]);
				// pid controller
				float u0[3];
				att_pid_update(err, u0, gyr_t, dT, baseThrottle);
				out[2] = u0[2];
				// disturbance conpensation
				adrc_att_dis_comp(u0, out);
			}
		}else{
			/* pid control */
			err[0] = Rad2Deg(err[0]);
			err[1] = Rad2Deg(err[1]);
			err[2] = Rad2Deg(err[2]);
			att_pid_update(err, out, gyr_t, dT, baseThrottle);
		}

		/* calculate motor output */
		_ctrl_mix_throttle_out(_throttle_out, out, baseThrottle);
		ctrl_constrain_throttle(_throttle_out, MOTOR_NUM);
		/* throttle feedforward to compensate motor delay */
		throttle_compensate(_throttle_out);
	}else{
		for(uint8_t i = 0 ; i < MOTOR_NUM ; i++){
			_throttle_out[i] = baseThrottle;
		}
	}

	ctrl_constrain_throttle(_throttle_out, MOTOR_NUM);
	ctrl_set_throttle(_throttle_out, MOTOR_NUM);
}

//HomePosition ctrl_get_home(void)
//{
//	return _home;
//}

uint8_t ctrl_set_home(void)
{
	struct vehicle_gps_position_s gps_t = gps_get_report();
	_home.lat = (double)gps_t.lat*1e-7;	// change to degree
	_home.lon = (double)gps_t.lon*1e-7;
	_home.home_set = 1;
	
	return 0;
}

/*****************************ADRC*********************************/
void ctrl_att_adrc_update(void)
{
	float gyr[3];
	sensor_get_gyr(gyr);
	
	adrc_att_observer_update(gyr, ctrl_get_basethrottle());
}

/******************************************************************/

//////////////////////////////////////////////////////////////////////

void calculate_target_attitude(float dT)
{
	quaternion qc = attitude_est_get_quaternion();
		
	/* current attitude */
	Euler ec;
	quaternion_toEuler(&qc, &ec);

	/* calculate target attitude according to rc value */
	_att_et = _calc_target_euler(1, dT);
}

void ctrl_set_throttle(float* throttle, uint8_t throttle_num)
{
	// if(throttle_num > MAX_THROTTLE_NUM){
	// 	Console.w(TAG, "throttle num:%d is larger than max throttle num:%d\n", throttle_num, MAX_THROTTLE_NUM);
	// 	return ;
	// }
	
	float throttle_switch = rc_get_chanval(CHAN_THROTTLE_SWITCH);
	if(throttle_switch < 0.1f){
		/* throttle is switch off */
		for(int i = 0 ; i < throttle_num ; i++){
			_throttle_out[i] = 0.0f;
		}
	}

#ifdef HIL_SIMULATION
	float control[16] = {0.0f};
	for(int i = 0 ; i < throttle_num ; i++)
		control[i] = _throttle_out[i];
	mavlink_send_hil_actuator_control(control, throttle_num);
#else
	rt_device_write(motor_device_t, MOTOR_CH_ALL, throttle, throttle_num);
#endif
	
	mcn_publish(MCN_ID(MOTOR_THROTTLE), _throttle_out);
}

void ctrl_unlock_vehicle(void)
{
	int on_off = 1;
	
	_att_outerloop_update = 1;
	
	quaternion cur_att = attitude_est_get_quaternion();
	Euler ec;
//	quaternion_toEuler(cur_att, &ec);
	mcn_copy_from_hub(MCN_ID(ATT_EULER), &ec);
	yaw_target = quaternion_getEuler(cur_att, 2);
	
	rt_device_control(motor_device_t, PWM_CMD_ENABLE, (void*)&on_off);
	_vehicle_status = 1;
	
	/* set home position */
	ctrl_set_home();
	
	uint32_t att_update_period = copter_get_event_period(AHRS_Period);
	uint32_t control_period = copter_get_event_period(Control_Period);
	att_pid_init(0.001f * att_update_period);
	_baseThrottle = 0.0f;
	_throttle_lpf = 0.0f;
	
	adrc_att_reset(0.001f*control_period);
	
	_ctrl_req_param.base_throttle = 0.0f;
	_ctrl_req_param.roll_sp = 0.0f;
	_ctrl_req_param.pitch_sp = 0.0f;
}

void ctrl_lock_vehicle(void)
{
	int on_off = 0;
	
	for(int i = 0 ; i < MOTOR_NUM ; i++){
		_throttle_out[i] = 0.0f;
	}
	ctrl_set_throttle(_throttle_out, MOTOR_NUM);
	rt_device_control(motor_device_t, PWM_CMD_ENABLE, (void*)&on_off);
	_vehicle_status = 0;
	alt_hold_mode = 0;
}

void control_init(void)
{
	motor_device_t = rt_device_find("motor");
	if(motor_device_t == RT_NULL){
		Console.e(TAG, "can't find motor device\n");
		return;
	}
	rt_device_open(motor_device_t , RT_DEVICE_OFLAG_RDWR);
	
	//ctrl_throttle_set_lpf(5, 0.004);
	
	int mcn_res;
	mcn_res = mcn_advertise(MCN_ID(MOTOR_THROTTLE));
	if(mcn_res != 0){
		Console.e(TAG, "err:%d, MOTOR_THROTTLE advertise fail!\n", mcn_res);
	}
	mcn_res = mcn_advertise(MCN_ID(ADRC));
	if(mcn_res != 0){
		Console.e(TAG, "err:%d, ADRC advertise fail!\n", mcn_res);
	}
	
	alt_node_t = mcn_subscribe(MCN_ID(ALT_INFO), NULL);
	if(alt_node_t == NULL)
		Console.e(TAG, "ALT_INFO subscribe err\n");
	
	/* lock the vehicle after init and set throttle to 0 */
	ctrl_lock_vehicle();
	
	_att_et.roll = _att_et.pitch = _att_et.yaw = 0.0f;
	
	uint32_t control_period = copter_get_event_period(Control_Period);
	
	for(int i = 0 ; i < MOTOR_NUM ; i++){
		adrc_td_init(&throttle_td[i], 0.001f * control_period, 500, 0.001f * control_period);
	}
	
	adrc_att_init(0.001f*control_period);
}

int control_vehicle(float dT)
{
	_switch_control_mode(rc_get_chanval(CHAN_CTRL_MODE));	
	/* lock status */
	if(!_vehicle_status)
		return 1;

	mcn_copy_from_hub(MCN_ID(ATT_EULER), &_ec);
	
	switch(_control_mode)
	{
		case 1:	/* attitude control mode */
		{
			control_attitude(dT);
		}break;
		case 2:	/* altitude hold mode */
		{
			control_altitude(dT);
		}break;
		case 3:	/* position control mode */
		{
			//TODO
		}break;
		default:
		{
			Console.e(TAG, "unknow control mode:%d\n", _control_mode);
			return 2;
		}break;
	}
	
	return 0;
}

void control_attitude(float dT)
{
	float baseThrottle;
	/* get throttle value */
	/* the throttle initialy in the central(0.5), so we start throttle from 0 when channel_throttle=0.5 */
	if(_request_control == false){
		baseThrottle = rc_get_chanval(CHAN_THROTTLE) - 0.5f;
		baseThrottle = baseThrottle>0.0f ? baseThrottle:0.0f;
		baseThrottle = 2.0f*baseThrottle;
	}else{
		baseThrottle = _ctrl_req_param.base_throttle;
	}
	
	ctrl_set_basethrottle(baseThrottle);
	_ctrl_att_with_baseThrottle(ctrl_get_basethrottle(), dT);
}

void control_altitude(float dT)
{
	float base_throttle;

	if(mcn_poll(alt_node_t)){
		Altitude_Info alt_info;
		mcn_copy(MCN_ID(ALT_INFO), alt_node_t, &alt_info);
		
		float raw_th = rc_get_chanval(CHAN_THROTTLE);
		float vel_sp;
		float accel_sp;
		/* if throttle within deadzone, hold current altitude */
		if( IN_RANGE(raw_th, 0.5f-THROTTLE_DEAD_ZONE, 0.5f+THROTTLE_DEAD_ZONE) || 
				(alt_info.relative_alt>MAX_ALTITUDE && raw_th>=0.5f+THROTTLE_DEAD_ZONE) ){
			if(alt_hold_mode == 0){
				/* the setpoint altitude only be set when it did not set before */
				alt_setpoint = alt_info.relative_alt*100.0f; //change to cm
				alt_hold_mode = 1;
			}
			vel_sp = alt_to_vel(alt_setpoint, alt_info.relative_alt*100.0f);
		}else{
			/* leave altitude hold mode */
			alt_hold_mode = 0;
			
			if(raw_th <= 0.5f-THROTTLE_DEAD_ZONE){
				vel_sp = (raw_th-(0.5f-THROTTLE_DEAD_ZONE))/(0.5f-THROTTLE_DEAD_ZONE)*MAX_ALT_RATE*100.0f;
			}else{
				vel_sp = (raw_th-(0.5f+THROTTLE_DEAD_ZONE))/(0.5f-THROTTLE_DEAD_ZONE)*MAX_ALT_RATE*100.0f;
			}
		}
		accel_sp = vel_to_accel(vel_sp, alt_info.vz*100.0f);
		base_throttle = accel_to_throttle(accel_sp, (alt_info.az-alt_info.az_bias)*100.0f);
		/* change to 0~1 */
		base_throttle = base_throttle*0.001f;
		ctrl_set_basethrottle(base_throttle);
	}
	
	/* control atttitude with base throttle value */
	base_throttle = ctrl_get_basethrottle();
	_ctrl_att_with_baseThrottle(base_throttle, dT);
}

uint8_t control_request(bool request)
{
	_request_control = request;
	return 0;
}

uint8_t control_set(char* name, float val)
{
	uint8_t res = 0;
	
	if(strcmp(name, "base_throttle") == 0){
		_ctrl_req_param.base_throttle = val;
	}else if(strcmp(name, "roll_sp") == 0){
		_ctrl_req_param.roll_sp = val;
	}else if(strcmp(name, "pitch_sp") == 0){
		_ctrl_req_param.pitch_sp = val;
	}else{
		res = 1;
	}
	
	return res;
}

int handle_control_shell_cmd(int argc, char** argv)
{
	if(argc > 1){
		if(strcmp(argv[1], "request") == 0){
			/* request control */
			if(!control_request(true))
				Console.print("request control successfully\n");
		}
		if(strcmp(argv[1], "release") == 0){
			/* release control */
			if(!control_request(false))
				Console.print("release control successfully\n");
		}
		if(strcmp(argv[1], "set") == 0){
			if(argc > 3){
				float val = atof(argv[3]);
				if(!control_set(argv[2], val)){
					Console.print("set %s=%f\n", argv[2], val);
				}
			}
		}
		if(strcmp(argv[1], "get") == 0){
			Console.print("control request:%s\n", _request_control==true?"true":"false");
			Console.print("base_throttle:%f\n", _ctrl_req_param.base_throttle);
			Console.print("roll_sp:%f\n", _ctrl_req_param.roll_sp);
			Console.print("pitch_sp:%f\n", _ctrl_req_param.pitch_sp);
		}
	}
	
	return 0;
}
