#include <stdlib.h>
#include <rthw.h>
#include <rtdevice.h>
#include <rtthread.h>
#include <sensor_manager.h>
#include "kalman.h"
#include <math.h>
#include "global.h"

//#define EVENT_POS_UPDATE        (1<<0)
//#define EVENT_ACC_UPDATE				(1<<1)

//#define POS_UPDATE_INTERVAL     100
//#define ACC_UPDATE_INTERVAL			15
//#define BARO_UPDATE_INTERVAL    15

#define Q 0.019347
#define R 0.033301

static float drone_global_position[6];

//static struct rt_timer timer_pos;
//static struct rt_timer timer_baro;
//static struct rt_timer timer_acc;

//static struct rt_event event_position;

//static rt_device_t baro_device_t1, gps_device_t1;
//static struct vehicle_gps_position_s gps_position;
//static struct satellite_info_s satellite_info;

//static MS5611_REPORT_Def baro_report;

static char* TAG = "Kalman";

rt_err_t position_init(void)
{
 //Global position initialization
 drone_global_position[0]=0;
 drone_global_position[1]=0;
 drone_global_position[2]=0;
 drone_global_position[3]=0;
 drone_global_position[4]=0;
 drone_global_position[5]=0;

 return RT_EOK;
}

//Get global position
const float* position_getPosition(void)
{
    return drone_global_position;
}


//#define PI 3.1415926535 
float gaussrand(){
    static float U, V;
    static int phase = 0;
    float Z;

    if(phase == 0)
    {
         U = rand() / (RAND_MAX + 1.0);
         V = rand() / (RAND_MAX + 1.0);
         Z = sqrt(-2.0 * log(U))* sin(2.0 * PI * V);
    }
    else
    {
         Z = sqrt(-2.0 * log(U)) * cos(2.0 * PI * V);
    }

    phase = 1 - phase;
    return Z;
}


void kalman2_init(kalman2_state *state, float *init_x, float init_u, float* init_p[2])
{
//		float dt_pos = POS_UPDATE_INTERVAL * 1e10-3;
//		float dt_acc = ACC_UPDATE_INTERVAL * 1e10-3;
//	
//    state->x[0]    = init_x[0];
//    state->x[1]    = init_x[1];
//    state->p[0][0] = init_p[0][0];
//    state->p[0][1] = init_p[0][1];
//    state->p[1][0] = init_p[1][0];
//    state->p[1][1] = init_p[1][1];
//		//state->u			 ={acceleration}
//		state->u = init_u;
//		state->B[0]=0.5*dt_acc*dt_acc;
//		state->B[0]=dt_acc;
//    //state->A       = {{1, 0.01}, {0, 1}};
//    state->A[0][0] = 1;
//    state->A[0][1] = dt_pos;
//    state->A[1][0] = 0;
//    state->A[1][1] = 1;
//    //state->H       = {1,0};
//    state->H[0]    = 1;
//    state->H[1]    = 1;
//    //state->q       = {{10e-6,0}, {0,10e-6}};  /* measure noise convariance */
//    state->q[0]    = Q;
//    state->q[1]    = Q;
//    state->r       = R;  /* estimated error convariance */
}



extern kalman2_state state_x;
extern kalman2_state state_y;
extern kalman2_state state_z;
void kalman2_filter(kalman2_state *state, float u, float *z_measure)
{
	/* Step1: Predate */
    /* x(n|n-1) = A*x(n-1|n-1) + B*u */
	float pred_x[2] = {state->x[0], state->x[1]};
    state->x[0] = state->A[0][0]*pred_x[0] + state->A[0][1]*pred_x[1] + state->B[0]*u;
    state->x[1] = state->A[1][0]*pred_x[0] + state->A[1][1]*pred_x[1] + state->B[1]*u;
	
    /* p(n|n-1)=A^2*p(n-1|n-1)+q */
	float pred_p[2][2];
	pred_p[0][0] = state->A[0][0]*(state->A[0][0]*state->p[0][0] + state->A[0][1]*state->p[1][0])
						+ state->A[0][1]*(state->A[0][0]*state->p[0][1] + state->A[0][1]*state->p[1][1])
						+ state->q[0][0];
	pred_p[0][1] = state->A[1][0]*(state->A[0][0]*state->p[0][0] + state->A[0][1]*state->p[1][0])
						+ state->A[1][1]*(state->A[0][0]*state->p[0][1] + state->A[0][1]*state->p[1][1])
						+ state->q[0][1];
	pred_p[1][0] = state->A[0][0]*(state->A[1][0]*state->p[0][0] + state->A[1][1]*state->p[1][0])
						+ state->A[0][1]*(state->A[1][0]*state->p[0][1] + state->A[1][1]*state->p[1][1])
						+ state->q[1][0];
	pred_p[1][1] = state->A[1][0]*(state->A[1][0]*state->p[0][0] + state->A[1][1]*state->p[1][0])
						+ state->A[1][1]*(state->A[1][0]*state->p[0][1] + state->A[1][1]*state->p[1][1])
						+ state->q[1][1];

    /* Step2: Update */
    /* gain = p * H^T * [r + H * p * H^T]^(-1), H^T means transpose. */
	/* calculate inv(S), H = I */
	float S[2][2], invS[2][2], detS;
	S[0][0] = pred_p[0][0] + state->r[0][0];
	S[0][1] = pred_p[0][1] + state->r[0][1];
	S[1][0] = pred_p[1][0] + state->r[1][0];
	S[1][1] = pred_p[1][1] + state->r[1][1];
	
	detS = S[0][0]*S[1][1] - S[0][1]*S[1][0];
	invS[0][0] =  S[1][1] / detS;
	invS[0][1] = -S[0][1] / detS;
	invS[1][0] = -S[1][0] / detS;
	invS[1][1] =  S[0][0] / detS;
	
	state->gain[0][0] =  pred_p[0][0]*invS[0][0] + pred_p[0][1]*invS[1][0];
	state->gain[0][1] =  pred_p[0][0]*invS[0][1] + pred_p[0][1]*invS[1][1];
	state->gain[1][0] =  pred_p[1][0]*invS[0][0] + pred_p[1][1]*invS[1][0];
	state->gain[1][1] =  pred_p[1][0]*invS[0][1] + pred_p[1][1]*invS[1][1];
	
//	if(state == &state_y){
//		Console.w(TAG, "P:%f %f %f %f gain:%f %f %f %f\n", state->p[0][0],state->p[0][1],state->p[1][0],state->p[1][1], state->gain[0][0],state->gain[0][1],state->gain[1][0],state->gain[1][1]);
//		//Console.w(TAG, "P:%f %f %f %f R:%f %f %f %f\n", state->p[0][0],state->p[0][1],state->p[1][0],state->p[1][1],
//		//	state->r[0][0],state->r[0][1],state->r[1][0],state->r[1][1]);
//	}
	
    /* x(n|n) = x(n|n-1) + gain(n) * [z_measure - H(n)*x(n|n-1)]*/
	float  y[2];
	/* residual y = z - H * x(k|k-1) */
	y[0] = z_measure[0] - state->x[0];
	y[1] = z_measure[1] - state->x[1];
	state->x[0] += state->gain[0][0]*y[0] + state->gain[0][1]*y[1];
	state->x[1] += state->gain[1][0]*y[0] + state->gain[1][1]*y[1];

    /* Update @p: p(n|n) = [I - gain * H] * p(n|n-1) */
    state->p[0][0] = (1 - state->gain[0][0])*pred_p[0][0] - state->gain[0][1]*pred_p[1][0];
    state->p[0][1] = (1 - state->gain[0][0])*pred_p[0][1] - state->gain[0][1]*pred_p[1][1];
    state->p[1][0] = (1 - state->gain[1][1])*pred_p[1][0] - state->gain[1][0]*pred_p[0][0];
    state->p[1][1] = (1 - state->gain[1][1])*pred_p[1][1] - state->gain[1][0]*pred_p[0][1];
}
