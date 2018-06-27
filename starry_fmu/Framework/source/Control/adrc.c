/*
 * File      : adrc.c
 *
 *
 * Change Logs:
 * Date           Author       	Notes
 * 2018-03-18     zoujiachi   	the first version
 */
 
#include "adrc.h"
#include "copter_main.h"
#include "ap_math.h"
#include "param.h"
#include "global.h"
#include "console.h"
#include "filter.h"
#include <math.h>

static float adrc_sign(float val)
{
	if(val >= 0.0f)
		return 1.0f;
	else
		return -1.0f;
}

float adrc_fhan(float v1, float v2, float r0, float h0)
{
	float d = h0 * h0 * r0;
	float a0 = h0 * v2;
	float y = v1 + a0;
	float a1 = sqrtf(d*(d + 8.0f*fabsf(y)));
	float a2 = a0 + adrc_sign(y)*(a1-d)*0.5f;
	float sy = (adrc_sign(y+d) - adrc_sign(y-d))*0.5f;
	float a = (a0 + y - a2)*sy + a2;
	float sa = (adrc_sign(a+d) - adrc_sign(a-d))*0.5f;
	
	return -r0*(a/d - adrc_sign(a))*sa - r0*adrc_sign(a);
}

float adrc_fal(float e, float alpha, float delta)
{
	if(fabsf(e) <= delta){
		return e / (powf(delta, 1.0f-alpha));
	}else{
		return powf(fabsf(e), alpha) * adrc_sign(e);
	}
}

void adrc_td_init(ADRC_TD_Def* td_t, float h, float r0, float h0)
{
	td_t->h = h;
	td_t->r0 = r0;
	td_t->h0 = h0;
	td_t->v1 = td_t->v2 = 0.0f;
}

void adrc_td(ADRC_TD_Def* td, float v)
{
	float fv = adrc_fhan(td->v1 - v, td->v2, td->r0, td->h0);
	
	td->v1 += td->h * td->v2;
	td->v2 += td->h * fv;
}

void adrc_td_control_init(TD_Controller_Def* td_controller, float h, float r2, float h2)
{
	td_controller->v1 = 0.0f;
	td_controller->v2 = 0.0f;
	td_controller->r2 = r2;
	td_controller->h2 = h2;
	td_controller->h = h;
}

float adrc_td_control(TD_Controller_Def* td_controller, float err)
{
	float fv = adrc_fhan(-err, td_controller->v2, td_controller->r2, td_controller->h2);
	td_controller->v1 += td_controller->h * td_controller->v2;
	td_controller->v2 += td_controller->h * fv;
	
	return td_controller->v2;
}

void adrc_eso_init(ADRC_ESO_Def* eso_t, float h, float beta1, float beta2, float alpha, float delta, float b0)
{
	eso_t->h = h;
	eso_t->beta1 = beta1;
	eso_t->beta2 = beta2;
	eso_t->u = 0.0f;
	eso_t->alpha = alpha;
	eso_t->delta = delta;
	eso_t->b0 = b0;
	
	eso_t->z1 = eso_t->z2 = 0.0f;
}

void adrc_eso(ADRC_ESO_Def* eso_t, float y)
{
	float e = eso_t->z1 - y;
	float fe = adrc_fal(e, eso_t->alpha, eso_t->delta);
	
	eso_t->z1 += eso_t->h*(eso_t->z2 + eso_t->b0*eso_t->u - eso_t->beta1*e);
    eso_t->z2 -= eso_t->h*eso_t->beta2*fe;
}

void adrc_leso_init(ADRC_LESO_Def* leso_t, float h, float w, float b0)
{
	leso_t->h = h;
	// (s + w)^2 = s^2 + beta_1 * s + beta_2
	leso_t->beta1 = 2.0f*w;
	leso_t->beta2 = w*w;
	leso_t->u = 0.0f;
	leso_t->b0 = b0;
	
	leso_t->z1 = leso_t->z2 = 0.0f;
}

void adrc_leso(ADRC_LESO_Def* leso_t, float y)
{
	float e = leso_t->z1 - y;
	
	leso_t->z1 += leso_t->h*(leso_t->z2 + leso_t->b0*leso_t->u - leso_t->beta1*e);
    leso_t->z2 -= leso_t->h*leso_t->beta2*e;
}

void adrc_nlsef_init(ADRC_NLSEF_Def* nlsef_t, float h, float r1, float h1, float c)
{
	nlsef_t->h = h;
	nlsef_t->h1 = h1;
	nlsef_t->r1 = r1;
	nlsef_t->c = c;
}

float adrc_nlsef(ADRC_NLSEF_Def* nlsef_t, float e1, float e2)
{
	float u0 = -adrc_fhan(e1, nlsef_t->c*e2, nlsef_t->r1, nlsef_t->h1);

	return u0;
}