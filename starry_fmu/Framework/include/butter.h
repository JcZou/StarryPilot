/*
 * File      : butter.h
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-03-30     zoujiachi    first version.
 */

#ifndef __BUTTER_H__
#define __BUTTER_H__

typedef struct{
    float	_cutoff_freq; 
    float	_a1;
    float	_a2;
    float	_b0;
    float	_b1;
    float	_b2;
    float	_delay_element_1;        // buffered sample -1
    float	_delay_element_2;        // buffered sample -2
}Butter2;

typedef struct
{
	float A[4];
	float B[4];
	float X[4];
	float Y[4];
}Butter3;

/* butter filter */
void butter2_set_cutoff_frequency(Butter2 *butter, float sample_freq, float cutoff_freq);
float butter2_reset(Butter2 *butter, float sample);
float butter2_filter_process(Butter2 *butter, float sample);
Butter3* butter3_filter_create(float b[4], float a[4]);
float butter3_filter_process(float in, Butter3* butter);

#endif
