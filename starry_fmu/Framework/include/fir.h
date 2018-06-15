/*
 * File      : fir.h
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-06-13     zoujiachi    first version.
 */

#ifndef __FIR_H__
#define __FIR_H__

typedef struct
{
	int fir_length;	/* fir_length = fir_order+1 */
	int fir_index;
	float *fir_coeff;
	float *fir_buffer;
}FIR;

void fir_init(FIR* fir, int order, float* coeff, float* buffer_addr);
float fir_filter_process(FIR* fir, float sample);

#endif
