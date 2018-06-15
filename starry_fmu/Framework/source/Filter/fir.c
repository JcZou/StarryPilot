/*
 * File      : fir.c
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-06-13     zoujiachi    first version.
 */

#include "fir.h"
#include "console.h"
#include "global.h"
#include <math.h>

void fir_init(FIR* fir, int order, float* coeff, float* buffer_addr)
{
	fir->fir_length = order+1;
	fir->fir_coeff = coeff;
	fir->fir_buffer = buffer_addr;
	fir->fir_index = 0;
	
	for(int i = 0 ; i < fir->fir_length ; i++){
		fir->fir_buffer[i] = 0.0f;
	}
}

float fir_filter_process(FIR* fir, float sample)
{
	float output = 0.0f;
	
	fir->fir_buffer[fir->fir_index++] = sample;
	if(fir->fir_index >= fir->fir_length)
		fir->fir_index = 0;
	
	for(int i = 0 ; i < fir->fir_length ; i++){
		output += fir->fir_buffer[fir->fir_index] * fir->fir_coeff[i];
		if(fir->fir_index != 0){
			fir->fir_index --;
		} else {
			fir->fir_index = fir->fir_length-1;
		}
	}
	
	return output;
}
