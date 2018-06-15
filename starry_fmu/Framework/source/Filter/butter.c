/*
 * File      : butter.c
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-03-30     zoujiachi    first version.
 */

#include "butter.h"
#include "console.h"
#include "global.h"
#include <math.h>
//#include <rtthread.h>

void butter2_set_cutoff_frequency(Butter2 *butter, float sample_freq, float cutoff_freq)
{
    butter->_cutoff_freq = cutoff_freq;
    if (butter->_cutoff_freq <= 0.0f) {
        // no filtering
        return;
    }
    float fr = sample_freq/butter->_cutoff_freq;
    float ohm = tanf(PI/fr);
    float c = 1.0f+2.0f*cosf(PI/4.0f)*ohm + ohm*ohm;
    butter->_b0 = ohm*ohm/c;
    butter->_b1 = 2.0f*butter->_b0;
    butter->_b2 = butter->_b0;
    butter->_a1 = 2.0f*(ohm*ohm-1.0f)/c;
    butter->_a2 = (1.0f-2.0f*cosf(PI/4.0f)*ohm+ohm*ohm)/c;
	
	butter->_delay_element_1 = butter->_delay_element_2 = 0.0f;
}

float butter2_filter_process(Butter2 *butter, float sample)
{
    if (butter->_cutoff_freq <= 0.0f) {
        // no filtering
        return sample;
    }

    // do the filtering
    float delay_element_0 = sample - butter->_delay_element_1 * butter->_a1 - butter->_delay_element_2 * butter->_a2;
//    if (!isfinite(delay_element_0)) {
//        // don't allow bad values to propagate via the filter
//        delay_element_0 = sample;
//    }
    float output = delay_element_0 * butter->_b0 + butter->_delay_element_1 * butter->_b1 + 
					butter->_delay_element_2 * butter->_b2;
    
    butter->_delay_element_2 = butter->_delay_element_1;
    butter->_delay_element_1 = delay_element_0;

    // return the value.  Should be no need to check limits
    return output;
}

float butter2_reset(Butter2 *butter, float sample)
{
	float dval = sample / (butter->_b0 + butter->_b1 + butter->_b2);
    butter->_delay_element_1 = dval;
    butter->_delay_element_2 = dval;
	
    return butter2_filter_process(butter, sample);
}

Butter3* butter3_filter_create(float b[4], float a[4])
{
	Butter3* butter = rt_malloc(sizeof(Butter3));
	if(butter == NULL){
		Console.print("butter create fail, malloc fail\n");
		return NULL;
	}
	for(uint8_t i = 0 ; i < 4 ; i++){
		butter->B[i] = b[i];
		butter->A[i] = a[i];
		butter->X[i] = butter->Y[i] = 0.0f;
	}

	return butter;
}

float butter3_filter_process(float in, Butter3* butter)
{
	float out;
	butter->X[3] = in;
	/* a(1)*y(n) = b(1)*x(n) + b(2)*x(n-1) + ... + b(nb+1)*x(n-nb)
                         - a(2)*y(n-1) - ... - a(na+1)*y(n-na)  */
	butter->Y[3] = butter->B[0]*butter->X[3] + butter->B[1]*butter->X[2] + butter->B[2]*butter->X[1]
		+ butter->B[3]*butter->X[0] - butter->A[1]*butter->Y[2] - butter->A[2]*butter->Y[1] - butter->A[3]*butter->Y[0];

	/* we assume a(1)=1 */
	out = butter->Y[3];

	/* move X and Y */
	butter->X[0] = butter->X[1];
	butter->X[1] = butter->X[2];
	butter->X[2] = butter->X[3];
	butter->Y[0] = butter->Y[1];
	butter->Y[1] = butter->Y[2];
	butter->Y[2] = butter->Y[3];

	return out;
}
