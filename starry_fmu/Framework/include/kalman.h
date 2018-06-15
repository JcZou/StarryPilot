#ifndef  _KALMAN_FILTER_H
#define  _KALMAN_FILTER_H

/* 
 * NOTES: n Dimension means the state is n dimension, 
 * measurement always 1 dimension 
 */

/* 2 Dimension */
typedef struct {
    float x[2];     /* state: [0]-angle [1]-diffrence of angle, 2x1 */
	float u;			/* control vector: 1x2*/
	float B[2];			/* control input model*/
    float A[2][2];  /* X(n)=A*X(n-1)+U(n),U(n)~N(0,q), 2x2 */
    float H[2][2];     /* Z(n)=H*X(n)+W(n),W(n)~N(0,r), 1x2   */
    float q[2][2];     /* process(predict) noise convariance,2x1 [q0,0; 0,q1] */
    float r[2][2];        /* measure noise convariance */
    float p[2][2];  /* estimated error convariance,2x2 [p0 p1; p2 p3] */
    float gain[2][2];  /* 3x1 */
}kalman2_state;                   

void kalman2_init(kalman2_state *state, float *init_x, float init_u, float* init_p[2]);
void kalman2_filter(kalman2_state *state, float u, float *z_measure);
float gaussrand(void);
//void position_loop(void *parameter);

#endif  /*_KALMAN_FILTER_H*/
