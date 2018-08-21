/**
* File      : calibration.c
*
* 最小二乘法椭球拟合校正算法
*
* Change Logs:
* Date      	Author       	Notes
* 2016-06-30	zoujiachi		the first version
*/

//#include <rthw.h>
#include <rtdevice.h>
#include <rtthread.h>
#include <string.h>
#include <math.h>
#include "console.h"
#include "sensor_manager.h"
#include "delay.h"
#include "shell.h"
#include "calibration.h"
#include "mavproxy.h"
#include "mavlink_param.h"

#define MATRIX_SIZE 7
#define u8 unsigned char

static bool gyr_calibrate_flag = false;
	
double Abs(double a)
{
	return a<0 ? -a : a;
}

u8 Equal(double a,double b)
{
	return Abs(a-b) < 1e-8;
}

rt_err_t calibrate_gyr(uint16_t p_num)
{
	float gyr_data_p[3];
	double sum_gyr[3] = {0.0f,0.0f,0.0f};
	float offset_gyr[3];
	
	Console.print("reading gyr data...\n");
	for(uint32_t i = 0 ; i<p_num ; i++)
	{
		sensor_gyr_measure(gyr_data_p);
		sum_gyr[0] += gyr_data_p[0];
		sum_gyr[1] += gyr_data_p[1];
		sum_gyr[2] += gyr_data_p[2];
		//Console.print("(%.2f,%.2f,%.2f) " , gyr_data_p[i*3],gyr_data_p[i*3+1],gyr_data_p[i*3+2]);
		time_waitMs(1);
	}
	//Console.print("\r\n");
	
	offset_gyr[0] = -sum_gyr[0]/p_num;
	offset_gyr[1] = -sum_gyr[1]/p_num;
	offset_gyr[2] = -sum_gyr[2]/p_num;
	
	Console.print("gyr offset:%f %f %f\r\n" , offset_gyr[0],offset_gyr[1],offset_gyr[2]);
	
	return RT_EOK;
}

void gyr_mavlink_calibration(void)
{
	float gyr_data_p[3];
	static float sum_gyr[3] = {0.0f,0.0f,0.0f};
	float offset_gyr[3];
	static uint16_t count = 0;
	int ret = 0;

	if (!gyr_calibrate_flag) {
		return;
	}

	sensor_gyr_measure(gyr_data_p);
	sum_gyr[0] += gyr_data_p[0];
	sum_gyr[1] += gyr_data_p[1];
	sum_gyr[2] += gyr_data_p[2];
	count++;

	if (!(count % 20) || (count == GYR_CALIBRATE_COUNT)) {
		mavlink_send_calibration_progress_msg(((float)count / GYR_CALIBRATE_COUNT) * 10);
	}

	if (count == GYR_CALIBRATE_COUNT) {
		offset_gyr[0] = -sum_gyr[0]/count;
		offset_gyr[1] = -sum_gyr[1]/count;
		offset_gyr[2] = -sum_gyr[2]/count;
		sum_gyr[0] = 0.0f;
		sum_gyr[1] = 0.0f;
		sum_gyr[2] = 0.0f;

		ret = mavlink_param_set_value_by_index(CAL_GYRO0_XOFF, offset_gyr[0]);
		ret |= mavlink_param_set_value_by_index(CAL_GYRO0_YOFF, offset_gyr[1]);
		ret |= mavlink_param_set_value_by_index(CAL_GYRO0_ZOFF, offset_gyr[2]);

		if (!ret) {
			mavlink_send_status(CAL_DONE);
		} else {
			mavlink_send_status(CAL_FAILED);
		}

		count = 0;
		gyr_calibrate_flag = false;
	}
}

void gyr_mavlink_calibration_start(void)
{
	gyr_calibrate_flag = true;
}
	
//#ifdef CALI_METHOD_1

double m_matrix[MATRIX_SIZE][MATRIX_SIZE+1];
int m = MATRIX_SIZE;	
int n = MATRIX_SIZE+1;
double m_result[MATRIX_SIZE];	

void DispMatrix(void);

/**************************** Elipsoid fitting algorithm ************************************/

void ResetMatrix(void)
{
	int row , column;
	
	for(row = 0 ; row<m ; row++){
		for(column = 0 ; column<n ; column++)
			m_matrix[row][column] = 0.0f;
	}
}
	
void CalcData_Input(double x , double y , double z)
{
	double V[MATRIX_SIZE];
	int row , column;
	
	V[0] = x*x;
    V[1] = y*y;
    V[2] = z*z;
    V[3] = x;
    V[4] = y;
    V[5] = z;
    V[6] = 1.0;
	
	//构建VxVt矩阵(Vt为V的转置)，并进行累加
	for(row = 0 ; row<MATRIX_SIZE ; row++){
		for(column = 0 ; column<MATRIX_SIZE ; column++){
			m_matrix[row][column] += V[row]*V[column];
		}
	}
}

void SwapRow(int row1 , int row2)
{
	int column;
	double tmp;
	
	for(column = 0 ; column<n ; column++){
		tmp = m_matrix[row1][column];
		m_matrix[row1][column] = m_matrix[row2][column];
		m_matrix[row2][column] = tmp;
	}
}

void MoveBiggestElement2Top(int s_row , int s_column)
{
	int row;
	
	for(row = s_row+1 ; row<m ; row++){
		if( Abs(m_matrix[s_row][s_column])<Abs(m_matrix[row][s_column])){
			SwapRow(s_row , row);
		}
	}
}

//高斯消元法，求行阶梯型矩阵
u8 Matrix_GaussElimination(void)
{
	int row,column,i,j;
	double tmp;
	
	for(row = 0,column=0 ; row<m-1 && column<n-1 ; row++,column++){
		//将当前列最大的一行移上来
		MoveBiggestElement2Top(row , column);
		
		//整列都为0
		if(Equal(m_matrix[row][column],0.0f)){
			//Console.print("qiyi matrix:%d %d\r\n" , row , column);
			//DispMatrix();
			//return 0;
			row--;
			continue;
		}
		
		//高斯消元
		for(i = row+1 ; i<m ; i++){	
			if(Equal(m_matrix[i][column],0.0f))
				continue;	//为0，无需处理
			
			tmp = m_matrix[i][column]/m_matrix[row][column];
			
			for(j = column ; j<n ; j++){
				m_matrix[i][j] -= m_matrix[row][j]*tmp;
			}
		}

//		DispMatrix();
//		Console.print("\r\n");
	}

	return 1;
}

//求行最简型矩阵
int Matrix_RowSimplify(void)
{
    int c = n;//返回值，表示(解的任意常量数+1)；
    //
    int row,column,s,t;
    //
    for(row=0,column=0;row<m && column<n;row++,column++)
    {
        if(Equal(m_matrix[row][column],0))//平移，找出本行第一个非零；
        {
            row--;
            continue;
        }
        //
        c--;
        //
		//这里不化成对角矩阵为1的矩阵，为了防止输入的数据较大的时候，求出的解为接近于0值的情况
        //tmp = 1 / m_matrix[row][column];
        //for(k=column;k<n;k++)//前面的"0"就不处理了；
            //m_matrix[row][k] *= tmp;
        //
        //化上三角矩阵为对角矩阵
		
		if(row == m-1)
			m_matrix[row][column] = 0.0f;	//强制为0，释放一个自由度，否则很难有解
		
        for(s=0;s<row;s++)//下面的0也不用处理；
        {
			float tmp;
			
            if(Equal(m_matrix[s][column],0))
                continue;//已经为0；
            //
            tmp = m_matrix[s][column]/m_matrix[row][column];
            for(t=column;t<n;t++)
                m_matrix[s][t] -= m_matrix[row][t]*tmp;
            //
        }
//		DispMatrix();
//		Console.print("\r\n");
    }
    //
    return c;
}

void Matrix_Solve(double* C , double* sol)
{
	int row,column,i;
	int any_sol[MATRIX_SIZE];

	//找出任意解的位置
	memset(any_sol , 0 , MATRIX_SIZE);
	for(row=0,column=0 ; row<m && column<n-1 ; row++,column++){
		if(Equal(m_matrix[row][column] , 0.0f)){
			any_sol[column] = 1;	//记录任意解的位置
			row--;	//右移1列
		}
	}

	//求解
	row = 0;
	for(column = 0 ; column<n-1 ; column++){
		if(any_sol[column] == 1){	//任意解
			sol[column] = C[column];
		}else{
			sol[column] = m_matrix[row][n-1];
			//加上任意解
			for(i = column+1 ; i<n-1 ; i++){
				if(any_sol[i]==1 && !Equal(m_matrix[row][i],0.0f)){
					sol[column] -= m_matrix[row][i]*C[i];
				}
			}	
			sol[column] /= m_matrix[row][column];        //除以对角线元素
			row++;
		}
	}
}

void DispMatrix(void)
{
	int row,column;
	
	for(row = 0 ; row<m ; row++){
		for(column = 0 ; column<n ; column++){
			Console.print("%.2f		" , m_matrix[row][column]);
		}
		Console.print("\r\n");
	}
}

double* calibrate_process(double radius)
{
	double C[MATRIX_SIZE];
	double Res[MATRIX_SIZE];
	int i;
	double k;
	
	Matrix_GaussElimination();
	Matrix_RowSimplify();

    //赋值任意解参数值
	for(i = 0 ; i<MATRIX_SIZE ; i++){
		C[i] = 1000.0f;
	}

	Matrix_Solve(C , Res);

	Console.print("a:%.2f b:%.2f c:%.2f d:%.2f e:%.2f f:%.2f g:%.2f\r\n" , Res[0],Res[1],Res[2],Res[3],Res[4],Res[5],Res[6]);
	
	k = (Res[3]*Res[3]/Res[0]+Res[4]*Res[4]/Res[1]+Res[5]*Res[5]/Res[2] - 4*Res[6])/(4*radius*radius);

	if(Res[0]*k<0 || Res[1]*k<0 || Res[2]*k<0)
	{
		Console.print("data is not typical\r\n");
		return RT_NULL;
	}
	
	m_result[0] = sqrt(Res[0] / k);
    m_result[1] = sqrt(Res[1] / k);
    m_result[2] = sqrt(Res[2] / k);
    m_result[3] = Res[3] / (2 * Res[0]);
    m_result[4] = Res[4] / (2 * Res[1]);
    m_result[5] = Res[5] / (2 * Res[2]);

	Console.print("Xo:%f Yo:%f Zo:%f Xg:%f Yg:%f Zg:%f k:%f\r\n" , m_result[3],m_result[4],m_result[5],m_result[0],m_result[1],m_result[2],k);

	return m_result;
}

void cali_input_acc_data(uint16_t p_num)
{
	uint16_t i;
	float acc[3];
	
	for(i = 0 ; i<p_num ; i++)
	{
		sensor_acc_measure(acc);
		Console.print("(%.2f,%.2f,%.2f) " , acc[0],acc[1],acc[2]);
		CalcData_Input(acc[0], acc[1], acc[2]);
		time_waitMs(20);
	}
	Console.print("\r\n");
}

void cali_input_mag_data(uint16_t p_num)
{
	uint16_t i;
	float mag[3];
	
	for(i = 0 ; i<p_num ; i++)
	{
		sensor_mag_measure(mag);
		Console.print("(%.2f,%.2f,%.2f) " , mag[0],mag[1],mag[2]);
		CalcData_Input(mag[0], mag[1], mag[2]);
		time_waitMs(20);
	}
	Console.print("\r\n");
}

void Calc_Process(double radius)
{
	double C[MATRIX_SIZE];
	double Res[MATRIX_SIZE];
	int i;
	double k;

	ResetMatrix();

	//输入任意个数磁场测量点坐标，请尽量保证在椭球上分布均匀
//	CalcData_Input(-9.81,-0.20,-0.91);
//	CalcData_Input(-9.71,-0.06,-1.04);
//	CalcData_Input(-9.88,-0.06,-1.14);
//	CalcData_Input(-9.74,-0.17,-0.93);
//	CalcData_Input(-9.76,-0.27,-1.10);
//	CalcData_Input(-9.91,-0.13,-0.95);

//	CalcData_Input(0.19,-9.77,-1.54);
//	CalcData_Input(0.17,-9.72,-1.68);
//	CalcData_Input(0.06,-9.79,-1.73);
//	CalcData_Input(0.18,-9.81,-1.75);
//	CalcData_Input(0.08,-9.75,-1.73);
//	CalcData_Input(0.19,-9.76,-1.68);

//	CalcData_Input(9.94,-0.04,-0.74);
//	CalcData_Input(9.92,-0.04,-0.83);
//	CalcData_Input(10.30,-0.04,-0.96);
//	CalcData_Input(10.07,-0.05,-0.83);
//	CalcData_Input(10.02,-0.10,-0.54);
//	CalcData_Input(9.94,-0.20,-0.52);

//	CalcData_Input(0.22,9.54,-0.66);
//	CalcData_Input(0.01,9.45,-1.05);
//	CalcData_Input(0.18,9.57,-0.73);
//	CalcData_Input(0.11,9.52,-0.98);
//	CalcData_Input(0.24,9.65,-1.18);
//	CalcData_Input(0.10,9.58,-0.95);

//	CalcData_Input(-0.03,-0.17,8.10);
//	CalcData_Input(0.03,-0.08,8.29);
//	CalcData_Input(0.06,-0.11,8.22);
//	CalcData_Input(0.05,-0.18,8.47);
//	CalcData_Input(-0.03,-0.04,8.23);
//	CalcData_Input(-0.08,-0.21,8.51);

//	CalcData_Input(0.15,-0.07,-10.54);
//	CalcData_Input(0.07,0.19,-10.87); 
//	CalcData_Input(0.33,0.21,-10.93); 
//	CalcData_Input(0.51,0.25,-10.85); 
//	CalcData_Input(-0.17,-0.03,-10.71); 
//	CalcData_Input(-0.19,0.15,-10.77); 

	Matrix_GaussElimination();
	Matrix_RowSimplify();

    //赋值任意解参数值
	for(i = 0 ; i<MATRIX_SIZE ; i++){
		C[i] = 1000.0f;
	}

	Matrix_Solve(C , Res);

	Console.print("a:%.2f b:%.2f c:%.2f d:%.2f e:%.2f f:%.2f g:%.2f\r\n" , Res[0],Res[1],Res[2],Res[3],Res[4],Res[5],Res[6]);

	k = (Res[3]*Res[3]/Res[0]+Res[4]*Res[4]/Res[1]+Res[5]*Res[5]/Res[2] - 4*Res[6])/(4*radius*radius);

	m_result[0] = sqrt(Res[0] / k);
    m_result[1] = sqrt(Res[1] / k);
    m_result[2] = sqrt(Res[2] / k);
    m_result[3] = Res[3] / (2 * Res[0]);
    m_result[4] = Res[4] / (2 * Res[1]);
    m_result[5] = Res[5] / (2 * Res[2]);

	Console.print("Xo:%f Yo:%f Zo:%f Xg:%f Yg:%f Zg:%f k:%f\r\n" , m_result[3],m_result[4],m_result[5],m_result[0],m_result[1],m_result[2],k);
}
/**************************** Elipsoid fitting algorithm End ************************************/

//#elif defined CALI_METHOD_2

/**************************** Calibrate method 2 ************************************/
#define MAX_ROW		8
#define MAX_COL		9

#define squre(a,b,c)	(a*a+b*b+c*c)

double m_matrix2[MAX_ROW][MAX_COL];

int data_cnt;
float prev[3];
double saveP[9][3];

// 椭球 (x-Xo)^2/a^2 + (y-Yo)^2/b^2 + (z-Zo)^2/c^2 = 1
float Xo = 0.07;
float Yo = -0.03;
float Zo = 0.1;
float a = 1.02;
float b = 0.93;
float c = 1.03;
//非正交量
float alpha = 0.2;
float beta = 0.13;
float gamma_param = 0.08;

int g_is_first = 1; 

double a11, a12, a13, bx, a22, a23, by, bz;

#ifndef RAND_MAX
#define RAND_MAX (0xFFFFFFFFU)
#endif

/* 
** return a random real in the interval 
** [a, b] (also [a, b)) 
*/  
double rand_real(double a, double b) { 
//    if (g_is_first) {  
//        g_is_first = 0;  
//        srand((unsigned int)time(NULL));  
//    }  
  
    return (double)rand() / ((double)RAND_MAX / (b - a)) + a;  
}  

//随机产生椭球上点坐标
void randomEllipsoidPoint(double* P)
{
	P[0] = rand_real( -a+Xo, a+Xo );
	P[1] = rand_real( -b*sqrt(1-((P[0]-Xo)*(P[0]-Xo))/(a*a))+Yo, b*sqrt(1-((P[0]-Xo)*(P[0]-Xo))/(a*a))+Yo );
	P[2] = sqrt((1 - (P[0]-Xo)*(P[0]-Xo)/(a*a) - (P[1]-Yo)*(P[1]-Yo)/(b*b))*c*c);
	if(rand() % 2)
		P[2] = -P[2]+Zo;
	else
		P[2] += Zo;
}

//添加非正交量
void addNonOrthogonality(double* P)
{
	P[0] = P[0]*cos(gamma_param)*cos(alpha)+P[1]*sin(gamma_param)+P[2]*sin(alpha);
	P[1] = P[1]*cos(beta)+P[2]*sin(beta);
	P[2] = P[2];
}

//double Abs(double a)
//{
//	return a<0 ? -a : a;
//}

//u8 Equal(double a,double b)
//{
//	return Abs(a-b) < 1e-8;
//}

void SwapRow2(int row1 , int row2)
{
	int column;
	double tmp;
	
	for(column = 0 ; column<MAX_COL ; column++){
		tmp = m_matrix2[row1][column];
		m_matrix2[row1][column] = m_matrix2[row2][column];
		m_matrix2[row2][column] = tmp;
	}
}   

void MoveBiggestElement2Top2(int s_row , int s_column)
{
	int row;
	
	for(row = s_row+1 ; row<MAX_ROW ; row++){
		if( Abs(m_matrix2[s_row][s_column])<Abs(m_matrix2[row][s_column])){
			SwapRow2(s_row , row);
		}
	}
}

int inputMagRawData(float mx, float my, float mz)
{
	if(data_cnt > MAX_ROW)
		return -1;

	//Assume:
	// lamda1<<1, lamda2<<1, aij<<1
	// bx^2+by^2+bz^2<<B
	
	if(data_cnt > 0){
		//S_0t
		m_matrix2[data_cnt-1][MAX_COL-1] = (squre(prev[0], prev[1], prev[2]) - squre(mx, my, mz)) / 2;
		//S_1t
		m_matrix2[data_cnt-1][0] = mx*mx - prev[0]*prev[0];
		//S_2t
		m_matrix2[data_cnt-1][1] = my*my - prev[1]*prev[1];
		//S_3t
		m_matrix2[data_cnt-1][2] = mx*my - prev[0]*prev[1];
		//S_4t
		m_matrix2[data_cnt-1][3] = my*mz - prev[1]*prev[2];
		//S_5t
		m_matrix2[data_cnt-1][4] = mz*mx - prev[2]*prev[0];
		//S_6t
		m_matrix2[data_cnt-1][5] = mx - prev[0];
		//S_7t
		m_matrix2[data_cnt-1][6] = my - prev[1];
		//S_8t
		m_matrix2[data_cnt-1][7] = mz - prev[2];
	}

	prev[0] = mx;
	prev[1] = my;
	prev[2] = mz;
	
	saveP[data_cnt][0] = mx;
	saveP[data_cnt][1] = my;
	saveP[data_cnt][2] = mz;

	data_cnt++;

	return data_cnt;
}

void DispMatrix2(void)
{
	int row,column;
	
	for(row = 0 ; row<MAX_ROW ; row++){
		for(column = 0 ; column<MAX_COL ; column++){
			Console.print("%.2f	" , m_matrix2[row][column]);
		}
		Console.print("\r\n");
	}
	Console.print("\r\n");
}

//高斯消元法，求行阶梯型矩阵
u8 Matrix_GaussElimination2(void)
{
	int row,column,i,j;
	double tmp;
	
	for(row = 0,column=0 ; row<MAX_ROW-1 && column<MAX_COL-1 ; row++,column++){
		//将当前列最大的一行移上来
		MoveBiggestElement2Top2(row , column);
		
		//整列都为0
		if(Equal(m_matrix2[row][column],0.0f)){
			//Console.print("qiyi matrix:%d %d\r\n" , row , column);
			//DispMatrix();
			//return 0;
			row--;
			continue;
		}
		
		//高斯消元
		for(i = row+1 ; i<MAX_ROW ; i++){	
			if(Equal(m_matrix2[i][column],0.0f))
				continue;	//为0，无需处理
			
			tmp = m_matrix2[i][column]/m_matrix2[row][column];
			
			for(j = column ; j<MAX_COL ; j++){
				m_matrix2[i][j] -= m_matrix2[row][j]*tmp;
			}
		}

	}

	DispMatrix2();
	Console.print("\r\n");

	return 1;
}

//求行最简型矩阵
void Matrix_RowSimplify2(void)
{
    int row,column,s,t;
    //
    for(row=0,column=0 ; row<MAX_ROW && column<MAX_COL ; row++,column++)
    {
        if(Equal(m_matrix2[row][column],0))//平移，找出本行第一个非零；
        {
            row--;
            continue;
        }

        //化上三角矩阵为对角矩阵
        for(s=0 ; s<row ; s++)//下面的0也不用处理；
        {
			float tmp;
			
            if(Equal(m_matrix2[s][column],0))
                continue;//已经为0；
            //
            tmp = m_matrix2[s][column]/m_matrix2[row][column];
            for(t=column ; t<MAX_COL ; t++)
                m_matrix2[s][t] -= m_matrix2[row][t]*tmp;
            //
        }
     }

	DispMatrix2();
}

//解齐次线性方程
int solveMatrix2(double* solution)
{
	u8 i;

	Matrix_GaussElimination2();
	Matrix_RowSimplify2();

	for(i = 0 ; i < 8 ; i++){
		if(Equal(m_matrix2[i][i], 0.0f))
			return 1;
		
		solution[i] = m_matrix2[i][MAX_COL-1] / m_matrix2[i][i];
		Console.print("solution[%d]:%f\n", i, solution[i]);
	}

	return 0;
}

u8 validate(void)
{
	u8 i;
	double cali[3];

	Console.print("\nvalidate value:\n");
	for(i = 0 ; i < 9 ; i++){
		cali[0] = a11*saveP[i][0] + a12*saveP[i][1] + a13*saveP[i][2] + bx;
		cali[1] = a22*saveP[i][1] + a23*saveP[i][2] + by;
		cali[2] = saveP[i][2] + bz;

		Console.print("before:%f after:%f\n", saveP[i][0]*saveP[i][0]+saveP[i][1]*saveP[i][1]+saveP[i][2]*saveP[i][2],
				cali[0]*cali[0]+cali[1]*cali[1]+cali[2]*cali[2]);
	}
	
	return 0;
}

void cali_input_mag_data2(uint16_t p_num)
{
	uint16_t i;
	float mag[3];
	
	for(i = 0 ; i<p_num ; i++)
	{
		sensor_mag_measure(mag);
		Console.print("(%.2f,%.2f,%.2f) " , mag[0],mag[1],mag[2]);
		inputMagRawData(mag[0], mag[1], mag[2]);
		time_waitMs(20);
	}
	Console.print("\r\n");
}

void reset_cali(void)
{
	data_cnt = 0;
}

void calibrate_process2(void)
{
	double solution[8];

	if(solveMatrix2(solution))
		Console.print("NA\n");
	else{
		a11 = 1 + solution[0];
		a22 = 1 + solution[1];
		a12 = solution[2];
		a23 = solution[3];
		a13 = solution[4];
		bx = solution[5];
		by = solution[6];
		bz = solution[7];

		Console.print("a11:%f a12:%f a13:%f bx:%f\n", a11, a12, a13, bx);
		Console.print("a22:%f a23:%f by:%f\n", a22, a23, by);
		Console.print("bz:%f\n", bz);

		validate();
	}	
}

/**************************** Calibrate method 2 End ************************************/

int calibrate_acc_run(struct finsh_shell *shell)
{
//	Console.print("Calibrate acc:\r\n");
//	char ch;

//	/* reset matrix */
//	ResetMatrix();

//	Console.print("forward [Y/N]\r\n");
//	if (rt_sem_take(&shell->rx_sem, RT_WAITING_FOREVER) != RT_EOK) return 1;
//	rt_device_read(shell->device, 0, &ch, 1);
//	if(ch != 'Y' && ch != 'y')
//		return 1;
//	cali_input_acc_data(6);

//	Console.print("behind [Y/N]\r\n");
//	if (rt_sem_take(&shell->rx_sem, RT_WAITING_FOREVER) != RT_EOK) return 1;
//	rt_device_read(shell->device, 0, &ch, 1);
//	if(ch != 'Y' && ch != 'y')
//		return 1;
//	cali_input_acc_data(6);

//	Console.print("left [Y/N]\r\n");
//	if (rt_sem_take(&shell->rx_sem, RT_WAITING_FOREVER) != RT_EOK) return 1;
//	rt_device_read(shell->device, 0, &ch, 1);
//	if(ch != 'Y' && ch != 'y')
//		return 1;
//	cali_input_acc_data(6);

//	Console.print("right [Y/N]\r\n");
//	if (rt_sem_take(&shell->rx_sem, RT_WAITING_FOREVER) != RT_EOK) return 1;
//	rt_device_read(shell->device, 0, &ch, 1);
//	if(ch != 'Y' && ch != 'y')
//		return 1;
//	cali_input_acc_data(6);

//	Console.print("up [Y/N]\r\n");
//	if (rt_sem_take(&shell->rx_sem, RT_WAITING_FOREVER) != RT_EOK) return 1;
//	rt_device_read(shell->device, 0, &ch, 1);
//	if(ch != 'Y' && ch != 'y')
//		return 1;
//	cali_input_acc_data(6);

//	Console.print("down [Y/N]\r\n");
//	if (rt_sem_take(&shell->rx_sem, RT_WAITING_FOREVER) != RT_EOK) return 1;
//	rt_device_read(shell->device, 0, &ch, 1);
//	if(ch != 'Y' && ch != 'y')
//		return 1;
//	cali_input_acc_data(6);

//	/* calculate result */
//	if(calibrate_process(ACC_STANDARD_VALUE) == RT_NULL)
//		return 1;
//	else
//		return 0;

	return 0;
}

int calibrate_mag_run(struct finsh_shell *shell)
{
//	Console.print("Calibrate mag:\r\n");
//	char ch;

//#ifdef CALI_METHOD_1	
//	/* reset matrix */
//	ResetMatrix();
//	
//	Console.print("forward [Y/N]\r\n");
//	if (rt_sem_take(&shell->rx_sem, RT_WAITING_FOREVER) != RT_EOK) return 1;
//	rt_device_read(shell->device, 0, &ch, 1);
//	if(ch != 'Y' && ch != 'y')
//		return 1;
//	cali_input_mag_data(6);
//	
//	Console.print("behind [Y/N]\r\n");
//	if (rt_sem_take(&shell->rx_sem, RT_WAITING_FOREVER) != RT_EOK) return 1;
//	rt_device_read(shell->device, 0, &ch, 1);
//	if(ch != 'Y' && ch != 'y')
//		return 1;
//	cali_input_mag_data(6);
//	
//	Console.print("left [Y/N]\r\n");
//	if (rt_sem_take(&shell->rx_sem, RT_WAITING_FOREVER) != RT_EOK) return 1;
//	rt_device_read(shell->device, 0, &ch, 1);
//	if(ch != 'Y' && ch != 'y')
//		return 1;
//	cali_input_mag_data(6);
//	
//	Console.print("right [Y/N]\r\n");
//	if (rt_sem_take(&shell->rx_sem, RT_WAITING_FOREVER) != RT_EOK) return 1;
//	rt_device_read(shell->device, 0, &ch, 1);
//	if(ch != 'Y' && ch != 'y')
//		return 1;
//	cali_input_mag_data(6);
//	
//	Console.print("up [Y/N]\r\n");
//	if (rt_sem_take(&shell->rx_sem, RT_WAITING_FOREVER) != RT_EOK) return 1;
//	rt_device_read(shell->device, 0, &ch, 1);
//	if(ch != 'Y' && ch != 'y')
//		return 1;
//	cali_input_mag_data(6);
//	
//	Console.print("down [Y/N]\r\n");
//	if (rt_sem_take(&shell->rx_sem, RT_WAITING_FOREVER) != RT_EOK) return 1;
//	rt_device_read(shell->device, 0, &ch, 1);
//	if(ch != 'Y' && ch != 'y')
//		return 1;
//	cali_input_mag_data(6);
//	
//	/* calculate result */
//	if(calibrate_process(MAG_STANDARD_VALUE) == RT_NULL )
//		return 1;
//	else
//		return 0;
//#elif defined CALI_METHOD_2
//	reset_cali();
//	
//	for(int i = 0 ; i < 9 ; i++){
//		Console.print("%d point [Y/N]\r\n", i+1);
//		if (rt_sem_take(&shell->rx_sem, RT_WAITING_FOREVER) != RT_EOK) return 1;
//		rt_device_read(shell->device, 0, &ch, 1);
//		if(ch != 'Y' && ch != 'y')
//			return 1;
//		cali_input_mag_data2(1);
//	}
//	
//	calibrate_process2();	
//#endif

	return 0;
}

void calibrate_gyr_run(struct finsh_shell *shell)
{
	Console.print("Calibrate gyr:\r\n");
	calibrate_gyr(10000);
}

//#endif
