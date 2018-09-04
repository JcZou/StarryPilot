/*
 * Light Matrix: C code implementation for basic matrix operation
 *
 * Copyright (C) 2017 Jiachi Zou
 *
 * This code is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This code is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with code.  If not, see <http:#www.gnu.org/licenses/>.
 */

#ifndef __LIGHT_MATRIX__
#define __LIGHT_MATRIX__

#define LIGHT_MATRIX_TYPE		float

typedef struct  {
	int row, col;
	LIGHT_MATRIX_TYPE **element;
}Mat;

Mat* MatCreate(Mat* mat, int row, int col);
void MatDelete(Mat* mat);
Mat* MatSetVal(Mat* mat, LIGHT_MATRIX_TYPE* val);
void MatDump(const Mat* mat);

Mat* MatZeros(Mat* mat);
Mat* MatEye(Mat* mat);

Mat* MatAdd(Mat* src1, Mat* src2, Mat* dst);
Mat* MatSub(Mat* src1, Mat* src2, Mat* dst);
Mat* MatMul(Mat* src1, Mat* src2, Mat* dst);
Mat* MatTrans(Mat* src, Mat* dst);
LIGHT_MATRIX_TYPE MatDet(Mat* mat);
Mat* MatAdj(Mat* src, Mat* dst);
Mat* MatInv(Mat* src, Mat* dst);

void MatEig(Mat *mat, LIGHT_MATRIX_TYPE *eig_val, Mat *eig_vec, LIGHT_MATRIX_TYPE eps, int njt);
	
void MatCopy(Mat* src, Mat* dst);

#endif
