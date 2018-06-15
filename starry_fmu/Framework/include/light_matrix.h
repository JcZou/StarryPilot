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

typedef struct  {
	int row, col;
	float **element;
}Mat;

Mat* MatCreate(Mat* mat, int row, int col);
void MatDelete(Mat* mat);
Mat* MatSetVal(Mat* mat, float* val);
void MatDump(const Mat* mat);

Mat* MatZeros(Mat* mat);
Mat* MatEye(Mat* mat);

Mat* MatAdd(Mat* src1, Mat* src2, Mat* dst);
Mat* MatSub(Mat* src1, Mat* src2, Mat* dst);
Mat* MatMul(Mat* src1, Mat* src2, Mat* dst);
Mat* MatTrans(Mat* src, Mat* dst);
float MatDet(Mat* mat);
Mat* MatAdj(Mat* src, Mat* dst);
Mat* MatInv(Mat* src, Mat* dst);

void MatCopy(Mat* src, Mat* dst);

#endif
