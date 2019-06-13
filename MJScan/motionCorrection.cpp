#include <iostream>
#include <fstream>
#include <vector>

#include "motionCorrection.h"

#include "math.h"

//****************************************************************//
// 基本三角函数，输入值和返回值单位均为度
//****************************************************************//
#pragma region Tri
double asind(double a)
{
	double PI = 3.1415926;
	return asin(a) / PI * 180;
}

double acosd(double a)
{
	double PI = 3.1415926;
	return acos(a) / PI * 180;
}

double sind(double a)
{
	double PI = 3.1415926;
	return sin(a / 180 * PI);
}

double cosd(double a)
{
	double PI = 3.1415926;
	return cos(a / 180 * PI);
}
#pragma endregion

//****************************************************************//
// 基本的矩阵及向量操作
//****************************************************************//
#pragma region Matrix
vec2D matrixAdjoint(vec2D a, int x, int y)
{
	for (auto i = 0; i < a.size(); i++)
		a[i].erase(a[i].begin() + y);

	a.erase(a.begin() + x);

	return a;
}

double matrixDet(vec2D a)
{
	double deter = 0;
	double deterSign = -1;
	if (a.size() == 2)
		return a[0][0] * a[1][1] - a[1][0] * a[0][1];
	else
	{
		for (auto i = 0; i < a.size(); i++) {
			deterSign *= -1;
			deter += deterSign * a[0][i] * matrixDet(matrixAdjoint(a, 0, i));
		}
		return deter;
	}
}

vec2D matrixInverse(vec2D a)
{
	vec2D b(a.size(), vec1D(a.size(), 0));
	double deterA = matrixDet(a);

	for (auto i = 0; i < a.size(); i++)
		for (auto j = 0; j < a.size(); j++) {
			b[i][j] = pow(-1, i + j) * matrixDet(matrixAdjoint(a, i, j)) / deterA;
		}
	return matrixTranspose(b);
}

vec2D matrixTranspose(vec2D a)
{
	vec2D b(a[0].size(), vec1D(a.size(), 0));

	for (auto i = 0; i < a[0].size(); i++)
		for (auto j = 0; j < a.size(); j++)
			b[i][j] = a[j][i];

	return b;
}

vec2D vecTranspose(vec1D a)
{
	vec2D b(a.size(), vec1D(1, 0));

	for (auto i = 0; i < a.size(); i++)
			b[i][0] = a[i];

	return b;
}

vec1D vecTranspose(vec2D a)
{
	vec1D b(a.size());

	for (auto i = 0; i < a.size(); i++)
		b[i] = a[i][0];

	return b;
}

vec2D matrixMultiply(vec2D a, vec2D b)
{
	vec2D c(a.size(), vec1D(b[0].size(), 0));

	for (auto i = 0; i < a.size(); i++)
		for (auto j = 0; j < b[0].size(); j++)
			for (auto k = 0; k < a[0].size(); k++)
				c[i][j] += a[i][k] * b[k][j];

	return c;
}

double vecNorm(vec1D a)
{
	double result = 0;
	for (auto i = 0; i < a.size(); i++) {
		result += a[i] * a[i];
	}
	return sqrt(result);
}

vec1D cross(vec1D a, vec1D b)
{
	vec1D c(a.size());

	c[0] = a[1] * b[2] - a[2] * b[1];
	c[1] = a[2] * b[0] - a[0] * b[2];
	c[2] = a[0] * b[1] - a[1] * b[0];

	return c;
}

double dot(vec1D a, vec1D b)
{
	return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

#pragma endregion


//****************************************************************//
// 机器人姿态xyzabc和对应的变换矩阵相互转换
//****************************************************************//
#pragma region conversion between coodinates and transformation matrices
double solveAorC(double sinC, double cosC)
{
	double C=0;

	if (cosC > 0)
		C = asind(sinC);
	else if (cosC < 0)
		if (sinC > 0)
			C = (180 - asind(sinC));
		else if (sinC < 0)
			C = (-180 - asind(sinC));
		else
			C = 0;

	return C;
}

vec2D coor2matrix(vec1D coor)
{
	vec2D mat(4, vec1D(4, 0));

	mat[0][0] = cosd(coor[3])*cosd(coor[4]);
	mat[0][1] = cosd(coor[3])*sind(coor[4])*sind(coor[5]) - sind(coor[3])*cosd(coor[5]);
	mat[0][2] = sind(coor[3])*sind(coor[5]) + cosd(coor[3])*sind(coor[4])*cosd(coor[5]);
	mat[1][0] = sind(coor[3])*cosd(coor[4]);
	mat[1][1] = sind(coor[3])*sind(coor[4])*sind(coor[5]) + cosd(coor[3])*cosd(coor[5]);
	mat[1][2] = sind(coor[3])*sind(coor[4])*cosd(coor[5]) - cosd(coor[3])*sind(coor[5]);
	mat[2][0] = -sind(coor[4]);
	mat[2][1] = cosd(coor[4])*sind(coor[5]);
	mat[2][2] = cosd(coor[4])*cosd(coor[5]);

	mat[0][3] = coor[0];
	mat[1][3] = coor[1];
	mat[2][3] = coor[2];
	mat[3][3] = 1;

	return mat;
}

vec1D matrix2coor(vec2D mat)
{
	vec1D coor(6, 0);
	coor[0] = mat[0][3];
	coor[1] = mat[1][3];
	coor[2] = mat[2][3];

	coor[4] = asind(-mat[2][0]);

	if (coor[4] == 90 || coor[4] == -90)
	{
		coor.clear();
		return coor;
	}

	double sinC = mat[2][1] / cosd(coor[4]);
	double cosC = mat[2][2] / cosd(coor[4]);
	coor[5] = solveAorC(sinC, cosC);
	double cosA = mat[0][0] / cosd(coor[4]);
	double sinA = mat[1][0] / cosd(coor[4]);
	coor[3] = solveAorC(sinA, cosA);

	return coor;
}
#pragma endregion

//****************************************************************//
// 各种坐标系之间的转换，输入均为某个坐标系下的xyzabc
//****************************************************************//
#pragma region conversion between different coordinate systems
vec1D scan2tcs(vec1D pipePoint, vec1D pipeNorm, vec1D toolPoint)
{
	vec1D pipeTan{ -pipeNorm[1], pipeNorm[0], 0 };

	for (auto i = 0; i < pipeTan.size(); i++) {
		pipeNorm[i] /= vecNorm(pipeNorm);
		pipeTan[i] /= vecNorm(pipeTan);
	}
	vec1D pipeBitan = cross(pipeNorm, pipeTan);

	vec1D trans(pipePoint.size());
	double hoverDist = 5;

	for (auto i = 0; i < trans.size(); i++)
		trans[i] = pipePoint[i] - toolPoint[i] - hoverDist * pipeNorm[i];

	vec2D mat(4);

	pipeNorm.push_back(0);
	mat[0] = pipeNorm;
	pipeTan.push_back(0);
	mat[1] = pipeTan;
	pipeBitan.push_back(0);
	mat[2] = pipeBitan;
	trans.push_back(1);
	mat[3] = trans;

	return matrix2coor(matrixTranspose(mat));
}

vec1D scan2robot(vec1D vecScan)
{
	vec2D mat(4, vec1D(4, 0));
	vec1D vecRobot(6);
	vec1D vec1(4);
	vec1D vec2(4);

	ifstream fid("robotShining.txt", ios::in);

	for (auto i = 0; i < mat.size(); i++)
		for (auto j = 0; j < mat[0].size(); j++)
			fid >> mat[i][j];
	fid.close();
	
	vec1[3] = 1;
	for (auto i = 0; i < 3; i++) vec1[i] = vecScan[i];

	vec2 = vecTranspose(matrixMultiply(mat, vecTranspose(vec1)));
	for (auto i = 0; i < 3; i++) {
		vecRobot[i] = vec2[i];
		vec1[i] = vecScan[i + 3];
	}

	vec2 = vecTranspose(matrixMultiply(mat, vecTranspose(vec1)));
	for (auto i = 0; i < 3; i++) {
		vecRobot[i+3] = vec2[i];
		vec1[i] = 0;
	}

	vec2 = vecTranspose(matrixMultiply(mat, vecTranspose(vec1)));
	for (auto i = 0; i < 3; i++) {
		vecRobot[i + 3] -= vec2[i];
	}

	return vecRobot;
}

vec1D tcs2wcs(vec1D coorOffset, vec1D coorIni)
{
	/*
	vec2D vec = matrixMultiply(coor2matrix(coorIni), coor2matrix(coorOffset));

	for (auto i = 0; i < vec.size(); i++) {
		for (auto j = 0; j < vec[0].size(); j++)
			cout << vec[i][j] << "  ";
		cout << endl;
	}
	*/

	return matrix2coor(matrixMultiply(coor2matrix(coorIni), coor2matrix(coorOffset)));
}

vec1D tcs2wcsPref(vec1D coorOffset, vec1D coorFin)
{
	return matrix2coor(matrixMultiply(coor2matrix(coorFin), matrixInverse(coor2matrix(coorOffset))));
}

vec1D tcs2wcsPrefC(vec1D coorOffset, vec1D coorFin)
{
	vec1D coorPrefWcs = tcs2wcsPref(coorOffset, coorFin);

	double angle = getAngleC(coorOffset);

	vec1D rot1{0, 0, -180, 0, 0, 0};
	vec1D rot2{0, 0, 0, 0, 0, angle};
	vec1D rot3{0, 0, 180, 0, 0, 0 };

	return tcs2wcs(rot3, tcs2wcs(rot2, tcs2wcs(rot1, coorPrefWcs)));
}

vec1D wcs2tcs(vec1D coorFin, vec1D coorIni)
{
	return matrix2coor(matrixMultiply( matrixInverse(coor2matrix(coorIni)), coor2matrix(coorFin)));
}

#pragma endregion

//****************************************************************//
// 机器人视觉校正相关
//****************************************************************//
#pragma region correction algorithm
double getAngle(vec1D coor)
{
	vec2D mat = coor2matrix(coor);

	vec2D mat2 = matrixAdjoint(mat, mat.size()-1, mat.size()-1);

	vec1D vec{1,0,0};
	vec1D vec2 = vecTranspose(matrixMultiply(mat2, vecTranspose(vec)));

	return acosd(dot(vec2, vec) / vecNorm(vec2));
}

double getAngleC(vec1D coor)
{
	vec2D mat = coor2matrix(coor);

	vec2D mat2 = matrixAdjoint(mat, mat.size() - 1, mat.size() - 1);

	vec1D vec{ 1, 0, 0 };
	vec = vecTranspose(matrixMultiply(mat2, vecTranspose(vec)));

	vec1D vec1{ 0, 1, 0 };
	vec1D vec2{ 0, 1, -vec[1] / vec[2] };

	double angle = acosd(dot(vec1, vec2) / vecNorm(vec2));

	vec1D tmp = cross(vec1, vec2);
	if (tmp[0] < 0)	angle = -angle;

	return angle;
}

vec1D getCorrection(vec1D coor)
{
	vec2D coef(5, vec1D(5, 0));
	vec1D corr(5,0);


	ifstream fid("polyCoef.txt", ios::in);

	for (auto i = 0; i < corr.size(); i++)
		fid >> corr[i];

	for (auto i = 0; i < coef.size(); i++)
		for (auto j = 0; j < coef[0].size(); j++)
			fid >> coef[i][j];
	fid.close();

	for (auto i = 0; i < coef.size(); i++)
		for (auto j = 0; j < coef[0].size(); j++)
			corr[j] += coor[i] * coef[i][j];

	corr.push_back(0);

	return corr;
}

vec1D getCorrectionAngle(vec1D coor)
{
	vec2D coef(5, vec1D(5, 0));
	vec1D corrAngle(5, 0);

	vec1D coorAngle(5, 0);
	coorAngle[0] = coor[0];
	coorAngle[1] = coor[1];
	coorAngle[2] = coor[2];
	coorAngle[3] = getAngle(coor);
	coorAngle[4] = getAngleC(coor);

	ifstream fid("polyAngleCoef.txt", ios::in);

	for (auto i = 0; i < corrAngle.size(); i++)
		fid >> corrAngle[i];

	for (auto i = 0; i < coef.size(); i++)
		for (auto j = 0; j < coef[0].size(); j++)
			fid >> coef[i][j];
	fid.close();

	for (auto i = 0; i < coef.size(); i++)
		for (auto j = 0; j < coef[0].size(); j++)
			corrAngle[j] += coorAngle[i] * coef[i][j];

	for (auto i = 0; i < corrAngle.size(); i++) corrAngle[i] += coorAngle[i];


	vec1D corr{ corrAngle[0],corrAngle[1],corrAngle[2],0,0,0 };
	vec1D rot1{ 0,0,0,0,0,corrAngle[4] };
	vec1D rot2{ 0,0,0,0,corrAngle[3],0 };

	corr = tcs2wcs(rot2,tcs2wcs(rot1, corr));

	for (auto i = 0; i < 6; i++) corr[i] -= coor[i];

	return corr;

}

vec1D getResidual(vec1D coorFin, vec1D calFin)
{	
	vec1D res(3, 0);
	vec1D coorRes(6, 0);

	coorRes = wcs2tcs(coorFin, calFin);

	res[0] = getAngle(coorRes);

	res[1] = sqrt(coorRes[1]* coorRes[1] + coorRes[2]* coorRes[2]);

	res[2] = coorRes[0];

	return res;
}

#pragma endregion

#pragma region misc
void readFile(vec1D &vec, string filename)
{
	ifstream fid(filename, ios::in);

	for (auto i = 0; i < vec.size(); i++) fid >> vec[i];

	fid.close();
}

void readFile(vec2D &vec, string filename)
{
	ifstream fid(filename, ios::in);

	for (auto i = 0; i < vec.size(); i++)
		for (auto j = 0; j < vec[0].size(); j++)
			fid >> vec[i][j];

	fid.close();
}

void outputVec(vec1D vec)
{
	for (auto i = 0; i < vec.size(); i++) cout << vec[i] << " ";
	cout << endl;

}
#pragma endregion
