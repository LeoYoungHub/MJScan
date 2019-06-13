#include <iostream>
#include <fstream>
#include <vector>

#include "math.h"

using namespace std;

using vec1D = vector<double>;
using vec2D = vector<vector<double>>;


//****************************************************************//
// �������Ǻ���������ֵ�ͷ���ֵ��λ��Ϊ��
//****************************************************************//
double asind(double a);
double acosd(double a);
double sind(double a);
double cosd(double a);


//****************************************************************//
// �����ľ���������������桢������ʽ������˷��Լ��������
//****************************************************************//
vec2D matrixAdjoint(vec2D a, int x, int y);
double matrixDet(vec2D a);
vec2D matrixInverse(vec2D a);
vec2D matrixTranspose(vec2D a);
vec2D matrixMultiply(vec2D a, vec2D b);
double vecNorm(vec1D a);
vec1D cross(vec1D, vec1D);
double dot(vec1D);

//****************************************************************//
// ��������̬xyzabc�Ͷ�Ӧ�ı任�����໥ת��
//****************************************************************//
double solveAorC(double sinC, double cosC);
vec2D coor2matrix(vec1D coor);
vec1D matrix2coor(vec2D mat);
vec1D tcs2wcs(vec1D, vec1D);
vec1D wcs2tcs(vec1D, vec1D);
vec1D tcs2wcsPref(vec1D, vec1D);
vec1D tcs2wcsPrefC(vec1D , vec1D);
vec1D scan2robot(vec1D);
//****************************************************************//
// �������Ӿ�У�����
//****************************************************************//
double getAngle(vec1D coor);
double getAngleC(vec1D coor);
vec1D getCorrection(vec1D coor);
vec1D getCorrectionAngle(vec1D coor);
vec1D getResidual(vec1D, vec1D);
//****************************************************************//
// ����
//****************************************************************//
void readFile(vec1D&, string);
void readFile(vec2D&, string);
void outputVec(vec1D);