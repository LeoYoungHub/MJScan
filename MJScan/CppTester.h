// CppTester.cpp : ���� DLL Ӧ�ó���ĵ���������
//
#include <string>
using namespace std;
//�����������
struct points
{
	char user[11][256];
};
struct dPoints
{
	char point[6][256];
};

extern "C" _declspec(dllexport)  bool _stdcall selectedPoints(char* path, points* outStr);
extern "C" _declspec(dllexport) int _stdcall getTreatPoint(char* bestRef, char* kukaRef, char* scanRef, dPoints* abutmentRef);