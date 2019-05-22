// CppTester.cpp : 定义 DLL 应用程序的导出函数。
//
#include <string>
using namespace std;
//更新相机设置
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