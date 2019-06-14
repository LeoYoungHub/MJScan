// CppTester.cpp : 定义 DLL 应用程序的导出函数。
//
#include <string>
#include <QObject>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QPixmap>
#include <QByteArray>
#include <vector>
#include <osgViewer/Viewer>
#include <osgDB/ReadFile>
#include <osg/Group>
#include <thread>
#include <QDialog>
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
extern "C" _declspec(dllexport) int _stdcall getTreatPoint(char* bestRef, char* kukaRef, char* scanRef, bool isBestPoint, dPoints* abutmentRef);
extern "C" _declspec(dllexport) bool _stdcall createPixmap(char* key, int offset, int width, int height, int channel, int rotate, uchar* imageP);
extern "C" _declspec(dllexport) bool _stdcall Scan2Robot(char* scanPoint, dPoints* robotPoint);