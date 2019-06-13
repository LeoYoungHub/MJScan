#include "CppTester.h"
#include "cylinderFitting.h"
#include "motionCorrection.h"
#include <iostream>
#include <fstream>
#include <string>
#include  "stdlib.h"
#include "unsupported/Eigen/NumericalDiff"
#include "unsupported/Eigen/NonLinearOptimization"
#include <QtDebug>
#include <QSharedMemory>
#include <QMessageBox>
#include <osg/ShapeDrawable>
#include <QJsonObject>
#include <QJsonDocument>
#include <QFile>
#include <QByteArray>
#include <QProcess>
#include <QBuffer>

char* doubleToChar(double num)
{
	char str[256];
	sprintf(str, "%lf", num);

	return str;
}

bool  _stdcall selectedPoints(char* path, points* outStr)
{
	try {
		std::vector<Eigen::Vector3d> points;
		std::vector<Eigen::Vector3d> normals;

		ifstream fin(path, ios::in | ios::binary);

		if (!fin.good())
		{
			cout << "文件打开错误" << endl;
			return false;
		}
		double readX = 0;
		double readY = 0;
		double readZ = 0;
		fin.seekg(0, ios::beg);
		Eigen::Vector3d iter;
		while (fin.peek() != EOF)
		{
			fin.read(reinterpret_cast<char*>(&readX), sizeof(double));
			fin.read(reinterpret_cast<char*>(&readY), sizeof(double));
			fin.read(reinterpret_cast<char*>(&readZ), sizeof(double));
			iter.x() = readX;
			iter.y() = readY;
			iter.z() = readZ;
			points.push_back(iter);

			fin.read(reinterpret_cast<char*>(&readX), sizeof(double));
			fin.read(reinterpret_cast<char*>(&readY), sizeof(double));
			fin.read(reinterpret_cast<char*>(&readZ), sizeof(double));
			iter.x() = readX;
			iter.y() = readY;
			iter.z() = readZ;
			normals.push_back(iter);
		}
		cout << "points Size:" << points.size() << endl;
		if (points.size() > 0 && normals.size() > 0)
		{
			Sn3DAlgorithm::CylinderFitting fitting(points, normals);

			std::vector<double> para;
			fitting.cylinder_fitting(para);
			cout << "para Size1:" << para.size() << endl;
			if (para.size() == 11)
			{
				for (int i = 0; i < 11; ++i)
				{
					cout << para[i] << endl;
					char* a = doubleToChar(para[i]);
					strcpy(outStr->user[i], a);					
				}
			}

			return true;
		}
		return false;

	}
	catch (string e) {
		cout << "exception" << e << endl;
	}	
	return false;
}


string doubleToString(double num)
{
	char str[256];
	sprintf(str, "%lf", num);
	string result = str;
	return result;
}

vector<double> split(const string &str, const string &pattern)
{
	//const char* convert to char*
	char * strc = new char[strlen(str.c_str()) + 1];
	strcpy(strc, str.c_str());
	vector<double> resultVec;
	char* tmpStr = strtok(strc, pattern.c_str());
	while (tmpStr != NULL)
	{
		resultVec.push_back(strtod(tmpStr,NULL));
		tmpStr = strtok(NULL, pattern.c_str());
	}

	delete[] strc;

	return resultVec;
}

int _stdcall getTreatPoint(char* bestRef, char* kukaRef, char* scanRef, dPoints* abutmentRef)
{
	//	1. 点击“前往起始点”按钮，让机器人前往预设好的位置。			用机器人已有指令实现。
	//	2. 放置海绵等操作。										不涉及软件。
	//	3. 点击“扫描”，从点云中选取针尖尾端。						用视觉已有指令实现。
	//	   点击“发送坐标”
	vec1D coorOffset(6, 0);		// 视觉软件计算出的发送结果（不包含校正量）
	vec1D coorIni(6, 0);			// 当前kuka机器人的xyzabc，从PLC中读取
	vec1D coorPrefTcs(6, 0);		// 最佳成像点(工具坐标系)，从文件中读取
	vec1D coorPrefWcs(6, 0);		// 最佳成像点(世界坐标系)，计算得到
	double needleDia = 4.3;		// 视觉反馈的针直径

	if (needleDia > 4.8) {
		cout << "直径误差过大，请重新扫描" << endl;
		return 1;
	}

	coorPrefTcs = split(bestRef, " ");				// 读入最佳成像点数据	
	coorIni = split(kukaRef, " ");					// 读入当前机器人姿态的xyzabc。实际情况应读取PLC的数据块获得。
	coorOffset = split(scanRef, " ");			// 读入机器人在工具坐标系下需要运动的xyzzbc。实际情况应由视觉软件给出。这部分xyzabc不包含校正量。

	vec1D res;
	coorPrefWcs = tcs2wcsPrefC(coorPrefTcs, tcs2wcs(coorOffset, coorIni));

	res = getResidual(coorPrefTcs, coorOffset);

	if (res[0] > 4 || res[1] > 10)						// 这个数值需要进一步测试定下来，暂定4和10
	{
		cout << "偏离最佳成像区域，请点击前往最佳点" << endl << endl;

		// 3.1 点击“前往最佳点”
		// 将coorPrefWcs的值写入到PLC对应的DB块中，在使能的情况下点击“前往最佳点”，会让机器人重新定位，此后回归到第3步。

		// DEBUG
		cout << "以下是当前最佳点" << endl;
		for (auto i = 0; i < 6; i++) cout << coorPrefWcs[i] << " ";		// 请与文件夹中"robotIniPref.txt"中数据对比，一致则说明计算正确。
		cout << endl << endl;

		cout << "以下是与最佳点的偏差" << endl;
		cout << "角度偏差：" << res[0] << "　位置偏差：" << res[1] << endl << endl;

		for (auto i = 0; i < 6; i++)
		{
			char* a = doubleToChar(coorPrefWcs[i]);
			strcpy(abutmentRef->point[i], a);
		}// 根据校正量更新工具坐标系下的xyzabc

		return 2;
	}
	else {
		cout << "以下是与最佳点的偏差" << endl;
		cout << "角度偏差：" << res[0] << "　位置偏差：" << res[1] << endl << endl;

		cout << "已在最佳成像区域，请继续执行" << endl << endl;			// "robotIniPref.txt"中的数据可忽略，都在最佳成像区域内，不必要强行要求一致。
		coorPrefWcs = coorIni;
	}

	//	4. 点击“记录拍摄点”		
	//	此时的拍摄点，也就是机器人的姿态，应该是前面计算得到的coorPrefWcs

	//	5. 点击“前往对接点” 
	//  直接将coorOffset（工具坐标系）发送给PLC，进行对接
	//	如果将来机器人的操作改成了在世界坐标系下运动，则请进行下面的操作
	//  tcs2wcs(coorOffset, coorPrefWcs)
	//  输出的量即为世界坐标系下的机器人姿态（这部分之前让杨工写过，这里其实重复了）
	//  以上操作均不含校正量，软件界面上的correction应该都是0

	//	6. 不涉及软件
	//  7. 点击“记录对接点”
	//vec1D coorBase(6, 0);								// 对接后机器人的姿态
	//readFile(coorBase, "robotBase.txt");				// 读入机器人在xyzzbc。实际情况应从PLC中读取。


	//  8. 点击“前往拍摄点”
	//	回到coorPrefWcs的位置

	//  9. 点击“扫描”，从点云中选取针尖尾端。						用视觉已有指令实现。
	//	   点击“校准”

	//vec1D coorOffsetPref(6, 0);
	//readFile(coorOffsetPref, "robotOffsetPref.txt");			// 读入机器人在工具坐标系下需要运动的xyzabc。实际情况应由视觉软件给出。这部分xyzabc不包含校正量。

	vec1D coorCorr(6, 0);
	coorCorr = getCorrectionAngle(coorOffset);				// 计算校正量

	cout << "当前校正量:" << endl;
	outputVec(coorCorr);
	cout << endl;

	for (auto i = 0; i < 6; i++)
	{
		coorOffset[i] += coorCorr[i];

		char* a = doubleToChar(coorOffset[i]);
		strcpy(abutmentRef->point[i], a);
	}// 根据校正量更新工具坐标系下的xyzabc
	
	//// 如若希望在得到世界坐标系下的xyzabc，请使用tcs2wcs(coorOffsetPref, coorPrefWcs)														

	//res = getResidual(coorBase, tcs2wcs(coorOffsetPref, coorPrefWcs));				// 计算剩余量，剩余量的计算中自带了世界坐标系的转换

	//if (res[0] > 2.5 || res[1] > 2.5)
	//{
	//	cout << "误差过大，校准失败" << endl;

	//	cout << "校正后的剩余量" << endl;
	//	for (auto i = 0; i < 3; i++) cout << res[i] << " ";		// 分别为角度、yz平面上的距离和沿x方向的距离
	//	cout << endl;

	//	return 3;
	//}
	//else {
	//	cout << "校正后的剩余量" << endl;
	//	for (auto i = 0; i < 3; i++) cout << res[i] << " ";		// 分别为角度、yz平面上的距离和沿x方向的距离
	//	cout << endl << endl;

	//	cout << "误差允许范围内，可正常运行" << endl;
	//}
}

void createPixmap(unsigned char* data, int width, int height, int channel, int rotate, uchar* imageP)
{
	int sizeColor = width * height * 3;
	int sizeSingle = width * height * channel;
	int byte_per_line = 0;
	unsigned char* pDataColor = new unsigned char[sizeColor];
	if (3 == channel)
	{
		byte_per_line = width*channel;
		memcpy(pDataColor, data, sizeSingle);
	}
	else
	{
		for (int i = 0; i < sizeSingle; i++)
		{
			pDataColor[i * 3] = data[i];
			pDataColor[i * 3 + 1] = data[i];
			pDataColor[i * 3 + 2] = data[i];
			if (data[i] > 230)
			{
				pDataColor[i * 3] = 255;
				pDataColor[i * 3 + 1] = 0;
				pDataColor[i * 3 + 2] = 0;
			}
		}// 显示红色 add by lzy
	}

	QMatrix left_matrix_;
	left_matrix_.rotate(rotate);


	QImage* image_ = 0;
	if (byte_per_line > 0)
		image_ = new QImage(pDataColor, width, height, byte_per_line, QImage::Format_RGB888);
	else
		image_ = new QImage(pDataColor, width, height, QImage::Format_RGB888);

	if (width != 1280)
		*image_ = image_->transformed(left_matrix_).mirrored(true); //sign 1121
	else
		*image_ = image_->transformed(left_matrix_); //sign 1121

	QString imagePath =  "image.bmp";
	image_->save(imagePath, "BMP");

	cout << image_->byteCount() << endl;
	memcpy(imageP, image_->bits(), image_->byteCount());

	delete image_;
	delete[]pDataColor;
	pDataColor = NULL;
}

bool _stdcall createPixmap(char* key, int offset, int width, int height, int channel, int rotate, uchar* imageP)
{
	cout << offset << key << width << channel << height << rotate << endl;
	QSharedMemory shm;
	//QString str = QChar(*key);
	shm.setNativeKey(key);
	auto result = shm.attach(QSharedMemory::ReadWrite);
	if (!result){
		cout << "video1 memory:cannot attach to " << key << endl;
		return false;
	}

	void *p = shm.data();
	if (nullptr == p){
		cout << "share memory Pointer is null\n" << endl;
		return false;
	}

	cout << shm.size() <<" "<<shm.isAttached()<< endl;
	auto data = static_cast<unsigned char*>(shm.data()) + offset;
	
	createPixmap(data, width, height, channel, rotate, imageP);
	return true;
}

bool _stdcall Scan2Robot(char* scanPoint, dPoints* robotPoint)
{
	vec1D coorOffset(6, 0);		// 视觉软件计算出的发送结果（不包含校正量）

	coorOffset = split(scanPoint, " ");			// 读入机器人在工具坐标系下需要运动的xyzzbc。实际情况应由视觉软件给出。这部分xyzabc不包含校正量。

	coorOffset = scan2robot(coorOffset);

	for (auto i = 0; i < 6; i++)
	{
		char* a = doubleToChar(coorOffset[i]);
		strcpy(robotPoint->point[i], a);
	}// 根据校正量更新工具坐标系下的xyzabc
	return true;
}