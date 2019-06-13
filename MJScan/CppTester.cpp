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
			cout << "�ļ��򿪴���" << endl;
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
	//	1. �����ǰ����ʼ�㡱��ť���û�����ǰ��Ԥ��õ�λ�á�			�û���������ָ��ʵ�֡�
	//	2. ���ú���Ȳ�����										���漰�����
	//	3. �����ɨ�衱���ӵ�����ѡȡ���β�ˡ�						���Ӿ�����ָ��ʵ�֡�
	//	   ������������ꡱ
	vec1D coorOffset(6, 0);		// �Ӿ����������ķ��ͽ����������У������
	vec1D coorIni(6, 0);			// ��ǰkuka�����˵�xyzabc����PLC�ж�ȡ
	vec1D coorPrefTcs(6, 0);		// ��ѳ����(��������ϵ)�����ļ��ж�ȡ
	vec1D coorPrefWcs(6, 0);		// ��ѳ����(��������ϵ)������õ�
	double needleDia = 4.3;		// �Ӿ���������ֱ��

	if (needleDia > 4.8) {
		cout << "ֱ��������������ɨ��" << endl;
		return 1;
	}

	coorPrefTcs = split(bestRef, " ");				// ������ѳ��������	
	coorIni = split(kukaRef, " ");					// ���뵱ǰ��������̬��xyzabc��ʵ�����Ӧ��ȡPLC�����ݿ��á�
	coorOffset = split(scanRef, " ");			// ����������ڹ�������ϵ����Ҫ�˶���xyzzbc��ʵ�����Ӧ���Ӿ�����������ⲿ��xyzabc������У������

	vec1D res;
	coorPrefWcs = tcs2wcsPrefC(coorPrefTcs, tcs2wcs(coorOffset, coorIni));

	res = getResidual(coorPrefTcs, coorOffset);

	if (res[0] > 4 || res[1] > 10)						// �����ֵ��Ҫ��һ�����Զ��������ݶ�4��10
	{
		cout << "ƫ����ѳ�����������ǰ����ѵ�" << endl << endl;

		// 3.1 �����ǰ����ѵ㡱
		// ��coorPrefWcs��ֵд�뵽PLC��Ӧ��DB���У���ʹ�ܵ�����µ����ǰ����ѵ㡱�����û��������¶�λ���˺�ع鵽��3����

		// DEBUG
		cout << "�����ǵ�ǰ��ѵ�" << endl;
		for (auto i = 0; i < 6; i++) cout << coorPrefWcs[i] << " ";		// �����ļ�����"robotIniPref.txt"�����ݶԱȣ�һ����˵��������ȷ��
		cout << endl << endl;

		cout << "����������ѵ��ƫ��" << endl;
		cout << "�Ƕ�ƫ�" << res[0] << "��λ��ƫ�" << res[1] << endl << endl;

		for (auto i = 0; i < 6; i++)
		{
			char* a = doubleToChar(coorPrefWcs[i]);
			strcpy(abutmentRef->point[i], a);
		}// ����У�������¹�������ϵ�µ�xyzabc

		return 2;
	}
	else {
		cout << "����������ѵ��ƫ��" << endl;
		cout << "�Ƕ�ƫ�" << res[0] << "��λ��ƫ�" << res[1] << endl << endl;

		cout << "������ѳ������������ִ��" << endl << endl;			// "robotIniPref.txt"�е����ݿɺ��ԣ�������ѳ��������ڣ�����Ҫǿ��Ҫ��һ�¡�
		coorPrefWcs = coorIni;
	}

	//	4. �������¼����㡱		
	//	��ʱ������㣬Ҳ���ǻ����˵���̬��Ӧ����ǰ�����õ���coorPrefWcs

	//	5. �����ǰ���Խӵ㡱 
	//  ֱ�ӽ�coorOffset����������ϵ�����͸�PLC�����жԽ�
	//	������������˵Ĳ����ĳ�������������ϵ���˶��������������Ĳ���
	//  tcs2wcs(coorOffset, coorPrefWcs)
	//  ���������Ϊ��������ϵ�µĻ�������̬���ⲿ��֮ǰ���д����������ʵ�ظ��ˣ�
	//  ���ϲ���������У��������������ϵ�correctionӦ�ö���0

	//	6. ���漰���
	//  7. �������¼�Խӵ㡱
	//vec1D coorBase(6, 0);								// �ԽӺ�����˵���̬
	//readFile(coorBase, "robotBase.txt");				// �����������xyzzbc��ʵ�����Ӧ��PLC�ж�ȡ��


	//  8. �����ǰ������㡱
	//	�ص�coorPrefWcs��λ��

	//  9. �����ɨ�衱���ӵ�����ѡȡ���β�ˡ�						���Ӿ�����ָ��ʵ�֡�
	//	   �����У׼��

	//vec1D coorOffsetPref(6, 0);
	//readFile(coorOffsetPref, "robotOffsetPref.txt");			// ����������ڹ�������ϵ����Ҫ�˶���xyzabc��ʵ�����Ӧ���Ӿ�����������ⲿ��xyzabc������У������

	vec1D coorCorr(6, 0);
	coorCorr = getCorrectionAngle(coorOffset);				// ����У����

	cout << "��ǰУ����:" << endl;
	outputVec(coorCorr);
	cout << endl;

	for (auto i = 0; i < 6; i++)
	{
		coorOffset[i] += coorCorr[i];

		char* a = doubleToChar(coorOffset[i]);
		strcpy(abutmentRef->point[i], a);
	}// ����У�������¹�������ϵ�µ�xyzabc
	
	//// ����ϣ���ڵõ���������ϵ�µ�xyzabc����ʹ��tcs2wcs(coorOffsetPref, coorPrefWcs)														

	//res = getResidual(coorBase, tcs2wcs(coorOffsetPref, coorPrefWcs));				// ����ʣ������ʣ�����ļ������Դ�����������ϵ��ת��

	//if (res[0] > 2.5 || res[1] > 2.5)
	//{
	//	cout << "������У׼ʧ��" << endl;

	//	cout << "У�����ʣ����" << endl;
	//	for (auto i = 0; i < 3; i++) cout << res[i] << " ";		// �ֱ�Ϊ�Ƕȡ�yzƽ���ϵľ������x����ľ���
	//	cout << endl;

	//	return 3;
	//}
	//else {
	//	cout << "У�����ʣ����" << endl;
	//	for (auto i = 0; i < 3; i++) cout << res[i] << " ";		// �ֱ�Ϊ�Ƕȡ�yzƽ���ϵľ������x����ľ���
	//	cout << endl << endl;

	//	cout << "�������Χ�ڣ�����������" << endl;
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
		}// ��ʾ��ɫ add by lzy
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
	vec1D coorOffset(6, 0);		// �Ӿ����������ķ��ͽ����������У������

	coorOffset = split(scanPoint, " ");			// ����������ڹ�������ϵ����Ҫ�˶���xyzzbc��ʵ�����Ӧ���Ӿ�����������ⲿ��xyzabc������У������

	coorOffset = scan2robot(coorOffset);

	for (auto i = 0; i < 6; i++)
	{
		char* a = doubleToChar(coorOffset[i]);
		strcpy(robotPoint->point[i], a);
	}// ����У�������¹�������ϵ�µ�xyzabc
	return true;
}