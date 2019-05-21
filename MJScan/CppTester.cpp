#include "CppTester.h"
#include "cylinderFitting.h"
#include <iostream>
#include <fstream>
#include <string>
#include  "stdlib.h"
#include "unsupported/Eigen/NumericalDiff"
#include "unsupported/Eigen/NonLinearOptimization"

char* doubleToChar(double num)
{
	char str[256];
	sprintf(str, "%lf", num);

	return str;
}

//Sn3DAlgorithm::CylinderFitting fitting;
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
					//sprintf(outStr[i], doubleToChar(para[i]));
					
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



bool _stdcall cylinderFitting(char** str)
{
	try {
		std::vector<double> para;
		//fitting.cylinder_fitting(para);
		cout << "para Size:" << para.size() << endl;
		if (para.size() ==11)
		{
			for (int i = 0; i < 11; ++i)
			{
				sprintf(str[i], doubleToChar(para[i]));
				cout << str[i] << endl;
			}
		}
	}
	catch (string e) {
		cout << "exception" << e << endl;
	}
	return true;
}