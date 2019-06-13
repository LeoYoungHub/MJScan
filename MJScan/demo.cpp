#include <iostream>
#include <fstream>
#include <sstream>
#include <iterator>
#include <vector>

#include "motionCorrection.h"

#include "math.h"

using namespace std;


//****************************************************************//
// 主函数，展示整个校准流程
//****************************************************************//
int main()
{
	//	1. 点击“前往起始点”按钮，让机器人前往预设好的位置。			用机器人已有指令实现。
	//	2. 放置海绵等操作。										不涉及软件。
	//	3. 点击“扫描”，从点云中选取针尖尾端。						用视觉已有指令实现。
	//	   点击“发送坐标”
	vec1D coorOffset(6,0);		// 视觉软件计算出的发送结果（不包含校正量）
	vec1D coorIni(6,0);			// 当前kuka机器人的xyzabc，从PLC中读取
	vec1D coorPrefTcs(6,0);		// 最佳成像点(工具坐标系)，从文件中读取
	vec1D coorPrefWcs(6,0);		// 最佳成像点(世界坐标系)，计算得到
	double needleDia = 4.3;		// 视觉反馈的针直径

	if (needleDia > 4.8) {
		cout << "直径误差过大，请重新扫描" << endl;
		return 1;
	}

	readFile(coorPrefTcs, "robotPref.txt");				// 读入最佳成像点数据	
	readFile(coorIni, "robotIni.txt");					// 读入当前机器人姿态的xyzabc。实际情况应读取PLC的数据块获得。
	readFile(coorOffset, "robotOffset.txt");			// 读入机器人在工具坐标系下需要运动的xyzzbc。实际情况应由视觉软件给出。这部分xyzabc不包含校正量。
	//**************************//
	// 扫描器坐标系转换至机器人坐标系		20190613添加
	coorOffset = scan2robot(coorOffset);
	//*************************//

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
		for (auto i = 0; i < 6; i++) cout << coorPrefWcs[i]<<" ";		// 请与文件夹中"robotIniPref.txt"中数据对比，一致则说明计算正确。
		cout << endl << endl;

		cout << "以下是与最佳点的偏差" << endl;
		cout << "角度偏差：" << res[0] << "　位置偏差：" << res[1] << endl << endl;

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
	vec1D coorBase(6, 0);								// 对接后机器人的姿态
	readFile(coorBase, "robotBase.txt");				// 读入机器人在xyzzbc。实际情况应从PLC中读取。


	//  8. 点击“前往拍摄点”
	//	回到coorPrefWcs的位置

	//  9. 点击“扫描”，从点云中选取针尖尾端。						用视觉已有指令实现。
	//	   点击“校准”

	vec1D coorOffsetPref(6, 0);
	readFile(coorOffsetPref, "robotOffsetPref.txt");			// 读入机器人在工具坐标系下需要运动的xyzabc。实际情况应由视觉软件给出。这部分xyzabc不包含校正量。
						
//**************************//
	// 扫描器坐标系转换至机器人坐标系		20190613添加
	coorOffsetPref = scan2robot(coorOffsetPref);
//*************************//


	vec1D coorCorr(6, 0);
	coorCorr = getCorrectionAngle(coorOffsetPref);				// 计算校正量

	cout << "当前校正量:"<<endl;
	outputVec(coorCorr);
	cout << endl;
			
	for (auto i = 0; i < 6; i++) coorOffsetPref[i]+=coorCorr[i];	// 根据校正量更新工具坐标系下的xyzabc
																	// 如若希望在得到世界坐标系下的xyzabc，请使用tcs2wcs(coorOffsetPref, coorPrefWcs)														

	res = getResidual(coorBase, tcs2wcs(coorOffsetPref, coorPrefWcs));				// 计算剩余量，剩余量的计算中自带了世界坐标系的转换

	if (res[0] > 2.5 || res[1] > 2.5)
	{
		cout << "误差过大，校准失败" << endl;

		cout << "校正后的剩余量" << endl;
		for (auto i = 0; i < 3; i++) cout << res[i] << " ";		// 分别为角度、yz平面上的距离和沿x方向的距离
		cout << endl;

		return 3;
	}
	else {
		cout << "校正后的剩余量" << endl;
		for (auto i = 0; i < 3; i++) cout << res[i] << " ";		// 分别为角度、yz平面上的距离和沿x方向的距离
		cout << endl<<endl;

		cout << "误差允许范围内，可正常运行" << endl;
	}
}


