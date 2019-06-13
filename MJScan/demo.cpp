#include <iostream>
#include <fstream>
#include <sstream>
#include <iterator>
#include <vector>

#include "motionCorrection.h"

#include "math.h"

using namespace std;


//****************************************************************//
// ��������չʾ����У׼����
//****************************************************************//
int main()
{
	//	1. �����ǰ����ʼ�㡱��ť���û�����ǰ��Ԥ��õ�λ�á�			�û���������ָ��ʵ�֡�
	//	2. ���ú���Ȳ�����										���漰�����
	//	3. �����ɨ�衱���ӵ�����ѡȡ���β�ˡ�						���Ӿ�����ָ��ʵ�֡�
	//	   ������������ꡱ
	vec1D coorOffset(6,0);		// �Ӿ����������ķ��ͽ����������У������
	vec1D coorIni(6,0);			// ��ǰkuka�����˵�xyzabc����PLC�ж�ȡ
	vec1D coorPrefTcs(6,0);		// ��ѳ����(��������ϵ)�����ļ��ж�ȡ
	vec1D coorPrefWcs(6,0);		// ��ѳ����(��������ϵ)������õ�
	double needleDia = 4.3;		// �Ӿ���������ֱ��

	if (needleDia > 4.8) {
		cout << "ֱ��������������ɨ��" << endl;
		return 1;
	}

	readFile(coorPrefTcs, "robotPref.txt");				// ������ѳ��������	
	readFile(coorIni, "robotIni.txt");					// ���뵱ǰ��������̬��xyzabc��ʵ�����Ӧ��ȡPLC�����ݿ��á�
	readFile(coorOffset, "robotOffset.txt");			// ����������ڹ�������ϵ����Ҫ�˶���xyzzbc��ʵ�����Ӧ���Ӿ�����������ⲿ��xyzabc������У������
	//**************************//
	// ɨ��������ϵת��������������ϵ		20190613���
	coorOffset = scan2robot(coorOffset);
	//*************************//

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
		for (auto i = 0; i < 6; i++) cout << coorPrefWcs[i]<<" ";		// �����ļ�����"robotIniPref.txt"�����ݶԱȣ�һ����˵��������ȷ��
		cout << endl << endl;

		cout << "����������ѵ��ƫ��" << endl;
		cout << "�Ƕ�ƫ�" << res[0] << "��λ��ƫ�" << res[1] << endl << endl;

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
	vec1D coorBase(6, 0);								// �ԽӺ�����˵���̬
	readFile(coorBase, "robotBase.txt");				// �����������xyzzbc��ʵ�����Ӧ��PLC�ж�ȡ��


	//  8. �����ǰ������㡱
	//	�ص�coorPrefWcs��λ��

	//  9. �����ɨ�衱���ӵ�����ѡȡ���β�ˡ�						���Ӿ�����ָ��ʵ�֡�
	//	   �����У׼��

	vec1D coorOffsetPref(6, 0);
	readFile(coorOffsetPref, "robotOffsetPref.txt");			// ����������ڹ�������ϵ����Ҫ�˶���xyzabc��ʵ�����Ӧ���Ӿ�����������ⲿ��xyzabc������У������
						
//**************************//
	// ɨ��������ϵת��������������ϵ		20190613���
	coorOffsetPref = scan2robot(coorOffsetPref);
//*************************//


	vec1D coorCorr(6, 0);
	coorCorr = getCorrectionAngle(coorOffsetPref);				// ����У����

	cout << "��ǰУ����:"<<endl;
	outputVec(coorCorr);
	cout << endl;
			
	for (auto i = 0; i < 6; i++) coorOffsetPref[i]+=coorCorr[i];	// ����У�������¹�������ϵ�µ�xyzabc
																	// ����ϣ���ڵõ���������ϵ�µ�xyzabc����ʹ��tcs2wcs(coorOffsetPref, coorPrefWcs)														

	res = getResidual(coorBase, tcs2wcs(coorOffsetPref, coorPrefWcs));				// ����ʣ������ʣ�����ļ������Դ�����������ϵ��ת��

	if (res[0] > 2.5 || res[1] > 2.5)
	{
		cout << "������У׼ʧ��" << endl;

		cout << "У�����ʣ����" << endl;
		for (auto i = 0; i < 3; i++) cout << res[i] << " ";		// �ֱ�Ϊ�Ƕȡ�yzƽ���ϵľ������x����ľ���
		cout << endl;

		return 3;
	}
	else {
		cout << "У�����ʣ����" << endl;
		for (auto i = 0; i < 3; i++) cout << res[i] << " ";		// �ֱ�Ϊ�Ƕȡ�yzƽ���ϵľ������x����ľ���
		cout << endl<<endl;

		cout << "�������Χ�ڣ�����������" << endl;
	}
}


