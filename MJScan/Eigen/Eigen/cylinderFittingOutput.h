#include <Eigen/Dense>
#include <Eigen/StdVector>

#pragma comment(lib,"./lib/cylinderFittingLibd.lib")
/**
*  \brief �������ݣ�Բ����ϡ�
*  \param[in]  cloud��������������ݡ�
*  \param[out] para����ϵ�Բ��������para[0-2]���߶��㣬para[3-5]���߻��㣬para[6-8]���߷����������ɻ���ָ�򶥵㣩��para[9]�߶�,para[10]�뾶��
*  \param[in]  radius��Բ���뾶��Ĭ��Ϊ��������ư뾶���������ֵ�뾶�򲻽��а뾶�Ĺ��ơ�
*/
bool cloud_cylinder_fitting(const std::vector<Eigen::Vector3d>& points, const std::vector<Eigen::Vector3d>& normals, std::vector<double>& para, const double radius = -1);