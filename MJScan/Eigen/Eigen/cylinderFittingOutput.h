#include <Eigen/Dense>
#include <Eigen/StdVector>

#pragma comment(lib,"./lib/cylinderFittingLibd.lib")
/**
*  \brief 点云数据，圆柱拟合。
*  \param[in]  cloud，待处理点云数据。
*  \param[out] para，拟合的圆柱参数。para[0-2]轴线顶点，para[3-5]轴线基点，para[6-8]轴线方向向量（由基点指向顶点），para[9]高度,para[10]半径。
*  \param[in]  radius，圆柱半径，默认为负数需估计半径，如给定正值半径则不进行半径的估计。
*/
bool cloud_cylinder_fitting(const std::vector<Eigen::Vector3d>& points, const std::vector<Eigen::Vector3d>& normals, std::vector<double>& para, const double radius = -1);