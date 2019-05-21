/***************************************************
* 本文件定义 圆柱拟合
* \author 胡俊
***************************************************/
#ifndef TEST_SN3D_ALGORITHM_CYLINDER_FITTING_H
#define TEST_SN3D_ALGORITHM_CYLINDER_FITTING_H

#define M_PI  3.14159265358979323846

#include <Eigen/Dense>
#include <Eigen/StdVector>

namespace Sn3DAlgorithm
{
	/** \brief 实现 参数不敏感的圆柱拟合，以及指定半径的圆柱拟合。
	*/
	class CylinderFitting
	{
	public:
		/** \brief 构造、析构
		*  \param[in]  points，待处理点云数据点。
		*  \param[in]  normals，待处理点云数据点的法矢。
		*/
		CylinderFitting(const std::vector<Eigen::Vector3d>& points, const std::vector<Eigen::Vector3d>& normals)
			: sampleSize_(2)
			, modelSize_(7)
			, samplesRadius_(0.)
			, maxIterations_(1000)
			, maxSampleChecks_(1000)
			, normalDistanceWeight_(0.8)
			, probability_(0.99)
			, radiusMin_(-DBL_MAX)
			, radiusMax_(DBL_MAX)
			, threshold_(DBL_MAX)
		{
			points_ = points;
			normals_ = normals;
			indices_.resize(points_.size());
			for (int i = 0; i < points_.size(); ++i)
				indices_[i] = i;
			shuffledIndices_ = indices_;
		};
		CylinderFitting(){};

		~CylinderFitting(){}
	public:
		/** \brief 点云数据，圆柱拟合。
		*  \param[out] para，拟合的圆柱参数。para[0-2]轴线顶点，para[3-5]轴线基点，para[6-8]轴线方向向量，para[9]高度,para[10]半径。
		*  \param[in]  radius，圆柱半径，默认为负数需估计半径，如给定正值半径则不进行半径的估计。
		*/
		bool cylinder_fitting(std::vector<double>& para, const double radius = -1);

		//<计算拟合误差
		double get_the_fitting_error(const Eigen::VectorXd &coefficients);

		//<设置最大迭代次数
		void set_max_iterations(const int& maxIterations){ maxIterations_ = maxIterations; }

		//<设置阈值
		void set_threshold(const double& threshold){ threshold_ = threshold; }

		//<设置圆柱半径最小值
		void set_radius_min(const double& radiusMin){ radiusMin_ = radiusMin; }

		//<设置圆柱半径最大值
		void set_radius_max(const double& radiusMax){ radiusMax_ = radiusMax; }

		//<设置采样最大查看次数
		void max_sample_checks(const double& maxSampleChecks){ maxSampleChecks_ = maxSampleChecks; }

		//设置法矢距离权重
		void max_normal_distance_weight(const double& normalDistanceWeight){ normalDistanceWeight_ = normalDistanceWeight; }

	private:
		//<LeastMedianSquares represents an implementation of the LMedS (Least Median of Squares) algorithm. LMedS 
		//<is a RANSAC - like model - fitting algorithm that can tolerate up to 50 % outliers without requiring thresholds to be set.
		bool ransac_Lmeds_cylinder_fitting(std::vector<int>& inliers, Eigen::VectorXd& coefficients);

		//<ransac
		bool ransac_cylinder_fitting(std::vector<int>& inliers, Eigen::VectorXd& coefficients, const double& threshold);

		//<随机采用（TODO）
		void get_samples(int &iterations, std::vector<int> &samples);

		//<由采样点计算圆柱参数
		bool compute_model_coefficients(const std::vector<int> &samples, Eigen::VectorXd &coefficients);

		//<计算点云上的点到拟合的圆柱轴线上的距离
		void get_distances_to_model(const Eigen::VectorXd& modelCoefficients, std::vector<double>& distances);

		void select_within_distance(const Eigen::VectorXd &coefficients, std::vector<int> &inliers, const double& threshold);

		int count_within_distance(const Eigen::VectorXd &coefficients, const double& threshold);

		//<非线性最小二乘圆柱拟合
		void non_liner_least_squares_model_fitting(const std::vector<int>& inliers, Eigen::VectorXd &coefficients);

		//<非线性最小二乘圆柱拟合
		void non_liner_least_squares_model_fitting_with_radius(const double& radius, std::vector<int>& inliers, Eigen::VectorXd &coefficients);

		//<重新计算需要的参数
		void refine_para(const std::vector<int>& inliers, const Eigen::VectorXd& coefficients, std::vector<double>& para, const double radius = -1);
	private:
		//<获得内点
		std::vector<int> get_indices(){ return indices_; }

		//<获得阈值
		double get_threshold(){ return threshold_; }

		//<获得采样点数
		int get_sample_size(){ return sampleSize_; }

	private:
		static double pointToLineDistance(const Eigen::Vector3d& pt, const Eigen::Vector3d& linePt, const Eigen::Vector3d& lineDir)
		{
			// Calculate the distance from the point to the line
			// D = ||(P2-P1) x (P1-P0)|| / ||P2-P1|| = norm (cross (p2-p1, p1-p0)) / norm(p2-p1)
			return sqrt(lineDir.cross(linePt - pt).squaredNorm() / lineDir.squaredNorm());
		}

		static double sqrtPointToLineDistance(const Eigen::Vector3d& pt, const Eigen::Vector3d& linePt, const Eigen::Vector3d& lineDir)
		{
			// Calculate the distance from the point to the line
			// D = ||(P2-P1) x (P1-P0)|| / ||P2-P1|| = norm (cross (p2-p1, p1-p0)) / norm(p2-p1)
			return lineDir.cross(linePt - pt).squaredNorm() / lineDir.squaredNorm();
		}

		static double getAngle3D(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2, const bool inDegree = false)
		{
			// Compute the actual angle
			double rad = v1.normalized().dot(v2.normalized());
			if (rad < -1.0)
				rad = -1.0;
			else if (rad >  1.0)
				rad = 1.0;
			return (inDegree ? acos(rad) * 180.0 / M_PI : acos(rad));
		}


	private:
		std::vector<Eigen::Vector3d> points_;
		std::vector<Eigen::Vector3d> normals_;
		std::vector<int> indices_;
		std::vector<int> shuffledIndices_;
		int sampleSize_;
		int modelSize_;
		int maxSampleChecks_;
		int maxIterations_;
		double samplesRadius_;
		double normalDistanceWeight_;
		double radiusMax_;
		double radiusMin_;
		double threshold_;
		double probability_;

	private:
		/** Base functor all the models that need non linear optimization must
		* define their own one and implement operator() (const Eigen::VectorXdd& x, Eigen::VectorXdd& fvec)
		* or operator() (const Eigen::VectorXdf& x, Eigen::VectorXdf& fvec) dependening on the choosen _double
		*/
		template<typename _Scalar, int NX = Eigen::Dynamic, int NY = Eigen::Dynamic>
		struct Functor
		{
			typedef _Scalar Scalar;
			enum
			{
				InputsAtCompileTime = NX,
				ValuesAtCompileTime = NY
			};

			typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, 1> ValueType;
			typedef Eigen::Matrix<Scalar, InputsAtCompileTime, 1> InputType;
			typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, InputsAtCompileTime> JacobianType;

			/** \brief Empty Constructor. */
			Functor() : mDataPoints_(ValuesAtCompileTime) {}

			/** \brief Constructor
			* \param[in] m_data_points number of data points to evaluate.
			*/
			Functor(int mDataPoints) : mDataPoints_(mDataPoints) {}

			virtual ~Functor() {}

			/** \brief Get the number of values. */
			int values() const { return (mDataPoints_); }

		private:
			const int mDataPoints_;
		};


		/** \brief Functor for the optimization function */
		struct OptimizationFunctor : Functor<double>
		{
			/** Functor constructor
			* \param[in] m_data_points the number of data points to evaluate
			* \param[in] estimator pointer to the estimator object
			* \param[in] distance distance computation function pointer
			*/
			OptimizationFunctor(const std::vector<int>& inliers, const std::vector<Eigen::Vector3d>& points) :
			Functor<double>(inliers.size()), inliers_(inliers), points_(points){}

			/** Cost function to be minimized
			* \param[in] x variables array
			* \param[out] fvec resultant functions evaluations
			* \return 0
			*/
			int operator() (const Eigen::VectorXd &x, Eigen::VectorXd &fvec) const
			{
				Eigen::Vector3d linePt(x[0], x[1], x[2]);
				Eigen::Vector3d lineDir(x[3], x[4], x[5]);

				for (int i = 0; i < values(); ++i)
				{
					// dist = f - r
					Eigen::Vector3d pt(points_[inliers_[i]].x(), points_[inliers_[i]].y(), points_[inliers_[i]].z());

					fvec[i] = pointToLineDistance(pt, linePt, lineDir) - x[6] * x[6];//unpack x[6] to get r
				}
				return 0;
			}
			std::vector<int> inliers_;
			std::vector<Eigen::Vector3d> points_;
		};

		/** \brief Functor for the optimization function */
		struct OptimizationFunctorWithRadius : Functor<double>
		{
			/** Functor constructor
			* \param[in] m_data_points the number of data points to evaluate
			* \param[in] estimator pointer to the estimator object
			* \param[in] distance distance computation function pointer
			*/
			OptimizationFunctorWithRadius(const double& r, const std::vector<int>& inliers, const std::vector<Eigen::Vector3d>& points) :
			Functor<double>(inliers.size()), r_(r), inliers_(inliers), points_(points){}

			/** Cost function to be minimized
			* \param[in] x variables array
			* \param[out] fvec resultant functions evaluations
			* \return 0
			*/
			int operator() (const Eigen::VectorXd &x, Eigen::VectorXd &fvec) const
			{
				Eigen::Vector3d linePt(x[0], x[1], x[2]);
				Eigen::Vector3d lineDir(x[3], x[4], x[5]);

				for (int i = 0; i < values(); ++i)
				{
					// dist = f - r
					Eigen::Vector3d pt(points_[inliers_[i]].x(), points_[inliers_[i]].y(), points_[inliers_[i]].z());

					fvec[i] = pointToLineDistance(pt, linePt, lineDir) - r_;
				}
				return 0;
			}
			double r_;
			std::vector<int> inliers_;
			std::vector<Eigen::Vector3d> points_;
		};
	};

}//namespce Sn3DAlgorithm

#endif //TEST_SN3D_ALGORITHM_CYLINDER_FITTING_H
