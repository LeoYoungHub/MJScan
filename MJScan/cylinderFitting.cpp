#include <iostream>
#include "cylinderFitting.h"
#include "unsupported/Eigen/NumericalDiff"
#include "unsupported/Eigen/NonLinearOptimization"


namespace Sn3DAlgorithm	
{
	bool CylinderFitting::cylinder_fitting(std::vector<double>& para, double radius)
	{
		if (points_.size() != normals_.size()) 
		{
			std::cout << "points size != normals size" << std::endl;
			return false;
		}

		Eigen::VectorXd coefficients;
		std::vector<int> inliers;
		if (radius > 0)
		{
			Eigen::Vector3d meanCoord = Eigen::Vector3d::Zero();
			for (int v = 0; v < points_.size(); ++v) meanCoord += points_[v];
			meanCoord /= points_.size();

			coefficients.resize(6);
			coefficients[0] =  meanCoord[0]; coefficients[1] =  meanCoord[1]; coefficients[2] =  meanCoord[2];
			coefficients[3] = 1. / sqrt(3.); coefficients[4] = 1. / sqrt(3.); coefficients[5] = 1. / sqrt(3.);
			inliers = get_indices();
			non_liner_least_squares_model_fitting_with_radius(radius, inliers,coefficients);
		}
		else
		{
			if (normals_.empty())
			{
				return false;
			}
			coefficients.resize(7);
			if (!ransac_Lmeds_cylinder_fitting(inliers, coefficients))
			{
				std::cout << "ransac Lmeds model faild!" << std::endl;
				return false;
			}
			non_liner_least_squares_model_fitting(inliers, coefficients);

			select_within_distance(coefficients, inliers, 0.1*coefficients[6]);

			non_liner_least_squares_model_fitting(inliers, coefficients);
		}
		refine_para(get_indices(), coefficients, para, radius);
		return true;
	}

	bool CylinderFitting::ransac_Lmeds_cylinder_fitting(std::vector<int>& inliers, Eigen::VectorXd& coefficients)
	{
		int iterations = 0;

		double dBestPenalty = std::numeric_limits<double>::max();

		std::vector<int> bestModel;
		std::vector<int> selection;
		std::vector<int> modelSelection;
		Eigen::VectorXd modelCoefficients;
		std::vector<double> distances;

		int nInliersCount = 0;

		unsigned skippedCount = 0;

		// supress infinite loops by just allowing 10 x maximum allowed iterations for invalid model parameters!
		const unsigned maxSkip = maxIterations_ * 10;

		// Iterate
		while (iterations < maxIterations_ && skippedCount < maxSkip)
		{
			// Get X samples which satisfy the model criteria
			get_samples(iterations, selection);

			if (selection.empty())
			{
				std::cout << "selection is empty!" << std::endl;
				break;
			}

			// Search for inliers in the point cloud for the current plane model M
			if (!compute_model_coefficients(selection, modelCoefficients))
			{
				//iterations_++;
				++skippedCount;
				continue;
			}

			double dCurPenalty = 0;
			// d_cur_penalty = sum (min (dist, threshold))

			// Iterate through the 3d points and calculate the distances from them to the model
			get_distances_to_model(modelCoefficients, distances);

			// No distances? The model must not respect the user given constraints
			if (distances.empty())
			{
				//iterations_++;
				++skippedCount;
				continue;
			}

			std::sort(distances.begin(), distances.end());
			// d_cur_penalty = median (distances)
			size_t mid = get_indices().size() / 2;
			if (mid >= distances.size())
			{
				//iterations_++;
				++skippedCount;
				continue;
			}

			// Do we have a "middle" point or should we "estimate" one ?
			if (get_indices().size() % 2 == 0)
				dCurPenalty = (sqrt(distances[mid - 1]) + sqrt(distances[mid])) / 2;
			else
				dCurPenalty = sqrt(distances[mid]);

			// Better match ?
			if (dCurPenalty < dBestPenalty)
			{
				dBestPenalty = dCurPenalty;

				// Save the current model/coefficients selection as being the best so far
				modelSelection = selection;
				coefficients = modelCoefficients;
			}

			++iterations;
		}

		if (modelSelection.empty())
		{
			std::cout << "LeastMedianSquares::computeModel unable to find a solution!" << std::endl;
			return false;
		}

		// Classify the data points into inliers and outliers
		// Sigma = 1.4826 * (1 + 5 / (n-d)) * sqrt (M)
		// @note: See "Robust Regression Methods for Computer Vision: A Review"
		//double sigma = 1.4826 * (1 + 5 / (get_indices().size() - modelSelection.size())) * sqrt(dBestPenalty);
		//double threshold = 2.5 * sigma;

		// Iterate through the 3d points and calculate the distances from them to the model again
		get_distances_to_model(coefficients, distances);
		// No distances? The model must not respect the user given constraints
		if (distances.empty())
		{
			std::cout << "LeastMedianSquares model found failed to verify against the given constraints!" << std::endl;
			return (false);
		}

		const std::vector<int> &indices = get_indices();

		if (distances.size() != indices.size())
		{
			std::cout << "LeastMedianSquares model estimated distances(%lu) differs than the normal of indices" << std::endl;;
			return false;
		}

		inliers.resize(distances.size());
		// Get the inliers for the best model found
		nInliersCount = 0;
		for (size_t i = 0; i < distances.size(); ++i)
		if (distances[i] <= get_threshold())//get_threshold()
			inliers[nInliersCount++] = indices[i];

		// Resize the inliers vector
		inliers.resize(nInliersCount);
	}

	bool CylinderFitting::ransac_cylinder_fitting(std::vector<int>& inliers, Eigen::VectorXd& coefficients, const double& threshold)
	{
		// Warn and exit if no threshold was set
		if (threshold == std::numeric_limits<double>::max())
		{
			std::cout << "RandomSampleConsensus::computeModel No threshold set!" << std::endl;
			return false;
		}

		int iterations = 0;
		int nBestInliersCount = -INT_MAX;
		double k = 1.0;

		std::vector<int> selection;
		std::vector<int> bestselection;
		Eigen::VectorXd modelCoefficients;

		double logProbability = log(1.0 - probability_);
		double oneOverIndices = 1.0 / static_cast<double> (get_indices().size());

		int nInliersCount = 0;
		unsigned skippedCount = 0;
		// supress infinite loops by just allowing 10 x maximum allowed iterations for invalid model parameters!
		const unsigned maxSkip = maxIterations_ * 10;

		// Iterate
		while (iterations < k && skippedCount < maxSkip)
		{
			// Get X samples which satisfy the model criteria
			get_samples(iterations, selection);

			if (selection.empty())
			{
				std::cout<< "RandomSampleConsensus::computeModel] No samples could be selected!"<<std::endl;
				break;
			}

			// Search for inliers in the point cloud for the current plane model M
			if (!compute_model_coefficients(selection, modelCoefficients))
			{
				//++iterations_;
				++skippedCount;
				continue;
			}

			// Select the inliers that are within threshold_ from the model
			//sac_model_->selectWithinDistance (model_coefficients, threshold_, inliers);
			//if (inliers.empty () && k > 1.0)
			//  continue;

			nInliersCount = count_within_distance(modelCoefficients, threshold);

			// Better match ?
			if (nInliersCount > nBestInliersCount)
			{
				nBestInliersCount = nInliersCount;

				// Save the current model/inlier/coefficients selection as being the best so far
				bestselection = selection;
				coefficients = modelCoefficients;

				// Compute the k parameter (k=log(z)/log(1-w^n))
				double w = static_cast<double> (nBestInliersCount)* oneOverIndices;
				double p_no_outliers = 1.0 - pow(w, static_cast<double> (selection.size()));
				p_no_outliers = (std::max) (std::numeric_limits<double>::epsilon(), p_no_outliers);       // Avoid division by -Inf
				p_no_outliers = (std::min) (1.0 - std::numeric_limits<double>::epsilon(), p_no_outliers);   // Avoid division by 0.
				k = logProbability / log(p_no_outliers);
			}

			++iterations;
			if (iterations > maxIterations_)
			{
				break;
			}
		}

		if (bestselection.empty())
		{
			std::cout << "Best selection is empty!" << std::endl;
			inliers.clear();
			return false;
		}

		// Get the set of inliers that correspond to the best model found so far
		select_within_distance(coefficients, inliers, threshold);
		return true;
	}

	void CylinderFitting::get_samples(int &iterations, std::vector<int> &samples)
	{
		if (indices_.size() < get_sample_size())
		{
			std::cout << "Can not select 2 unique points!" << std::endl;
			samples.clear();
			iterations = INT_MAX - 1;
			return;
		}
		samples.resize(get_sample_size());
		for (unsigned int iter = 0; iter < maxSampleChecks_; ++iter)
		{
			if (samplesRadius_ < std::numeric_limits<double>::epsilon())
			{
				int sampleSize = samples.size();
				int indexSize = shuffledIndices_.size();
				for (unsigned int i = 0; i < sampleSize; ++i)
				{
					std::swap(shuffledIndices_[i], shuffledIndices_[i + (rand() % (indexSize - i))]);
					std::copy(shuffledIndices_.begin(), shuffledIndices_.begin() + sampleSize, samples.begin());
				}
			}
			else
			{
				//TODO
			}
		}
	}

	bool CylinderFitting::compute_model_coefficients(const std::vector<int> &samples, Eigen::VectorXd &coefficients)
	{
		// Need 2 samples
		if (samples.size() != 2)
		{
			std::cout << " Need 2 samples!" << std::endl;
			return false;
		}

		if (normals_.empty())
		{
			std::cout << " Need normals!" << std::endl;
			return false;
		}

		if (fabs(points_[samples[0]].x() - points_[samples[1]].x()) <= std::numeric_limits<double>::epsilon() &&
			fabs(points_[samples[0]].y() - points_[samples[1]].y()) <= std::numeric_limits<double>::epsilon() &&
			fabs(points_[samples[0]].z() - points_[samples[1]].z()) <= std::numeric_limits<double>::epsilon())
		{
			return false;
		}

		const Eigen::Vector3d& p1 = points_[samples[0]];
		const Eigen::Vector3d& p2 = points_[samples[1]];

		const Eigen::Vector3d& n1 = normals_[samples[0]];
		const Eigen::Vector3d& n2 = normals_[samples[1]];

		Eigen::Vector3d w = n1 + p1 - p2;

		double a = n1.dot(n1);
		double b = n1.dot(n2);
		double c = n2.dot(n2);
		double d = n1.dot(w);
		double e = n2.dot(w);
		double denominator = a*c - b*b;
		double sc, tc;
		// Compute the line parameters of the two closest points
		if (denominator < 1e-8)// The lines are almost parallel
		{
			sc = 0.0f;
			tc = (b > c ? d / b : e / c);// Use the largest denominator
		}
		else
		{
			sc = (b*e - c*d) / denominator;
			tc = (a*e - b*d) / denominator;
		}

		// point_on_axis, axis_direction
		Eigen::Vector3d linePt = p1 + n1 + sc * n1;
		Eigen::Vector3d lineDir = p2 + tc * n2 - linePt;
		lineDir.normalize();

		coefficients.resize(7);
		//linePt
		coefficients[0] = linePt[0];
		coefficients[1] = linePt[1];
		coefficients[2] = linePt[2];
		//lineDir
		coefficients[3] = lineDir[0];
		coefficients[4] = lineDir[1];
		coefficients[5] = lineDir[2];
		//cylinder radius
		coefficients[6] = pointToLineDistance(p1, linePt, lineDir);

		if (coefficients[6] > radiusMax_ || coefficients[6] < radiusMin_)
			return false;

		return true;
	}

	void CylinderFitting::get_distances_to_model(const Eigen::VectorXd& modelCoefficients, std::vector<double>& distances)
	{
		distances.resize(get_indices().size());

		Eigen::Vector3d linePt(modelCoefficients[0], modelCoefficients[1], modelCoefficients[2]);
		Eigen::Vector3d lineDir(modelCoefficients[3], modelCoefficients[4], modelCoefficients[5]);
		double ptdotdir = linePt.dot(lineDir);
		double dirdotdir = 1.0f / lineDir.dot(lineDir);
		// Iterate through the 3d points and calculate the distances from them to the sphere
		const std::vector<int>& indices = get_indices();
		for (int i = 0; i < indices.size(); ++i)
		{
			// Aproximate the distance from the point to the cylinder as the difference between
			// dist(point,cylinder_axis) and cylinder radius
			// @note need to revise this.
			const Eigen::Vector3d& pt = points_[indices[i]];
			const Eigen::Vector3d& n = normals_[indices[i]];

			double dEuclid = fabs(pointToLineDistance(pt, linePt, lineDir) - modelCoefficients[6]);

			// Calculate the point's projection on the cylinder axis
			double k = (pt.dot(lineDir) - ptdotdir) * dirdotdir;
			Eigen::Vector3d pt_proj = linePt + k * lineDir;
			Eigen::Vector3d dir = pt - pt_proj;
			dir.normalize();

			// Calculate the angular distance between the point normal and the (dir=pt_proj->pt) vector
			double dNormal = fabs(getAngle3D(n, dir));
			dNormal = std::min(dNormal, M_PI - dNormal);

			distances[i] = fabs(normalDistanceWeight_ * dNormal + (1 - normalDistanceWeight_) * dEuclid);
		}
	}

	void CylinderFitting::select_within_distance(const Eigen::VectorXd &coefficients, std::vector<int> &inliers, const double& threshold)
	{
		int nrP = 0;
		inliers.resize(get_indices().size());

		Eigen::Vector3d linePt(coefficients[0], coefficients[1], coefficients[2]);
		Eigen::Vector3d lineDir(coefficients[3], coefficients[4], coefficients[5]);
		double ptdotdir = linePt.dot(lineDir);
		double dirdotdir = 1.0f / lineDir.dot(lineDir);
		// Iterate through the 3d points and calculate the distances from them to the sphere
		const std::vector<int>& indices = get_indices();

		for (int i = 0; i < indices.size(); ++i)
		{
			// Aproximate the distance from the point to the cylinder as the difference between
			// dist(point,cylinder_axis) and cylinder radius
			const Eigen::Vector3d& pt = points_[indices[i]];
			const Eigen::Vector3d& n = normals_[indices[i]];
			double dEuclid = fabs(pointToLineDistance(pt, linePt, lineDir) - coefficients[6]);

			// Calculate the point's projection on the cylinder axis
			double k = (pt.dot(lineDir) - ptdotdir) * dirdotdir;
			Eigen::Vector3d ptProj = linePt + k * lineDir;
			Eigen::Vector3d dir = pt - ptProj;
			dir.normalize();

			// Calculate the angular distance between the point normal and the (dir=pt_proj->pt) vector
			double dNormal = fabs(getAngle3D(n, dir));
			dNormal = std::min(dNormal, M_PI - dNormal);

			double distance = fabs(normalDistanceWeight_ * dNormal + (1 - normalDistanceWeight_) * dEuclid);
			if (dEuclid < threshold)
			{
				// Returns the indices of the points whose distances are smaller than the threshold
				inliers[nrP] = indices[i];
				++nrP;
			}
		}
		inliers.resize(nrP);
	}

	int CylinderFitting::count_within_distance(const Eigen::VectorXd &coefficients, const double& threshold)
	{
		int nrP = 0;

		Eigen::Vector3d linePt(coefficients[0], coefficients[1], coefficients[2]);
		Eigen::Vector3d lineDir(coefficients[3], coefficients[4], coefficients[5]);
		double ptdotdir = linePt.dot(lineDir);
		double dirdotdir = 1.0f / lineDir.dot(lineDir);
		// Iterate through the 3d points and calculate the distances from them to the sphere
		const std::vector<int>& indices = get_indices();
		for (int i = 0; i < indices.size(); ++i)
		{
			// Aproximate the distance from the point to the cylinder as the difference between
			// dist(point,cylinder_axis) and cylinder radius
			const Eigen::Vector3d& pt = points_[indices[i]];
			const Eigen::Vector3d& n = normals_[indices[i]];
			double dEuclid = fabs(pointToLineDistance(pt, linePt, lineDir) - coefficients[6]);

			// Calculate the point's projection on the cylinder axis
			double k = (pt.dot(lineDir) - ptdotdir) * dirdotdir;
			Eigen::Vector3d ptProj = linePt + k * lineDir;
			Eigen::Vector3d dir = pt - ptProj;
			dir.normalize();

			// Calculate the angular distance between the point normal and the (dir=pt_proj->pt) vector
			double dNormal = fabs(getAngle3D(n, dir));
			dNormal = std::min(dNormal, M_PI - dNormal);

			double distance = fabs(normalDistanceWeight_ * dNormal + (1 - normalDistanceWeight_) * dEuclid);
			if (distance < threshold) nrP++;
		}
		return nrP;
	}

	void CylinderFitting::non_liner_least_squares_model_fitting(const std::vector<int>& inliers, Eigen::VectorXd &coefficients)
	{
		// Needs a set of valid model coefficients
		if (coefficients.size() != 7)
		{
			std::cout << "Invalid number of model coefficients given!" << std::endl;
			return;
		}

		if (points_.empty())
		{
			std::cout << "Inliers vector empty! Returning the same coefficients!" << std::endl;
			return;
		}

		//Parameterize r as r^2 so that it can't be negative. 
		coefficients[6] = sqrt(coefficients[6]);
		OptimizationFunctor functor(inliers, points_);
		Eigen::NumericalDiff<OptimizationFunctor > numDiff(functor);
		Eigen::LevenbergMarquardt<Eigen::NumericalDiff<OptimizationFunctor>, double> lm(numDiff);
		int info = lm.minimize(coefficients);

		Eigen::Vector3d lineDir(coefficients[3], coefficients[4], coefficients[5]);
		lineDir.normalize();
		coefficients[3] = lineDir[0];
		coefficients[4] = lineDir[1];
		coefficients[5] = lineDir[2];

		//recover r from optimizedCoefficients[6]
		coefficients[6] = pow(coefficients[6], 2);
	}

	void CylinderFitting::non_liner_least_squares_model_fitting_with_radius(const double& radius, std::vector<int>& inliers, Eigen::VectorXd &coefficients)
	{
		// Needs a set of valid model coefficients
		if (coefficients.size() != 6)
		{
			std::cout << "Invalid number of model coefficients given!" << std::endl;
			return;
		}

		if (points_.empty())
		{
			std::cout << "Inliers vector empty! Returning the same coefficients!" << std::endl;
			return;
		}
		//Parameterize r as r^2 so that it can't be negative. 
		OptimizationFunctorWithRadius functor(radius, inliers, points_);
		Eigen::NumericalDiff<OptimizationFunctorWithRadius > numDiff(functor);
		Eigen::LevenbergMarquardt<Eigen::NumericalDiff<OptimizationFunctorWithRadius>, double> lm(numDiff);
		int info = lm.minimize(coefficients);
		Eigen::Vector3d lineDir(coefficients[3], coefficients[4], coefficients[5]);
		lineDir.normalize();
		coefficients[3] = lineDir[0];
		coefficients[4] = lineDir[1];
		coefficients[5] = lineDir[2];
	}

	void CylinderFitting::refine_para(const std::vector<int>& inliers, const Eigen::VectorXd& coefficients, std::vector<double>& para, const double radius)
	{
		//获得圆柱的端点以及高度
		Eigen::Vector3d linePt(coefficients[0], coefficients[1], coefficients[2]);
		Eigen::Vector3d lineDir(coefficients[3], coefficients[4], coefficients[5]);
		double ptdotdir = linePt.dot(lineDir);
		double dirdotdir = 1.0f / lineDir.dot(lineDir);

		double maxT = -std::numeric_limits<double>::max();
		double minT =  std::numeric_limits<double>::max();
		for (int i = 0; i < inliers.size(); ++i)
		{
			Eigen::Vector3d pt = points_[inliers[i]];
			// Calculate the point's projection on the cylinder axis
			double t = (pt.dot(lineDir) - ptdotdir) * dirdotdir;

			if (t > maxT) maxT = t;
			if (t < minT) minT = t;

		}
		Eigen::Vector3d basePt = linePt + minT * lineDir;
		Eigen::Vector3d topPt = linePt + maxT * lineDir;
		if (topPt.y() < basePt.y()) std::swap(topPt, basePt);
	
		Eigen::Vector3d dir = topPt - basePt;
		double height = dir.norm();
		dir /= height;

		para.resize(11);
		para[0] = topPt[0]; para[1] = topPt[1]; para[2] = topPt[2];
		para[3] = basePt[0]; para[4] = basePt[1]; para[5] = basePt[2];
		para[6] = dir[0]; para[7] = dir[1]; para[8] = dir[2];
		para[9] = height; 
		if (radius > 0) para[10] = radius;
		if (radius < 0) para[10] = coefficients[6];
	}

	double CylinderFitting::get_the_fitting_error(const Eigen::VectorXd &coefficients)
	{
		double fitError = 0.;
		Eigen::Vector3d linePt(coefficients[0], coefficients[1], coefficients[2]);
		Eigen::Vector3d lineDir(coefficients[3], coefficients[4], coefficients[5]);
		for (int i = 0; i < points_.size(); ++i)
		{
			fitError += pow((pointToLineDistance(points_[i], linePt, lineDir) - coefficients[6]),2);
		
		}
		fitError /= (double)points_.size();
		return fitError;
	}
}//namespace Sn3DAlgorithm