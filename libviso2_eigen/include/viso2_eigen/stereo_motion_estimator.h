/**
 * @file stereo_motion_estimator.h
 * @author Huimin Cheng (NUS)
 * @brief Estimate motion based on quad-matched points
 * @version 0.1
 * @date 2018-10-02
 * 
 * @copyright Copyright (c) 2018
 * 
 */

#ifndef STEREO_MOTION_ESTIMATOR_H
#define STEREO_MOTION_ESTIMATOR_H

#include <Eigen/Eigen>
#include <opencv2/core.hpp>

#include <vector>
#include <array>

#include <random>
#include <cmath>

#include <iostream>

#include "viso2_eigen_params.h"

class StereoMotionEstimator {

public:
    
    typedef std::array<int,4> DMatch;
    static const int L1 = 0, L2 = 1, R2 = 2, R1 = 3; // previous left, current left, current right, previous right

    // constructor, takes as inpute a parameter structure
    StereoMotionEstimator() : initialised(false) {}

	void setParam(StereoMotionEstimatorParam::Parameters &param);

    // Column sequence of matches_quad_vec: previous left, current left, current right, previous right
    void pushBackData(
        const std::vector<DMatch> &matches_quad_vec,
        const std::vector<cv::KeyPoint> &keyl1_vec,
        const std::vector<cv::KeyPoint> &keyl2_vec,
        const std::vector<cv::KeyPoint> &keyr2_vec,
        const std::vector<cv::KeyPoint> &keyr1_vec
    );

    
	bool updateMotion();

	std::vector<int> getInlier() { return result.inliers; };
	
	// Transform to obtain 3D point coordinate w.r.t. the current camera frame (change of frame, from the previous frame)
	// Equivalent to an active motion from current frame to previous frame
	Eigen::Affine3d getMotion() { return result.Tr_delta; } 

	// Motion of camera w.r.t. previous camera frame (active motion)
	// The matrix Tr_delta.inverse() can be interpreted as the attitude of current frame w.r.t. previous frame 
	Eigen::Affine3d getCameraMotion() { return result.Tr_delta.inverse(); } 

    double getArea(){ return result.area; }

    // deconstructor
	~StereoMotionEstimator() {}

	friend std::ostream& operator<< (std::ostream &os,StereoMotionEstimator &viso) {
		auto p = viso.getMotion();
		os << p(0,0) << " " << p(0,1) << " "  << p(0,2)  << " "  << p(0,3) << " ";
		os << p(1,0) << " " << p(1,1) << " "  << p(1,2)  << " "  << p(1,3) << " ";
		os << p(2,0) << " " << p(2,1) << " "  << p(2,2)  << " "  << p(2,3);
		return os;
	}

private:
    
    enum calcResult { UPDATED, FAILED, CONVERGED };

    struct Result{
        bool valid;
        std::vector<double> tr_delta; // final TR transform estimation in VISO vector form (rotation, translation)
        Eigen::Affine3d Tr_delta; // TR transform in homogenous matrix form

        std::vector<int> inliers; // index of all inliers
        double area;
    };

    void                 computeObservations(const std::vector<int> &active, bool inlier_mode = false);
    
    calcResult           updateParameters(std::vector<int> &active, std::vector<double> &tr, double step_size, double eps);
	void                 computeResidualsAndJacobian(const std::vector<double> &tr, const std::vector<int> &active, bool inlier_mode = false);

	std::vector<double> estimateMotion();

    double good_point_threshold;
	std::vector<int>    getInlier(std::vector<double> &tr);

    std::vector<int>    getRandomSample (int N,int num);

	Eigen::Affine3d transformationVectorToMatrix(const std::vector<double> &tr);

    bool initialised;
    StereoMotionEstimatorParam::Parameters param;

    const std::vector<DMatch> *matches_quad_vec;
    const std::vector<cv::KeyPoint> *keyl1_vec, *keyl2_vec, *keyr1_vec, *keyr2_vec;


    std::vector<double> X,Y,Z,J; // 3D points and Jacobian
    std::vector<double> GoodPointThreshold;
    std::vector<double> p_observe;  // observed 2d points
    std::vector<double> p_predict;  // predicted 2d points
    std::vector<double> p_residual; // residuals (p_residual=p_observe-p_predict)

    Result result;

    // std::vector<int> inliers;

	// std::vector<double> tr_delta_vec;
	// Eigen::Matrix<double,4,4> Tr_delta; // This stores the change of frame transformation from previous frame to current frame
};



#endif // STEREO_MOTION_ESTIMATOR_H