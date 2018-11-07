/**
 * @file stereo_motion_estimator.cpp
 * @author Huimin Cheng (NUS)
 * @brief Estimate motion based on quad-matched points
 * @version 0.1
 * @date 2018-10-02
 * 
 * @copyright Copyright (c) 2018
 * 
 */


#include "stereo_motion_estimator.h"

#include <cmath>
#include "opencv2/imgproc/imgproc.hpp"

/////////////////////////////////////////////////////////////////////////////////
//// Implementation of Public Member Functions
/////////////////////////////////////////////////////////////////////////////////


void StereoMotionEstimator::setParam(StereoMotionEstimatorParam::Parameters &param) { 

    assert (param.calib.cu != 0 && param.calib.cv != 0);

    this->param = param; 
    initialised = true;
    std::cout << std::endl << "StereoMotionEstimator parameters loaded!" << std::endl;
    std::cout << "- image size " << param.image_width << "x" << param.image_height << std::endl;
    std::cout << "- RANSAC iterations " << param.ransac_iters << std::endl;
    std::cout << "- baseline=" << this->param.calib.baseline << ", f=" << this->param.calib.f << ", cu=" << this->param.calib.cu << 
        ", cv=" << this->param.calib.cv << std::endl;
    std::cout << "==============================================================" << std::endl;
}

void StereoMotionEstimator::pushBackData(
                                            const std::vector<DMatch> &matches_quad_vec,
                                            const std::vector<cv::KeyPoint> &keyl1_vec,
                                            const std::vector<cv::KeyPoint> &keyl2_vec,
                                            const std::vector<cv::KeyPoint> &keyr2_vec,
                                            const std::vector<cv::KeyPoint> &keyr1_vec
                                            ){
    this->matches_quad_vec = &matches_quad_vec;
    this->keyl1_vec = &keyl1_vec;
    this->keyl2_vec = &keyl2_vec;
    this->keyr1_vec = &keyr1_vec;
    this->keyr2_vec = &keyr2_vec;

	// Sanity checks
	// The matched index amount should be smaller / equal to available points
	assert(matches_quad_vec.size() <= std::min( keyr1_vec.size() ,  keyr2_vec.size() ));
	
}

bool StereoMotionEstimator::updateMotion () {

	result.valid = false;
	// estimate motion
	std::vector<double> tr_delta = estimateMotion();
	//   tr_delta_vec.clear();

	// on failure
	if (tr_delta.size()!=6)
	return false;

	// set transformation matrix (previous to current frame)
	result.Tr_delta = transformationVectorToMatrix(tr_delta);
	//   tr_delta_vec = tr_delta;
	// result.tr_delta_rot = Eigen::Quaternion<double>(result.Tr_delta.block<3,3>(0,0));
	// result.tr_delta_trans = result.Tr_delta.block<3,1>(0,3);

	// success
	result.valid = true;
	return true;
}

inline double getContourArea(const std::vector<cv::Point2f> &pts){

	if (pts.size()<3)
		return 0;

	// Calculating the convex Hull
	// std::vector<cv::Point2f> hull;
	return contourArea(pts);
	
	// double area = 0;
	// int N = hull.size();

	// for (int i = 0 ; i < N - 1 ; i++){
	// 	area += hull[i].x*hull[i+1].y - hull[i].y*hull[i+1].x;
	// }

	// area += hull[N-1].x*hull[0].y - hull[N-1].y*hull[0].x;

	// area/=2.0;
	// return area;
}


/////////////////////////////////////////////////////////////////////////////////
//// Implementation of Private Member Functions
/////////////////////////////////////////////////////////////////////////////////

std::vector<double> StereoMotionEstimator::estimateMotion()
{

	this->result.tr_delta.clear();
	this->result.inliers.clear();
	this->result.area = 0;

	// compute minimum distance for RANSAC samples
	float width_max = 0, height_max = 0;
	float width_min = 1e5, height_min = 1e5;

	// Sanity check for number of matches
	const int N = matches_quad_vec->size();
    if (N < 10)
    {
        std::cerr << "Total poll of matches is too small < " << N << ", aborting viso: " << N << std::endl;
        return std::vector<double>();
    }


    // with reference to previous left frame
	for ( const auto match : (*matches_quad_vec) )
	{
        const cv::KeyPoint key = (*keyl1_vec)[ match[L1] ];

		if ( key.pt.x > width_max)  width_max = key.pt.x;
		if ( key.pt.x < width_min)  width_min = key.pt.x;

		if ( key.pt.y > height_max) height_max = key.pt.y;
		if ( key.pt.y < height_min) height_min = key.pt.y;
	}

	// Defined the min-dist between any random 3 matches
	float min_dist = std::max(width_max-width_min, height_max-height_min) / 5.f;	// default divided by 3.0

	// std::cout << "min_dist=" << min_dist << std::endl;

    if ( min_dist < std::min(param.image_height, param.image_width) / 20.f  )
    {
        std::cerr << "min_dist is too small (< 0.05*image_dimensions), aborting viso: " << min_dist << std::endl;
        return std::vector<double>();
    }

    min_dist = min_dist*min_dist;

	// clear vectors
	std::vector<int> inliers;
	double area = 0;
    X.resize(N);
    Y.resize(N);
    Z.resize(N);
    J.resize(4 * N * 6); // yx: save Jacobian matrix for each point (6*4: 6 functions and 4 unknowns)

    p_predict.resize(4 * N);
    p_observe.resize(4 * N);
    p_residual.resize(4 * N);

    double &_cu = param.calib.cu;
    double &_cv = param.calib.cv;
    double &_f = param.calib.f;

	// project matches of previous image into 3d

	for (size_t i = 0; i < (*matches_quad_vec).size() ; i++ )
	{
		const cv::KeyPoint &keyl1 = (*keyl1_vec)[ (*matches_quad_vec)[i][L1] ];
		const cv::KeyPoint &keyr1 = (*keyr1_vec)[ (*matches_quad_vec)[i][R1] ];

		if ( keyl1.pt.x - keyr1.pt.x < 0.0 )
		{
			std::cerr << "Warning: Flipped match at " << i << std::endl;
		}

		double d = std::max( keyl1.pt.x - keyr1.pt.x, 0.0001f);			// d = xl - xr
		X[i] = (keyl1.pt.x - _cu) * param.calib.baseline / d;				// X = (u1p - calib.cu)*baseline/d
		Y[i] = (keyl1.pt.y - _cv) * param.calib.baseline / d;				// Y = (v1p - calib.cv)*baseline/d
		Z[i] = _f * param.calib.baseline / d;									// Z = f*baseline/d

	}



	// loop variables
	std::vector<double> tr_delta;				// yx: ransac: bestfit
	std::vector<double> tr_delta_curr(6,0);

	std::vector<int> best_active;

	// // initial RANSAC estimate
	int max_iteration = N*N*N / 5;
	for (int k = 0; k < param.ransac_iters && k <  max_iteration ; k++)
	{
        // std::cout << "Begin k=" << k << std::endl;
        // active stores the current active rows in matches_quad_vec
		std::vector<int> active; 


        // Generate 3 points that satisfied the min_dist, in the previous left image
		for (int selection_iter = 0; ; selection_iter++)
		{
			active = getRandomSample(N, 3);

            int idx0 = (*matches_quad_vec)[ active[0] ][L1];
            int idx1 = (*matches_quad_vec)[ active[1] ][L1];
            int idx2 = (*matches_quad_vec)[ active[2] ][L1];

            const cv::Point2f pt0 = (*keyl1_vec)[idx0].pt;
            const cv::Point2f pt1 = (*keyl1_vec)[idx1].pt;
            const cv::Point2f pt2 = (*keyl1_vec)[idx2].pt;

			double x0 = (pt0.x - pt1.x)*(pt0.x - pt1.x);
			double y0 = (pt0.y - pt1.y)*(pt0.y - pt1.y);
			double x1 = (pt1.x - pt2.x)*(pt1.x - pt2.x);
			double y1 = (pt1.y - pt2.y)*(pt1.y - pt2.y);
			double x2 = (pt2.x - pt0.x)*(pt2.x - pt0.x);
			double y2 = (pt2.y - pt0.y)*(pt2.y - pt0.y);
			double d0 = x0 + y0; double d1 = x1 + y1; double d2 = x2 + y2;

			if (d0 >= min_dist && d1 >= min_dist && d2 >= min_dist)
				break;

			if (selection_iter >= 50)
            {
                std::cerr << "Finding RANSAC 3 matches > min_dist not possible..." << std::endl;
				break;
                // return std::vector<double>();
            }
		}

        assert (active.size() == 3);
        // std::cout << "Random 3 Generated: " << active[0] << ", " << active[1] << ", " << active[2] << std::endl;
		
        // Initialise current TR estimation
        tr_delta_curr.clear();
        tr_delta_curr.resize(6,0);			// yx: ransac: maybemodel (for current sample)

		// minimize reprojection errors
        computeObservations(active);
		

        // std::cout << "Right Before updateParameters Routine" << std::endl;

        // std::cout << "p_observe: ";
        // for (int i=0;i<4;i++)
        //     std::cout << p_observe[i] << ' ';
        // std::cout << std::endl;

		StereoMotionEstimator::calcResult result = UPDATED;
		for ( int iter = 1; result == UPDATED ; iter++ )
		{
			result = updateParameters(active, tr_delta_curr, 1, 1e-6);
			if (result == CONVERGED)
            {
                // std::cout << "RANSAC " << k << ": TR Converged after " << iter << " iterations. " << std::endl;
                break;
            }	

			if (iter >= 20) // hm: this happens very frequently
				break;
		}

        // std::cout << "Right After updateParameters Routine" << std::endl;

        // std::cout << "p_predict: ";
        // for (int i=0;i<4;i++)
        //     std::cout << p_predict[i] << ' ';
        // std::cout << std::endl;

        // std::cout << "p_residual: ";
        // for (int i=0;i<12;i++)
        //     std::cout << p_residual[i] << ' ';
        // std::cout << std::endl;

        // std::cout << " tr_delta_curr: " ;


        // if ( result == UPDATED)
        // {
        //     // std::cerr<< "RANSAC " << k <<  ": updateParameters() EXCEED 20 iterations." << std::endl;
        //     continue;
        // }

        if (result == FAILED)
            continue;

		

		// Update best inlier buffer if we have more inliers
        std::vector<int> inliers_curr = getInlier(tr_delta_curr);

		// calculate area obtained by the current inliers
		std::vector<cv::Point2f> pts;
		for (auto idx : inliers_curr){
			pts.push_back( (*keyl2_vec) [ (*matches_quad_vec)[idx][L2] ].pt );
		}

		double area_curr = std::sqrt(getContourArea(pts));

        if (inliers_curr.size()*area_curr > inliers.size()*area)
        {
            inliers = inliers_curr;
            tr_delta = tr_delta_curr;
			best_active = active;
			area = area_curr;
        }

        // std::cout << "inlier: " << inliers_curr.size() << " out of " << N << std::endl;

	// 	// probility of observing an inlier
	// 	/*
	// 	double Pin = double(inliers.size()) / double(N);
	// 	if (Pin > Pbest) Pbest = Pin;
	// 	N_ransac2 = log(1 - p) / log(1 - pow(Pbest, 3));

	// 	// N_ransac = 200 means, Pbest > 0.28
	// 	if (N_ransac2 < N_ransac &&  k < N_ransac2)
	// 	{
	// 		N_ransac = N_ransac2;
	// 	}
	// 	*/

    // std::cout << "End k=" << k << std::endl;
	}

    std::cout << "Best inlier: " << inliers.size() << " out of " << N << std::endl;

	// Sanity Check
	if ( inliers.size() / (double)N < param.inlier_ratio_min )
	{
		std::cout << "ERROR: Inlier % too small! Return false." << std::endl;
		if (tr_delta.size() > 0){
			this->result.tr_delta = tr_delta;
			this->result.inliers = getInlier(tr_delta);
		}else{
			this->result.tr_delta = std::vector<double>(6);
			this->result.inliers = std::vector<int>();
		}
		
		return std::vector<double>();
	}

	std::cout << "Pre-Refinement TR vector: ";
	for (int i=0;i<6;i++)
		std::cout << tr_delta[i] << ", ";
	std::cout << std::endl;
	
	assert (tr_delta.size() == 6);
	// final optimization (refinement)
	int iter = 0;
	StereoMotionEstimator::calcResult result = UPDATED;
	computeObservations(inliers);
	while (result == UPDATED)
	{
		result = updateParameters(inliers, tr_delta, 1, 1e-8);
		if (result == CONVERGED)
			break;
		if (iter++ > 20)
		{
			std::cerr << "Final Refinement step exceeds 20!" << std::endl;
			break;
		}
			
	}

	if (result == FAILED)
	{
		std::cerr << "WARNING refinement step -> updateParameters FAILED" << std::endl;
		return std::vector<double>();
	}

	// not converged
	if (result == UPDATED)
	{
		std::cerr << "WARNING refinement step -> updateParameters NOT converged" << std::endl;
		return std::vector<double>();
	}

	std::cout << "Post-Refinement TR vector: ";
	for (int i=0;i<6;i++)
		std::cout << tr_delta[i] << ", ";
	std::cout << std::endl;


	this->result.tr_delta = tr_delta;
	this->result.inliers = getInlier(tr_delta);
	this->result.area = area*area;
	
	std::cout << "inliers.size()= " << inliers.size() << " , this->result.inliers.size()= " << this->result.inliers.size() << std::endl;
	std::cout << "this->result.area=" << this->result.area << std::endl;
	assert( this->result.inliers.size() < 10 || this->result.inliers.size() >= inliers.size()*0.7);
	
	return tr_delta;
}

void StereoMotionEstimator::computeObservations(const std::vector<int> &active, bool inlier_mode ) {


    for (size_t i=0 ; i < active.size() || (inlier_mode && i < matches_quad_vec->size() ) ; i++)
    {
        int idx_l2 = (*matches_quad_vec)[ (inlier_mode ? i : active[i]) ][L2];
        int idx_r2 = (*matches_quad_vec)[ (inlier_mode ? i : active[i]) ][R2];
        p_observe[4*i+0] = (*keyl2_vec)[ idx_l2 ].pt.x;
        p_observe[4*i+1] = (*keyl2_vec)[ idx_l2 ].pt.y;
        p_observe[4*i+2] = (*keyr2_vec)[ idx_r2 ].pt.x;
        p_observe[4*i+3] = (*keyr2_vec)[ idx_r2 ].pt.y;
    }
//   // set all observations
//   for (int32_t i=0; i<(int32_t)active.size(); i++) {
//     p_observe[4*i+0] = p_matched[active[i]].u1c; // u1
//     p_observe[4*i+1] = p_matched[active[i]].v1c; // v1
//     p_observe[4*i+2] = p_matched[active[i]].u2c; // u2
//     p_observe[4*i+3] = p_matched[active[i]].v2c; // v2
//   }
}

StereoMotionEstimator::calcResult StereoMotionEstimator::updateParameters(std::vector<int> &active, std::vector<double> &tr, double step_size, double eps) {

	// extract observations and compute predictions
	computeResidualsAndJacobian(tr, active);

	// init
    Eigen::Matrix<double,6,6> A;
    Eigen::Matrix<double,6,1> B;

	// fill matrices A and B
	// JT*J = A
	for (int m = 0; m < 6; m++)
	{
		for (int n = 0; n < 6; n++)
		{
			double a = 0;
			for (size_t i = 0; i < 4 * active.size(); i++)
			{
				a += J[i * 6 + m] * J[i * 6 + n];
			}
			A(m,n) = a;
		}
		double b = 0;
		for (size_t i = 0; i < 4 * active.size(); i++)
		{
			b += J[i * 6 + m] * (p_residual[i]);
		}
		B(m,0) = b;
	}
	//double beta = A.det;
	// perform elimination: solve Ax=B

    Eigen::Matrix<double,6,1> x;

	// Other possibilities: https://eigen.tuxfamily.org/dox/group__TutorialLinearAlgebra.html
    x = A.colPivHouseholderQr().solve(B);
	if ( B.isApprox(A*x,1e-2)) // Precision of isApprox
	{
        bool converged = true;
		for (int m = 0; m < 6; m++)
		{
			tr[m] += step_size * x(m,0);
			if (fabs( x(m,0) ) > eps)
				converged = false;
		}

        if (converged)
		    return CONVERGED;
        else
            return UPDATED;
	}
	else
	{
        std::cout << "colPivHouseholderQr() results FAILED, difference: " << (A*x - B).transpose() << std::endl;
		return FAILED;
	}

    return FAILED;
}


std::vector<int> StereoMotionEstimator::getRandomSample (int N,int num) {

    // std::random_device rd;
    // std::mt19937 gen(rd());

    // std::uniform_int_distribution<int> dis(0, N-1);
    std::vector<int> result;

    for (int i=0 ; i<num ; )
    {
        // int data = dis(gen);

        int data = rand() % N;

        bool duplicated = false;
        for ( auto e : result)
        {
            if (e == data)
            {
                duplicated = true;
                break;
            }
        }

        if (!duplicated)
        {
            result.push_back(data);
            i++;
        }
    }

    return result;
}

std::vector<int> StereoMotionEstimator::getInlier(std::vector<double> &tr)
{

	// mark all observations active, empty
	std::vector<int> active;

	// extract observations and compute predictions
	computeObservations(active, true);
	computeResidualsAndJacobian(tr, active, true);

	// compute inliers
    double threshold = param.inlier_threshold * param.inlier_threshold;
	std::vector<int> inliers;
	for (size_t i = 0; i < matches_quad_vec->size() ; i++)
    {
		double sq0 = p_residual[4 * i + 0] * p_residual[4 * i + 0];
		double sq1 = p_residual[4 * i + 1] * p_residual[4 * i + 1];
		double sq2 = p_residual[4 * i + 2] * p_residual[4 * i + 2];
		double sq3 = p_residual[4 * i + 3] * p_residual[4 * i + 3];
		if ( (sq0 + sq1 + sq2 + sq3) / 2.0 < threshold)
            inliers.push_back(i);
        // double diff_lx = p_observe[4 * i + 0] - p_predict[4 * i + 0];
        // double diff_ly = p_observe[4 * i + 1] - p_predict[4 * i + 1];
        // double diff_rx = p_observe[4 * i + 2] - p_predict[4 * i + 2];
        // double diff_ry = p_observe[4 * i + 3] - p_predict[4 * i + 3];

        // if ( diff_lx*diff_lx + diff_ly*diff_ly + diff_rx*diff_rx + diff_ry*diff_ry < threshold)
        //     inliers.push_back(i);
    }

	return inliers;
}


void StereoMotionEstimator::computeResidualsAndJacobian(const std::vector<double> &tr, const std::vector<int> &active, bool inlier_mode)
{

    const double &_cu = param.calib.cu;
    const double &_cv = param.calib.cv;
    const double &_f = param.calib.f;
    
	// extract motion parameters
	const double &rx = tr[0]; const double &ry = tr[1]; const double &rz = tr[2];
	const double &tx = tr[3]; const double &ty = tr[4]; const double &tz = tr[5];

	// precompute sine/cosine
	const double sx = std::sin(rx); const double cx = std::cos(rx); const double sy = std::sin(ry);     // rx = alpha, ry = beta, rz = gamma
	const double cy = std::cos(ry); const double sz = std::sin(rz); const double cz = std::cos(rz);

	// compute rotation matrix and derivatives
	// rotation matrix = Rz*Ry*Rx
	// hm: reference http://www.songho.ca/opengl/gl_anglestoaxes.html (Rx*Ry*Rz)
	double r00 = +cy*cz;          double r01 = -cy*sz;          double r02 = +sy;
	double r10 = +sx*sy*cz + cx*sz; double r11 = -sx*sy*sz + cx*cz; double r12 = -sx*cy;
	double r20 = -cx*sy*cz + sx*sz; double r21 = +cx*sy*sz + sx*cz; double r22 = +cx*cy;

	double rdrx10 = +cx*sy*cz - sx*sz; double rdrx11 = -cx*sy*sz - sx*cz; double rdrx12 = -cx*cy;
	double rdrx20 = +sx*sy*cz + cx*sz; double rdrx21 = -sx*sy*sz + cx*cz; double rdrx22 = -sx*cy;
	double rdry00 = -sy*cz;          double rdry01 = +sy*sz;          double rdry02 = +cy;
	double rdry10 = +sx*cy*cz;       double rdry11 = -sx*cy*sz;       double rdry12 = +sx*sy;
	double rdry20 = -cx*cy*cz;       double rdry21 = +cx*cy*sz;       double rdry22 = -cx*sy;
	double rdrz00 = -cy*sz;          double rdrz01 = -cy*cz;
	double rdrz10 = -sx*sy*sz + cx*cz; double rdrz11 = -sx*sy*cz - cx*sz;
	double rdrz20 = +cx*sy*sz + sx*cz; double rdrz21 = +cx*sy*cz - sx*sz;

	// loop variables
	double X1cd, Y1cd, Z1cd;

	// for all observations do
	for (size_t i = 0; i < active.size() || (inlier_mode && i < matches_quad_vec->size() ); i++)
	{

		// get 3d point in previous coordinate system
		const double &X1p = X[ (inlier_mode ? i : active[i]) ];
		const double &Y1p = Y[ (inlier_mode ? i : active[i]) ];
		const double &Z1p = Z[ (inlier_mode ? i : active[i]) ];

		// compute 3d point in current left coordinate system
		double X1c = r00*X1p + r01*Y1p + r02*Z1p + tx;
		double Y1c = r10*X1p + r11*Y1p + r12*Z1p + ty;
		double Z1c = r20*X1p + r21*Y1p + r22*Z1p + tz;

		// weighting   hm: (centre points are given up to 20x of weight, only in x-axis)
		double weight = 1.0;
		if (param.reweighting)
			weight = 1.0 / (fabs(p_observe[4 * i + 0] - _cu) / fabs(_cu) + 0.5);   // only for current left image

		// compute 3d point in current right coordinate system
		double X2c = X1c - param.calib.baseline;

		// for all paramters do  // six parameters: 3 rotations and 3 translations
		for (int j = 0; !inlier_mode && j < 6; j++)
		{

			// derivatives of 3d pt. in curr. left coordinates wrt. param j
			switch (j)
			{
			case 0: X1cd = 0;
				Y1cd = rdrx10*X1p + rdrx11*Y1p + rdrx12*Z1p;
				Z1cd = rdrx20*X1p + rdrx21*Y1p + rdrx22*Z1p;
				break;
			case 1: X1cd = rdry00*X1p + rdry01*Y1p + rdry02*Z1p;
				Y1cd = rdry10*X1p + rdry11*Y1p + rdry12*Z1p;
				Z1cd = rdry20*X1p + rdry21*Y1p + rdry22*Z1p;
				break;
			case 2: X1cd = rdrz00*X1p + rdrz01*Y1p;
				Y1cd = rdrz10*X1p + rdrz11*Y1p;
				Z1cd = rdrz20*X1p + rdrz21*Y1p;
				break;
			case 3: X1cd = 1; Y1cd = 0; Z1cd = 0; break;
			case 4: X1cd = 0; Y1cd = 1; Z1cd = 0; break;
			case 5: X1cd = 0; Y1cd = 0; Z1cd = 1; break;
			}

			// set jacobian entries (project via K)
			J[(4 * i + 0) * 6 + j] = weight*_f*(X1cd*Z1c - X1c*Z1cd) / (Z1c*Z1c); // left u'
			J[(4 * i + 1) * 6 + j] = weight*_f*(Y1cd*Z1c - Y1c*Z1cd) / (Z1c*Z1c); // left v'
			J[(4 * i + 2) * 6 + j] = weight*_f*(X1cd*Z1c - X2c*Z1cd) / (Z1c*Z1c); // right u'
			J[(4 * i + 3) * 6 + j] = weight*_f*(Y1cd*Z1c - Y1c*Z1cd) / (Z1c*Z1c); // right v'
		}

		// set prediction (project via K)
		p_predict[4 * i + 0] = _f*X1c / Z1c + _cu; // left center u
		p_predict[4 * i + 1] = _f*Y1c / Z1c + _cv; // left v
		p_predict[4 * i + 2] = _f*X2c / Z1c + _cu; // right u
		p_predict[4 * i + 3] = _f*Y1c / Z1c + _cv; // right v

		// set residuals
		p_residual[4 * i + 0] = weight*(p_observe[4 * i + 0] - p_predict[4 * i + 0]);
		p_residual[4 * i + 1] = weight*(p_observe[4 * i + 1] - p_predict[4 * i + 1]);
		p_residual[4 * i + 2] = weight*(p_observe[4 * i + 2] - p_predict[4 * i + 2]);
		p_residual[4 * i + 3] = weight*(p_observe[4 * i + 3] - p_predict[4 * i + 3]);

        // if (inlier_mode)
        // {
        //     std::cout << 4*i << ": p_residual[4*i]= " << p_residual[4*i] << std::endl;
        // }
	} 
}


Eigen::Affine3d StereoMotionEstimator::transformationVectorToMatrix( const std::vector<double> &tr ) {

  // extract parameters
  const double &rx = tr[0];
  const double &ry = tr[1];
  const double &rz = tr[2];
  const double &tx = tr[3];
  const double &ty = tr[4];
  const double &tz = tr[5];

  // precompute sine/cosine
  const double sx = sin(rx);
  const double cx = cos(rx);
  const double sy = sin(ry);
  const double cy = cos(ry);
  const double sz = sin(rz);
  const double cz = cos(rz);

  // compute transformation
  Eigen::Matrix<double,4,4> Tr;
  Tr(0,0) = +cy*cz;          Tr(0,1) = -cy*sz;          Tr(0,2) = +sy;    Tr(0,3) = tx;
  Tr(1,0) = +sx*sy*cz+cx*sz; Tr(1,1) = -sx*sy*sz+cx*cz; Tr(1,2) = -sx*cy; Tr(1,3) = ty;
  Tr(2,0) = -cx*sy*cz+sx*sz; Tr(2,1) = +cx*sy*sz+sx*cz; Tr(2,2) = +cx*cy; Tr(2,3) = tz;
  Tr(3,0) = 0;               Tr(3,1) = 0;               Tr(3,2) = 0;      Tr(3,3) = 1;

  Eigen::Affine3d affine;
  affine = Tr;
  return affine;
}
