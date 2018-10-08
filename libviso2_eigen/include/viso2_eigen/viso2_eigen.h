/**
 * @file viso2_eigen.h
 * @author Huimin Cheng (NUS)
 * @brief A rewrite of original libviso2 into modern Eigen library.
 * @version 0.1
 * @date 2018-10-02
 * 
 * @copyright Copyright (c) 2018
 * 
 */

#ifndef VISO2_EIGEN_H
#define VISO2_EIGEN_H

#include <memory>
#include <bitset>
#include <opencv2/opencv.hpp>

// #include "quad_matcher.h"
// #include "stereo_motion_estimator.h"
#include "viso2_eigen_params.h"


class StereoMotionEstimator;

template<class TDescriptor, class TFeature>
class QuadMatcher;

class Viso2Eigen{

public:

    typedef std::bitset<256> bitset;

    enum Mode{
        FIRST_FRAME,
        USE_OLD_FRAME,
        NO_USE_OLD_FRAME
    };

    Viso2Eigen();
    ~Viso2Eigen();

    void setParam(int image_width, int image_height,
        QuadMatcherParam::Parameters& qm_param, StereoMotionEstimatorParam::Parameters& sme_param);

    bool process(const cv::Mat& leftImage, const cv::Mat& rightImage, Mode mode = NO_USE_OLD_FRAME, uint64_t stamp = 0);

    
    Eigen::Affine3d getCameraMotion();
    std::vector<int> getInlier(); // the inlier index is sorted in the ascending order

    double computeOpticalFlow();

    // initialised is true only after setParam is called
    bool isInitialised() { return initialised; }

    void getVisualisation(cv::Mat& image, cv::Mat& image_right) {
        image = this->outImg;
        image_right = this->outImg_right;
    }
    

private:
    inline void mat2Bitset(const cv::Mat& des_vec_in, std::vector<bitset>& des_vec_out);
    void detectAndCompute(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, std::vector<bitset>& descriptors);

    void drawKeypointMotion(cv::Mat& image, cv::Mat& image_right);

    bool initialised;

    std::unique_ptr<QuadMatcher<bitset,std::nullptr_t>> qm; // 2nd template argument TFeature is no longer used
    std::unique_ptr<StereoMotionEstimator> sme;


    std::vector< cv::KeyPoint > keys_l1, keys_r1, keys_l2, keys_r2; // previous == 1, current == 2
    std::vector<bitset> des_l1, des_r1, des_l2, des_r2;
    std::vector< std::array<int,4> > matches_quad_vec;

    cv::Mat outImg, outImg_right;

    bool compulte_scaled_keys;

};

#endif /* VISO2_EIGEN_H */
