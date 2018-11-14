#include <iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>

#include "quad_matcher.h"
#include "stereo_motion_estimator.h"
#include "viso2_eigen.h"

#include <chrono>

Viso2Eigen::Viso2Eigen() :  initialised(false) , 
                    qm (new QuadMatcher<bitset,std::nullptr_t>()),
                    sme (new StereoMotionEstimator()){}

Viso2Eigen::~Viso2Eigen() = default;

void Viso2Eigen::setParam(int image_width, int image_height,
        QuadMatcherParam::Parameters& qm_param, StereoMotionEstimatorParam::Parameters& sme_param){
        

        qm_param.image_width = sme_param.image_width = image_width;
        qm_param.image_height = sme_param.image_height = image_height;

        // Initialise paramters for both matcher and motion estimator
        qm->setParam(qm_param);
        sme->setParam(sme_param);

        this->compulte_scaled_keys = qm_param.compulte_scaled_keys; // use of octave entry in the cv::KeyPoint

        initialised = true;
        std::cout << "Viso2 Initialised with Parameters!" << std::endl;
}


bool Viso2Eigen::process(const cv::Mat& leftImage, const cv::Mat& rightImage, Viso2Eigen::Mode mode, uint64_t stamp){

    current_frame_feature_valid = false;
    //////////////////////////////
    //// STEP 1: Mode Logics to keep or throw away old frames
    //////////////////////////////

    if ( mode == NO_USE_OLD_FRAME){
        // remove old previous frames
        keys_l1 = std::move(keys_l2);
        keys_r1 = std::move(keys_r2);
        des_l1 = std::move(des_l2);
        des_r1 = std::move(des_r2);

        // assert(!keys_l1.empty() && !des_l1.empty());
    }

    keys_l2.clear();
    keys_r2.clear();
    des_l2.clear();
    des_r2.clear();

    //// Visualisation comes first
    cv::Mat outImg_tmp, outImg_right_tmp;
    cv::cvtColor(leftImage,outImg_tmp,cv::COLOR_GRAY2BGR);
    cv::cvtColor(rightImage,outImg_right_tmp,cv::COLOR_GRAY2BGR);
    outImg = outImg_tmp;
    outImg_right= outImg_right_tmp;

    //////////////////////////////
    //// STEP 2: Calculate current frame keys and descriptors
    //////////////////////////////
    {
        auto begin = std::chrono::steady_clock::now();


        const int fast_th = 25; // corner detector response threshold
        cv::FAST(leftImage, keys_l2, fast_th, /*nonmaxSuppression=*/ true);
        cv::FAST(rightImage, keys_r2, fast_th, /*nonmaxSuppression=*/ true);


        qm->bucketKeyPoints(keys_l2);

        qm->bucketKeyPoints(keys_r2);

        auto extractor = cv::xfeatures2d::BriefDescriptorExtractor::create(/*int 	bytes = 32, bool 	use_orientation = false*/);

        // auto extractor = cv::ORB::create(1000);
        // extractor->detect(leftImage,keys_l2);
        // extractor->detect(rightImage,keys_r2);
        
        cv::Mat descriptors;
        extractor->compute(leftImage, keys_l2, descriptors);
        mat2Bitset(descriptors, des_l2);

        descriptors.release();
        extractor->compute(rightImage, keys_r2, descriptors);
        mat2Bitset(descriptors, des_r2);

        if ( (keys_l2.size() + keys_r2.size() / 2.0) < 80 && compulte_scaled_keys)
        {
            cv::Mat leftImage_half, rightImage_half;
            std::vector< cv::KeyPoint > keys_l2_half, keys_r2_half;

            const double scale = 0.25;
            cv::resize(leftImage, leftImage_half, cv::Size(), scale, scale);
            cv::resize(rightImage, rightImage_half, cv::Size(), scale, scale);
            cv::FAST(leftImage_half, keys_l2_half, fast_th, /*nonmaxSuppression=*/ true);
            cv::FAST(rightImage_half, keys_r2_half, fast_th, /*nonmaxSuppression=*/ true);

            qm->bucketKeyPoints(keys_l2_half,1/scale);
            qm->bucketKeyPoints(keys_r2_half,1/scale);

            std::vector <bitset> des_l2_half, des_r2_half;
            descriptors.release();
            extractor->compute(leftImage_half, keys_l2_half, descriptors);
            mat2Bitset(descriptors, des_l2_half);

            descriptors.release();
            extractor->compute(rightImage_half, keys_r2_half, descriptors);
            mat2Bitset(descriptors, des_r2_half);

            for (auto &key : keys_l2_half){
                key.octave = 1;
                key.pt.x /=scale;
                key.pt.y /=scale;
            }
                
            
            for (auto &key : keys_r2_half){
                key.octave = 1;
                key.pt.x /=scale;
                key.pt.y /=scale;
            }      

            std::cout<< "# octave0 = " << keys_l2.size() << ", # octave 1 = " << keys_l2_half.size() << std::endl;

            //// append scaled down version of image keys and descriptors to the main vectors
            keys_l2.reserve(keys_l2.size()+keys_l2_half.size());
            keys_r2.reserve(keys_r2.size()+keys_r2_half.size());

            keys_l2.insert(keys_l2.end(), keys_l2_half.begin(), keys_l2_half.end());
            keys_r2.insert(keys_r2.end(), keys_r2_half.begin(), keys_r2_half.end());

            des_l2.reserve(des_l2.size()+des_l2_half.size());
            des_r2.reserve(des_r2.size()+des_r2_half.size());

            des_l2.insert(des_l2.end(), des_l2_half.begin(), des_l2_half.end());
            des_r2.insert(des_r2.end(), des_r2_half.begin(), des_r2_half.end());
        }

        // std::cout<< "Left Features" << std::endl;
        // for (size_t i = 0; i<keys_l2.size();i++)
        // {
        //     std::cout<< "[" << i << "] = " << keys_l2[i].pt.x << "," <<keys_l2[i].pt.y << "(response " << keys_l2[i].response << ") octave=" <<keys_l2[i].octave << std::endl;
        // }

        // std::cout << "des_r2.size()=" << des_r2.size() << ", descriptors.rows=" << descriptors.rows <<std::endl;
        // assert(des_r2.size() == descriptors.rows);

        if ( keys_l2.size() > 10 && keys_r2.size() > 10){
            current_frame_feature_valid = true;
        }

        auto end = std::chrono::steady_clock::now();

        std::cout << "Calc Keys & Descriptors = " 
            << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "ms" <<std::endl;

        if (mode == FIRST_FRAME)
        {
            return true;
        }
    }

        

    //////////////////////////////
    //// STEP 3: Quad Matching
    //////////////////////////////
    bool qm_result;
    {

        if (keys_l1.size()<4 || keys_l2.size()<4 || keys_r1.size()<4 || keys_r2.size()<4)
            qm_result = false;
        else{
            auto begin = std::chrono::steady_clock::now();
            qm->pushBackData(keys_l1,keys_l2,keys_r1,keys_r2, des_l1,des_l2,des_r1,des_r2);
            qm_result = qm->matchFeaturesQuad();
            auto end = std::chrono::steady_clock::now();

                std::cout << "Calc Quad Matching = " 
            << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "ms" <<std::endl;
        }
        

    }
    

    //////////////////////////////
    //// STEP 4: Motion Estimation
    //////////////////////////////

    matches_quad_vec.clear();

    if (qm_result){

        qm->getMatchesQuad(matches_quad_vec);

        bool sme_result = false;
        auto begin = std::chrono::steady_clock::now();
        if (matches_quad_vec.size() < 10)
        {
            std::cerr << "Total poll of matches is too small < " << matches_quad_vec.size() << ", waiting for the next frame " << std::endl;
        }else{
            
             // previous left --> current left --> current right --> previous right
            sme->pushBackData(matches_quad_vec, keys_l1, keys_l2,
                keys_r2, keys_r1);

            sme_result = sme->updateMotion();
        }       
        auto end = std::chrono::steady_clock::now();
        std::cout << "Calc updateMotion = " 
            << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "ms" <<std::endl;

        //// Visualisation Code
        begin = std::chrono::steady_clock::now();

        
        // cv::Mat outIm(cv::Size(qm_param.image_width,qm_param.image_height),CV_8UC1);
        cv::drawKeypoints(leftImage, keys_l2,outImg_tmp, cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_OVER_OUTIMG);
        cv::drawKeypoints(rightImage, keys_r2,outImg_right_tmp, cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_OVER_OUTIMG);

        drawKeypointMotion(outImg_tmp, outImg_right_tmp);

        cv::putText(outImg_tmp, "Left: " + std::to_string(stamp), cv::Point(5 /*column*/ ,30 /*row*/), cv::FONT_HERSHEY_COMPLEX_SMALL, 1.0, cv::Scalar(255,255,255));
        cv::putText(outImg_right_tmp, "Right: " + std::to_string(stamp), cv::Point(5 /*column*/ ,30 /*row*/), cv::FONT_HERSHEY_COMPLEX_SMALL, 1.0, cv::Scalar(255,255,255));

        outImg = outImg_tmp;
        outImg_right= outImg_right_tmp;

        end = std::chrono::steady_clock::now();

        std::cout << "Calc Visualisation = " 
            << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "ms" <<std::endl;

        if (sme_result){
            return true;
        }else{
            std::cerr << "updateMotion() Fails." << std::endl;
            return false;
        }

        // sme->getCameraMotion();

    }else
    {
        std::cerr << "matchFeaturesQuad() Fails." << std::endl;
        return false;
    }

    return false;
}

Eigen::Affine3d Viso2Eigen::getCameraMotion() {return sme->getCameraMotion();}
std::vector<int> Viso2Eigen::getInlier() {return sme->getInlier();}
double Viso2Eigen::getArea(){return sme->getArea(); }

double Viso2Eigen::computeOpticalFlow(){

    std::vector<int> inliers = getInlier();

    double highest_flow = 0.0;

    for (size_t i=0; i<inliers.size(); i++){
        size_t idx = inliers[i];
        const float &x1 = keys_l1[ matches_quad_vec[idx][0] ].pt.x;
        const float &y1 = keys_l1[ matches_quad_vec[idx][0] ].pt.y;
        const float &x2 = keys_l2[ matches_quad_vec[idx][1] ].pt.x;
        const float &y2 = keys_l2[ matches_quad_vec[idx][1] ].pt.y;

        double x_diff = x2 - x1;
        double y_diff = y2 - y1;

        double flow = x_diff*x_diff + y_diff*y_diff;

        if (highest_flow < flow) highest_flow = flow;
    }

    return highest_flow;
}


void Viso2Eigen::mat2Bitset(const cv::Mat& des_vec_in, std::vector<bitset>& des_vec_out){
    
    // des_vec_out.clear();
    des_vec_out.resize(des_vec_in.rows);
    for ( int i = 0 ; i < des_vec_in.rows ; i++)
        memcpy(&des_vec_out[i], des_vec_in.data + i*32, 32 /*32 byte == 256bit*/);

}

void Viso2Eigen::drawKeypointMotion(cv::Mat& image, cv::Mat& image_right){

    std::vector<int> inlier = sme->getInlier();

    std::vector<bool> inlier_vec(matches_quad_vec.size(),false);

    for (auto e : inlier)
        inlier_vec[e] = true;

    // trim keys_l2 with only matched points
    std::vector <cv::KeyPoint> keys_l2_matched_inliers, keys_l2_matched_outliers;
    std::vector <cv::KeyPoint> keys_r2_matched_inliers, keys_r2_matched_outliers;

    for ( size_t i = 0; i < matches_quad_vec.size(); i++ ){
        int idx = matches_quad_vec[i][1]; // current left index, in the match list
        int idx_right = matches_quad_vec[i][2]; // current right index, in the match list

        if (inlier_vec[i])
        {
            keys_l2_matched_inliers.push_back( keys_l2[idx] );
            keys_r2_matched_inliers.push_back( keys_r2[idx_right] );
        }
        else
        {
            keys_l2_matched_outliers.push_back( keys_l2[idx] );
            keys_r2_matched_outliers.push_back( keys_r2[idx_right] );
        }

    }
    cv::drawKeypoints(image, keys_l2_matched_outliers, image, cv::Scalar(255,0,0), cv::DrawMatchesFlags::DRAW_OVER_OUTIMG /*| cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS*/  ); // BGR
    cv::drawKeypoints(image, keys_l2_matched_inliers, image, cv::Scalar(0,255,0), cv::DrawMatchesFlags::DRAW_OVER_OUTIMG /*| cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS*/  ); // BGR
    cv::drawKeypoints(image_right, keys_r2_matched_outliers, image_right, cv::Scalar(255,0,0), cv::DrawMatchesFlags::DRAW_OVER_OUTIMG /*| cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS*/  ); // BGR
    cv::drawKeypoints(image_right, keys_r2_matched_inliers, image_right, cv::Scalar(0,255,0), cv::DrawMatchesFlags::DRAW_OVER_OUTIMG /*| cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS*/  ); // BGR

    // draw motion vector for each matched keypoint
    for ( size_t i = 0; i < matches_quad_vec.size(); i++ ){
        int idx_from = matches_quad_vec[i][0];
        int idx_to = matches_quad_vec[i][1];
        int idx_from_right = matches_quad_vec[i][3];
        int idx_to_right = matches_quad_vec[i][2];
        if (inlier_vec[i])
        {
            cv::line(image, keys_l1[idx_from].pt , keys_l2[idx_to].pt , cv::Scalar(0,180,0), 1, CV_AA);
            cv::line(image_right, keys_r1[idx_from_right].pt , keys_r2[idx_to_right].pt , cv::Scalar(0,180,0), 1, CV_AA);
        }
        else
        {
            cv::line(image, keys_l1[idx_from].pt , keys_l2[idx_to].pt , cv::Scalar(180,0,0), 1, CV_AA);
            cv::line(image_right, keys_r1[idx_from_right].pt , keys_r2[idx_to_right].pt , cv::Scalar(180,0,0), 1, CV_AA);
        }
            
    }



}