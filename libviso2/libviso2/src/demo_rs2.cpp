#include <iostream>
#include <string>
#include <vector>
#include <stdint.h>

#include "viso_stereo.h"
#include <png++/png.hpp>

#include <opencv2/opencv.hpp>   // Include OpenCV API

#include <chrono>

cv::Mat cv_drawMatches(std::string cv_window_name, cv::Mat cv_leftImg, 
				cv::Mat cv_rightImg, std::vector<Matcher::p_match> _matches, std::vector<int> _inlierIdx);

int main (int argc, char** argv) {

	std::string cv_window_name = "demo_rs2";

	cv::namedWindow(cv_window_name, cv::WINDOW_AUTOSIZE);

	// we need the path name to 2010_03_09_drive_0019 as input argument
	// if (argc<2) {
	//   cerr << "Usage: /home/dhl/git/catkin_ws/data/2018-06-12_indoorloop_highGain" << endl;
	//   return 1;
	// }

	// sequence directory
	std::string dir = "/home/dhl/git/catkin_ws/data/2018-06-15-8000exp-1"; //argv[1];
	
	// set most important visual odometry parameters
	// for a full parameter list, look at: viso_stereo.h
	VisualOdometryStereo::parameters param;
	
	// calibration parameters for sequence 2010_03_09_drive_0019 
	bool resolution_640 = true;
	if (!resolution_640)
	{
		param.calib.f  = 937.13372802734375; // 645.24; // focal length in pixels
		param.calib.cu = 634.03375244140625; // 635.96; // principal point (u-coordinate) in pixels
		param.calib.cv = 358.10275268554688; //194.13; // principal point (v-coordinate) in pixels
		param.base     = 0.055025237236240539; //0.5707; // baseline in meters
	}
	else
	{
		param.calib.f  = 624.7557983398438; // 645.24; // focal length in pixels
		param.calib.cu = 316.02252197265625; // 635.96; // principal point (u-coordinate) in pixels
		param.calib.cv = 238.73516845703125; //194.13; // principal point (v-coordinate) in pixels
		param.base     = 0.055025237236240539; //0.5707; // baseline in meters
	}
	
	//param.bucket.bucket_ornot = true;


	//param.match.fast_threshold_dense = 5; //40
	//param.match.numFastFeature_dense = 5000;// 2000

	// init visual odometry
	VisualOdometryStereo viso(param);
	
	// current pose (this matrix transforms a point from the current
	// frame's camera coordinates to the first frame's camera coordinates)
	Matrix pose = Matrix::eye(4);
		
	// loop through all frames i=0:372
	for (int32_t i=0; i<=858; i++) {

		// input file names
		char base_name[256]; 
		sprintf(base_name,"%06d.png",i);
		std::string left_img_file_name  = dir + "/L" + base_name;
		std::string right_img_file_name = dir + "/R" + base_name;
		
		// catch image read/write errors here
		try {

			// load left and right input image
			png::image< png::gray_pixel > left_img(left_img_file_name);
			png::image< png::gray_pixel > right_img(right_img_file_name);

			// image dimensions
			int32_t width  = left_img.get_width();
			int32_t height = left_img.get_height();

			// convert input images to uint8_t buffer
			uint8_t* left_img_data  = (uint8_t*)malloc(width*height*sizeof(uint8_t));
			uint8_t* right_img_data = (uint8_t*)malloc(width*height*sizeof(uint8_t));
			int32_t k=0;
			for (int32_t v=0; v<height; v++) {
				for (int32_t u=0; u<width; u++) {
					left_img_data[k]  = left_img.get_pixel(u,v);
					right_img_data[k] = right_img.get_pixel(u,v);
					k++;
				}
			}

			cv::Mat cv_leftImg(cv::Size(width, height), CV_8UC1, (void *)left_img_data, cv::Mat::AUTO_STEP);
			cv::Mat cv_rightImg(cv::Size(width, height), CV_8UC1, (void *)right_img_data, cv::Mat::AUTO_STEP);
			// cv::Mat(cv::Size(w, h), CV_8UC1, irleft, cv::Mat::AUTO_STEP);  

			auto clache = cv::createCLAHE(4,cv::Size(6,8));

			cv::Mat leftbuffer(cv::Size(width, height), CV_8UC1);
			cv::Mat rightbuffer(cv::Size(width, height), CV_8UC1);

			clache->apply(cv_leftImg,leftbuffer);
			clache->apply(cv_rightImg,rightbuffer);

			
			cv::bilateralFilter(leftbuffer,cv_leftImg,5,10,10);
			cv::bilateralFilter(rightbuffer,cv_rightImg,5,10,10);

			//cv::equalizeHist(cv_leftImg,cv_leftImg);
			//cv::equalizeHist(cv_rightImg,cv_rightImg);

			// status
			std::cout << "Processing: Frame: " << i ;
			
			// compute visual odometry
			int32_t dims[] = {width,height,width};

			auto start_t = std::chrono::system_clock::now();
			bool success = viso.process(cv_leftImg.data,cv_rightImg.data,dims);
			auto end_t = std::chrono::system_clock::now();
			std::chrono::duration<double> diff = end_t - start_t;
			std::cout << ", calc. time: " << diff.count();

			// output some statistics
			double num_matches = viso.getNumberOfMatches();
			double num_inliers = viso.getNumberOfInliers();
			std::cout << ", Matches: " << num_matches;
			std::cout << ", Inliers: " << 100.0*num_inliers/num_matches << " %" ;
			// << ", Current pose: " << std::endl;
			// std::cout << pose << std::endl ;
			std::cout << std::endl;

			if (success) {
			
				// on success, update current pose
				pose = pose * Matrix::inv(viso.getMotion());				

			} else {
				std::cerr << " ... failed!" << std::endl;
			}

			std::vector<Matcher::p_match> _matches = viso.getMatches();
			auto _inlierIdx = viso.getInlierIndices();

			cv::Mat outImg = cv_drawMatches(cv_window_name, cv_leftImg, cv_rightImg, _matches, _inlierIdx);

			std::string outpath = "/home/dhl/git/catkin_ws/data/out/";
			outpath= outpath + base_name;
			cv::imwrite(outpath,outImg);
			

			// release uint8_t buffers
			free(left_img_data);
			free(right_img_data);

		// catch image read errors here
		} catch (...) {
			std::cerr << "ERROR: Couldn't read input files!" << std::endl;
			return 1;
		}
	}
	
	// output
	cout << "Demo complete! Exiting ..." << endl;

	// exit
	return 0;
}



cv::Mat cv_drawMatches(std::string cv_window_name, cv::Mat cv_leftImg, cv::Mat cv_rightImg, std::vector<Matcher::p_match> _matches, std::vector<int> _inlierIdx)
{
	int num_matches = _matches.size();
	int num_inliers = _inlierIdx.size();

	std::vector<cv::KeyPoint> match_pre_left, match_pre_right, match_curr_left, match_curr_right;
	std::vector<cv::DMatch> matches1to2;

	int i=0;
	for (auto match : _matches)
	{
		cv::KeyPoint keypoint;
		keypoint.pt.x = match.u1p;
		keypoint.pt.y = match.v1p;
		match_pre_left.push_back(keypoint);

		keypoint.pt.x = match.u2p;
		keypoint.pt.y = match.v2p;
		match_pre_right.push_back(keypoint);

		keypoint.pt.x = match.u1c;
		keypoint.pt.y = match.v1c;
		match_curr_left.push_back(keypoint);

		keypoint.pt.x = match.u2c;
		keypoint.pt.y = match.v2c;
		match_curr_right.push_back(keypoint);

		cv::DMatch idx;
		idx.queryIdx=i;
		idx.trainIdx=i;
		matches1to2.push_back(idx);
		i++;
	}

	std::vector<char> mask_inlier;
	std::vector<char> mask_inlier_inv;

	mask_inlier.resize(num_matches);
	mask_inlier_inv.resize(num_matches);
	for (auto inlier : _inlierIdx)
	{
		mask_inlier[inlier] = true;
	}

	for (i=0; i<mask_inlier.size();i++)
	{
		mask_inlier_inv[i] = !mask_inlier[i];
	}

	cv::Mat outImg;
	cv::drawMatches(cv_leftImg, match_curr_left, cv_rightImg, match_curr_right, matches1to2, outImg, cv::Scalar(0,255,0), cv::Scalar(255,0,0),mask_inlier);

	//cv::Mat outImg(480, 640, CV_8UC3, cv::Scalar(255,0,255));
	
	cv::imshow(cv_window_name,outImg);
	cvWaitKey(1);

	return outImg;
	
}