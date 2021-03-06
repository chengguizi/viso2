#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <image_geometry/stereo_camera_model.h>
#include <image_geometry/pinhole_camera_model.h>
#include <cv_bridge/cv_bridge.h>

#include <viso2_eigen/viso2_eigen.h>

#include "stereo_processor.h"
#include "odometer_base.h"

// to remove after debugging
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>

#include <DUtils/Profiler.h>

#include <iostream>
#include <cassert>
#include <mutex>

#include <memory>

#include <cmath>

namespace viso2_ros
{

DUtils::Profiler profiler;

// StereoProcessor exposes the imageCallback() for synced image and camera info (subscribers)
// OdometerBase provides publisher to odometry and pose topic  integrateAndPublish()
class StereoOdometer : public StereoProcessor , public OdometerBase
{

private:

	ros::Subscriber _reset_sub;
	std::unique_ptr<Viso2Eigen> viso2;
	viso2_ros::Parameters param;

	std::mutex in_process;

	struct TfStamped{
		Eigen::Affine3d tf;
		uint64_t stamp = 0;
		bool lost = false;
		bool use_old_reference_frame = false; // updated by the previous frame
	};

	struct VOState {
		uint64_t last_frame_time_ns = 0;
		Eigen::Affine3d reference_motion = Eigen::Affine3d::Identity(); // updated by the previous frame
		std::vector<TfStamped> tfStamped;
	};

	VOState voState;

	cv::Mat rectImg;

public:

	StereoOdometer(const std::string& transport, const viso2_ros::Parameters& param) : 
		StereoProcessor(transport, param.queue_size, param.left_topic, param.right_topic, param.left_info_topic, param.right_info_topic), // OdometerBase(), 
		viso2(new Viso2Eigen()), param(param)
	{
		
		voState.tfStamped.reserve(2);
		ROS_INFO("StereoOdometer() initialised! Waiting for images...");

		// Resolve topic names
		ros::NodeHandle nh;

		// Subscribe to reset topic
    	_reset_sub = nh.subscribe("/reset", 1, &StereoOdometer::resetCallback, this);
	}

	~StereoOdometer(){
		std::cout << std::endl << "Execution time: "  << std::endl
		<< " - viso->process() : (mean) " << profiler.getMeanTime("viso->process") * 1e3
		<< " ms/image, (max)" << profiler.getMaxTime("viso->process") * 1e3
		<< " ms/image" << std::endl;
	}

	void getVisualisation(cv::Mat& image, cv::Mat& image_right)
	{
		viso2->getVisualisation(image,image_right);
	}

	uint64_t getLastFrameTimeNs() {return voState.last_frame_time_ns;}

protected:

	void initOdometer(	const sensor_msgs::CameraInfoConstPtr& l_info_msg,
						const sensor_msgs::CameraInfoConstPtr& r_info_msg){
		
		if (!param.use_different_stereo_calibration){
			// read calibration info from camera info message
			// to fill remaining parameters
			image_geometry::StereoCameraModel model;
			model.fromCameraInfo(*l_info_msg, *r_info_msg); // http://wiki.ros.org/image_pipeline/CameraInfo
			param.sme_param.calib_left.baseline = model.baseline(); // calculated as -right_.Tx() / right_.fx(); fx() is defined in pixels. baseline is in meter
			// Note: the Tx() in ROS is defined as   -(baseline (meter) * focal length (pixel)), not just baseline alone!
			// https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
			param.sme_param.calib_left.fx = model.left().fx(); // in pixels
			param.sme_param.calib_left.fy = model.left().fy(); // in pixels
			param.sme_param.calib_left.cu = model.left().cx();
			param.sme_param.calib_left.cv = model.left().cy();
			param.sme_param.calib_right = param.sme_param.calib_left;
			ROS_WARN("Using the same calibration for left and right image sensors.");

		}else{
			image_geometry::PinholeCameraModel model_left, model_right;

			model_left.fromCameraInfo(*l_info_msg);
			model_right.fromCameraInfo(*r_info_msg);

			const double left_Tz = model_left.projectionMatrix()(2,3);
			const double right_Tz = model_right.projectionMatrix()(2,3);

			ROS_ASSERT(model_left.Tx() == 0); // left camera should be at origin
			ROS_ASSERT(model_left.Ty() == 0);
			ROS_ASSERT(left_Tz == 0);

			param.sme_param.calib_left.baseline = - model_right.Tx() / model_right.fx(); // only take care of baseline along x-axis
			param.sme_param.calib_left.fx = model_left.fx();
			param.sme_param.calib_left.fy = model_left.fy();
			ROS_ASSERT( std::abs((model_left.fx() - model_left.fy()) / model_left.fy()) < 1e-2 ); // difference between fx and fy should be small
			param.sme_param.calib_left.cu = model_left.cx();
			param.sme_param.calib_left.cv = model_left.cy();

			param.sme_param.calib_right.baseline = param.sme_param.calib_left.baseline;
			param.sme_param.calib_right.fx = model_right.fx();
			param.sme_param.calib_right.fy = model_right.fy();
			ROS_ASSERT( std::abs((model_right.fx() - model_right.fy()) / model_right.fy()) < 1e-2 ); // difference between fx and fy should be small
			param.sme_param.calib_right.cu = model_right.cx();
			param.sme_param.calib_right.cv = model_right.cy();

			const double &baseline = param.sme_param.calib_left.baseline;
			ROS_ASSERT(std::abs(- model_right.Ty() / model_right.fy() / baseline ) < 1e-3); // Ty for right should be negligible
			ROS_ASSERT(std::abs(- right_Tz / baseline ) < 1e-3); // Tz for right should be negligible

			// Set epipolar offset for Quadmatcher
			param.qm_param.epipolar_offset = - model_left.cy() + (-model_right.Ty()) + model_right.cy() ;
			ROS_INFO_STREAM("epipolar_offset = " << param.qm_param.epipolar_offset);

			ROS_WARN("Using DIFFERENT calibrations for left and right image sensors.");
			ROS_INFO_STREAM("Right Camera (scaled): Tx() = " << model_right.Tx() << ", Ty() = " << model_right.Ty() << ", Tz() = " << right_Tz);
			ROS_INFO_STREAM("Right Camera (in meter): Tx() = " << - model_right.Tx() / model_right.fx() << ", Ty() = " << - model_right.Ty() / model_right.fy()  << ", Tz() = " << -right_Tz);
		}

		viso2->setParam(l_info_msg->width, l_info_msg->height, param.qm_param, param.sme_param);
		//if (l_info_msg->header.frame_id != "") setSensorFrameId(l_info_msg->header.frame_id);
		ROS_INFO_STREAM("Initialized libviso2 stereo odometry with the following parameters:" << std::endl << param );
	}

	void resetCallback(const std_msgs::HeaderPtr &header){
		ROS_WARN_STREAM("Reset occurs at " << header->stamp );

		in_process.lock();
		resetPose(header->stamp.toNSec());
		voState = VOState();
		voState.last_frame_time_ns = header->stamp.toNSec();
		voState.tfStamped.push_back(TfStamped());
		in_process.unlock();
	}
	
	bool disable_debug = false;
	void imageCallback(
			const sensor_msgs::ImageConstPtr l_image_msg,
			const sensor_msgs::ImageConstPtr r_image_msg,
			const sensor_msgs::CameraInfoConstPtr l_info_msg,
			const sensor_msgs::CameraInfoConstPtr r_info_msg)
	{

		if (disable_debug) return;

		std::cout << "\n\nimageCallback(): frame at " << l_image_msg->header.stamp << std::endl;

		if ( l_image_msg->header.stamp.toNSec() <= voState.last_frame_time_ns)
		{
			ROS_WARN_STREAM_THROTTLE(1,"Non-progressive frame detected in viso2 imageCallback()");
			return;
		}

		in_process.lock();

		// ros::WallTime start_time = ros::WallTime::now();

		// static cv::Mat cv_leftImg_prev;
		// static cv::Mat cv_rightImg_prev;

		bool first_run = !viso2->isInitialised();
		// create odometer if not exists
		if (first_run)
		{	
			initOdometer(l_info_msg, r_info_msg);
			assert(voState.tfStamped.empty());
			voState.tfStamped.push_back(TfStamped());
		}
			

		// convert to cvImage bridge
		auto cvImage_l = cv_bridge::toCvShare(l_image_msg, sensor_msgs::image_encodings::MONO8); // CvImageConstPtr 
		auto cvImage_r = cv_bridge::toCvShare(r_image_msg, sensor_msgs::image_encodings::MONO8);


		cv::Mat cv_leftImg_source = cvImage_l->image;
		cv::Mat cv_rightImg_source = cvImage_r->image;

		// outImg = cv_leftImg_source.clone();
		
		// hm: only keep two records
		if (voState.tfStamped.size() == 3)
			voState.tfStamped.erase(voState.tfStamped.begin());

		const int state_idx = voState.tfStamped.size() - 1;
		const int state_idx_next = voState.tfStamped.size();

		voState.tfStamped.push_back(TfStamped()); // preallocate the next state, no matter failure

		// hm: for the moment we only keep the recent two record
		assert (state_idx <= 1);

		//////////////////////////////////////
		//// STEP 1: Push Back Data, Quad Matching, Motion Estimation
		//////////////////////////////////////

		Viso2Eigen::Mode mode;
		if (first_run || voState.tfStamped[state_idx-1].lost == true)
			mode = Viso2Eigen::Mode::FIRST_FRAME;
		else if (voState.tfStamped[state_idx].use_old_reference_frame)
			mode = Viso2Eigen::Mode::USE_OLD_FRAME;
		else
			mode = Viso2Eigen::Mode::NO_USE_OLD_FRAME;

		////////////// CHECK FOR JITTER /////////////////////
		static ros::Time last_frame_time = ros::Time(0);
		static double avg_time_gap = 0;

		if (!last_frame_time.isZero()){
			double delta_t = (cvImage_l->header.stamp - last_frame_time).toSec();
			if (avg_time_gap == 0)
				avg_time_gap = delta_t;
				

			double jitter = delta_t - avg_time_gap;

			if (delta_t > 0)
			{
				if (std::abs(jitter/avg_time_gap) > 0.3){
					ROS_ERROR_STREAM("Jitter Detected: average_gap=" << avg_time_gap << ", but current=" << delta_t);

					if (param.skip_update_upon_jitter)
					{
						mode = Viso2Eigen::Mode::FIRST_FRAME;
						ROS_ERROR("Clear Viso2 Process, and restart from the first frame...");
					}
						
					// avg_time_gap = avg_time_gap*0.98 + delta_t*0.02;

					if (jitter / avg_time_gap < -0.3){
						ROS_WARN("Resetting average_gap");
						avg_time_gap = 0;
					}
				}else
					avg_time_gap = avg_time_gap*0.95 + delta_t*0.05;
			}
			
		}

		last_frame_time = cvImage_l->header.stamp;
		/////////////////////////////////////////////////////
		
		if (mode != Viso2Eigen::Mode::FIRST_FRAME) profiler.profile("viso->process");
		bool viso2_result = viso2->process(cv_leftImg_source, cv_rightImg_source, mode, cvImage_l->header.stamp.toNSec() );
		if (mode != Viso2Eigen::Mode::FIRST_FRAME) profiler.stop();

		// std::cout <<"viso2->process()"<<std::endl;

		voState.last_frame_time_ns = cvImage_l->header.stamp.toNSec(); // must be updated after process()

		cv::Mat outImg, outImg_right;
		if (mode == Viso2Eigen::Mode::FIRST_FRAME)
		{
			assert(viso2_result);
			voState.tfStamped[state_idx].stamp = cvImage_l->header.stamp.toNSec();
			std::cerr << "First frame process() SUCCESS" << std::endl;

			// we still want visualisation
			if (param.visualisation_on)
			{
				getVisualisation(outImg, outImg_right);
				publishDebugImg(outImg,outImg_right,l_info_msg,r_info_msg,cvImage_l->header.stamp);
			}
			
			voState.reference_motion = Eigen::Affine3d::Identity();
			in_process.unlock();
			return;
		}

		//////////////////////
		//// Quick sanity check
		//////////////////////

		if (viso2_result){
			Eigen::Affine3d tf = voState.reference_motion.inverse() * viso2->getCameraMotion();
			Eigen::Vector3d origin = {0,0,0};
			origin = tf * origin;
			if (avg_time_gap > 0 && origin.norm() > (10.0 / avg_time_gap) ){
				ROS_WARN_STREAM("Translation delta too big : " << origin.transpose() );
				ROS_WARN_STREAM(tf.matrix());
				viso2_result = false;
			}

			Eigen::AngleAxis<double> motion_aa(tf.rotation());
			if (std::abs(motion_aa.angle())/M_PI*180.0 > 60){
				ROS_WARN_STREAM("Angle delta too big : " << motion_aa.angle() );
				viso2_result = false;
			}
		}
		
		if (viso2_result){
			
			Eigen::Affine3d cameraMotion = viso2->getCameraMotion();
			

		//////////////////////////////////////
		//// STEP 2: Obtain Camera Motion Delta
		//////////////////////////////////////

			voState.tfStamped[state_idx].tf = voState.reference_motion.inverse() * cameraMotion;
			voState.tfStamped[state_idx].stamp = cvImage_l->header.stamp.toNSec();
			voState.tfStamped[state_idx].lost = false;

		//////////////////////////////////////
		//// STEP 3: Obtain use_old_reference_frame value for the next frame
		//////////////////////////////////////

		const std::vector<int> inliers = viso2->getInlier();
		double I = inliers.size();

		// compute optical flow and decide the next frame's use_old_reference_frame value

		voState.tfStamped[state_idx_next].use_old_reference_frame = false;
		voState.reference_motion = Eigen::Affine3d::Identity();

		if (I > param.ref_frame_inlier_threshold)
		{
			double opticalFlow = viso2->computeOpticalFlow(); // average of highest and average flow

			std::cout  << "flow: " << opticalFlow << std::endl;

			assert (voState.tfStamped.size() == state_idx_next + 1);

			if ( opticalFlow < param.ref_frame_motion_threshold * param.ref_frame_motion_threshold )
			{
				voState.tfStamped[state_idx_next].use_old_reference_frame = true;
				voState.reference_motion = cameraMotion;
			}
		}
		
		//////////////////////////////////////
		//// STEP 4: Calculate Variance
		//////////////////////////////////////


		int matched_size = viso2->getMatchedSize();
		double confidence = viso2->getConfidence();

		double delta_t = (voState.tfStamped[state_idx].stamp - voState.tfStamped[state_idx-1].stamp)/1.0e9;

		double variance;
		if (I <= 3 || confidence == 0.0)
			variance = 9999;
		else
		 	variance = param.variance_scale * std::sqrt(delta_t * 100 /*benchmark 100 Hz*/) / std::pow(confidence,3) ;

		// variance = std::max(0.005, variance);

		std::cout << "variance = " << variance << std::endl;

		//////////////////////////////////////
		//// STEP 5: Publish to ROS
		//////////////////////////////////////

			setPoseCovariance(variance,0);

			// std::cout << "Frame: " << state_idx << " stamp: " << voState.tfStamped[state_idx].stamp << std::endl; 
			std::cout << "tf:\n" << voState.tfStamped[state_idx].tf.matrix() << std::endl;
			std::cout << (voState.tfStamped[state_idx_next].use_old_reference_frame ? "use_old_frame next" : "replace_frame next") << std::endl;

			integrateAndPublish(voState.tfStamped[state_idx].tf, voState.tfStamped[state_idx].stamp, voState.tfStamped[state_idx-1].stamp);
		
		//////////////////////////////////////
		//// STEP 6: Publish Visualisation
		//////////////////////////////////////
		

		}else{
			std::cerr << "process() FAILED" << std::endl;
			voState.tfStamped[state_idx_next].use_old_reference_frame = false;
			if (viso2->isCurrentFrameFeatureValid()){
				std::cout << "Current Frame Feature Extraction is valid." << std::endl;
				voState.tfStamped[state_idx].stamp = cvImage_l->header.stamp.toNSec();
				voState.tfStamped[state_idx].tf = Eigen::Affine3d::Identity();
			}else
				voState.tfStamped[state_idx].lost = true;
			// possible bug? must add the line below
			voState.reference_motion = Eigen::Affine3d::Identity();
			
			outImg = cv_leftImg_source;
			outImg_right = cv_rightImg_source;

			disable_debug = false;
		}

		if (param.visualisation_on)
		{
			getVisualisation(outImg, outImg_right);
			publishDebugImg(outImg,outImg_right,l_info_msg,r_info_msg,cvImage_l->header.stamp);
		}
		in_process.unlock();
	}

	

	// double computeFeatureFlow(
	// 		const std::vector<Matcher::p_match>& matches)
	// {
	// 	double total_flow = 0.0;
	// 	for (size_t i = 0; i < matches.size(); ++i)
	// 	{
	// 		double x_diff = matches[i].u1c - matches[i].u1p;
	// 		double y_diff = matches[i].v1c - matches[i].v1p;
	// 		total_flow += sqrt(x_diff * x_diff + y_diff * y_diff);
	// 	}
	// 	return total_flow / matches.size();
	// }
};


} // end of namespace


int main(int argc, char **argv)
{
	// std::string cv_window_name_left = "Current Frame Left";
	// std::string cv_window_name_right = "Current Frame Right";
	// cv::namedWindow(cv_window_name_left, cv::WINDOW_AUTOSIZE);
	// cv::namedWindow(cv_window_name_right, cv::WINDOW_AUTOSIZE);


	// ROS Initialisation
	ros::init(argc, argv, "stereo_odometer");
	ros::NodeHandle nh;
	ros::NodeHandle local_nh("~");

	

	// Loading Parameters except calibration details
	viso2_ros::Parameters param;
	viso2_ros::loadParams(local_nh, param);

	std::string transport = argc > 1 ? argv[1] : "raw";
	viso2_ros::StereoOdometer odometer(transport, param);

	uint64_t seq_showed = 0;

	ros::AsyncSpinner spinner(2);
	spinner.start();

	ros::waitForShutdown(); // ros::spin();
	
	return 0;
}
