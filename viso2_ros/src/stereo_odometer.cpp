#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <image_geometry/stereo_camera_model.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <viso_stereo.h>

#include <viso2_ros/VisoInfo.h> // message file header

#include "stereo_processor.h"
#include "odometer_base.h"
#include "odometry_params.h"

// to remove after debugging
#include <opencv2/opencv.hpp>

#include <iostream>

namespace viso2_ros
{

std::string cv_window_name = "stereo_odometer";



cv::Mat cv_drawMatches(std::string cv_window_name, cv::Mat cv_leftImg, 
			cv::Mat cv_rightImg, std::vector<Matcher::p_match> _matches, std::vector<int> _inlierIdx);

cv::Mat correctGamma( cv::Mat& img, double gamma );

// some arbitrary values (0.1m^2 linear cov. 10deg^2. angular cov.)
static const boost::array<double, 36> STANDARD_POSE_COVARIANCE =
{ { 0.1, 0, 0, 0, 0, 0,
		0, 0.1, 0, 0, 0, 0,
		0, 0, 0.1, 0, 0, 0,
		0, 0, 0, 0.17, 0, 0,
		0, 0, 0, 0, 0.17, 0,
		0, 0, 0, 0, 0, 0.17 } };
static const boost::array<double, 36> STANDARD_TWIST_COVARIANCE =
{ { 0.05, 0, 0, 0, 0, 0,
		0, 0.05, 0, 0, 0, 0,
		0, 0, 0.05, 0, 0, 0,
		0, 0, 0, 0.09, 0, 0,
		0, 0, 0, 0, 0.09, 0,
		0, 0, 0, 0, 0, 0.09 } };
static const boost::array<double, 36> BAD_COVARIANCE =
{ { 9999, 0, 0, 0, 0, 0,
		0, 9999, 0, 0, 0, 0,
		0, 0, 9999, 0, 0, 0,
		0, 0, 0, 9999, 0, 0,
		0, 0, 0, 0, 9999, 0,
		0, 0, 0, 0, 0, 9999 } };

// StereoProcessor exposes the imageCallback() for synced image and camera info (subscribers)
// OdometerBase provides publisher to odometry and pose topic  integrateAndPublish()
class StereoOdometer : public StereoProcessor, public OdometerBase
{

private:

	boost::shared_ptr<VisualOdometryStereo> visual_odometer_;
	VisualOdometryStereo::parameters visual_odometer_params_;

	ros::Publisher point_cloud_pub_;
	ros::Publisher info_pub_;

	bool got_lost_;

	// change reference frame method. 0, 1 or 2. 0 means allways change. 1 and 2 explained below
	int ref_frame_change_method_;
	bool change_reference_frame_;
	double ref_frame_motion_threshold_; // method 1. Change the reference frame if last motion is small
	int ref_frame_inlier_threshold_; // method 2. Change the reference frame if the number of inliers is low
	Matrix reference_motion_;

public:

	typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

	StereoOdometer(const std::string& transport) : 
		StereoProcessor(transport), OdometerBase(), 
		got_lost_(false), change_reference_frame_(false)
	{
		// Read local parameters
		ros::NodeHandle local_nh("~");
		odometry_params::loadParams(local_nh, visual_odometer_params_);

		local_nh.param("ref_frame_change_method", ref_frame_change_method_, 0);
		local_nh.param("ref_frame_motion_threshold", ref_frame_motion_threshold_, 5.0);
		local_nh.param("ref_frame_inlier_threshold", ref_frame_inlier_threshold_, 150);

		point_cloud_pub_ = local_nh.advertise<PointCloud>("point_cloud", 1);
		info_pub_ = local_nh.advertise<VisoInfo>("info", 1);

		reference_motion_ = Matrix::eye(4);
	}

protected:

	void initOdometer(
			const sensor_msgs::CameraInfoConstPtr& l_info_msg,
			const sensor_msgs::CameraInfoConstPtr& r_info_msg)
	{
		// read calibration info from camera info message
		// to fill remaining parameters
		image_geometry::StereoCameraModel model;
		model.fromCameraInfo(*l_info_msg, *r_info_msg); // http://wiki.ros.org/image_pipeline/CameraInfo

		visual_odometer_params_.base = model.baseline(); // calculated as -right_.Tx() / right_.fx(); fx() is defined in pixels. baseline is in meter
		// Note: the Tx() in ROS is defined as   -(baseline (meter) * focal length (pixel)), not just baseline alone!
		// https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
		visual_odometer_params_.calib.f = model.left().fx(); // in pixels
		visual_odometer_params_.calib.cu = model.left().cx();
		visual_odometer_params_.calib.cv = model.left().cy();

		visual_odometer_params_.bucket.bucket_ornot = true;
		visual_odometer_params_.match.numFastFeature_dense = 1000;
		visual_odometer_params_.match.match_disp_tolerance = 6; // default 2
		visual_odometer_params_.match.outlier_disp_tolerance = 10;
		visual_odometer_params_.match.remove_outliers = false;
		//visual_odometer_params_.match.match_radius = 300;
		visual_odometer_params_.match.fast_threshold_dense = 8;


	ROS_INFO_STREAM("Initialized libviso2 stereo odometry "
										"with the following calibrations:" << std::endl << 
					"baseline = " << visual_odometer_params_.base << std::endl <<
					"focal length = " << visual_odometer_params_.calib.f << std::endl <<
					"optional centre (x) = " << visual_odometer_params_.calib.cu << std::endl <<
					"optional centre (y = " << visual_odometer_params_.calib.cv
										);
		
		visual_odometer_.reset(new VisualOdometryStereo(visual_odometer_params_));
		if (l_info_msg->header.frame_id != "") setSensorFrameId(l_info_msg->header.frame_id);
		ROS_INFO_STREAM("Initialized libviso2 stereo odometry "
										"with the following parameters:" << std::endl << 
										visual_odometer_params_ << 
										"  ref_frame_change_method = " << ref_frame_change_method_ << std::endl << 
										"  ref_frame_motion_threshold = " << ref_frame_motion_threshold_ << std::endl << 
										"  ref_frame_inlier_threshold = " << ref_frame_inlier_threshold_);
	}
 
	void imageCallback(
			const sensor_msgs::ImageConstPtr& l_image_msg,
			const sensor_msgs::ImageConstPtr& r_image_msg,
			const sensor_msgs::CameraInfoConstPtr& l_info_msg,
			const sensor_msgs::CameraInfoConstPtr& r_info_msg)
	{
		ros::WallTime start_time = ros::WallTime::now();
		bool first_run = false;
		// create odometer if not exists
		if (!visual_odometer_)
		{
			first_run = true;
			initOdometer(l_info_msg, r_info_msg);
		}

		// convert images if necessary
		uint8_t *l_image_data, *r_image_data;
		int l_step, r_step;
		cv_bridge::CvImageConstPtr l_cv_ptr, r_cv_ptr;
		l_cv_ptr = cv_bridge::toCvShare(l_image_msg, sensor_msgs::image_encodings::MONO8);
		l_image_data = l_cv_ptr->image.data;
		l_step = l_cv_ptr->image.step[0];
		r_cv_ptr = cv_bridge::toCvShare(r_image_msg, sensor_msgs::image_encodings::MONO8);
		r_image_data = r_cv_ptr->image.data;
		r_step = r_cv_ptr->image.step[0];

		ROS_ASSERT(l_step == r_step);
		ROS_ASSERT(l_image_msg->width == r_image_msg->width);
		ROS_ASSERT(l_image_msg->height == r_image_msg->height);

		int32_t dims[] = {l_image_msg->width, l_image_msg->height, l_step};

		cv::Mat cv_leftImg(cv::Size(dims[0], dims[1]), CV_8UC1, (void *)l_image_data, cv::Mat::AUTO_STEP);
		cv::Mat cv_rightImg(cv::Size(dims[0], dims[1]), CV_8UC1, (void *)r_image_data, cv::Mat::AUTO_STEP);

		cv_leftImg = correctGamma(cv_leftImg,1.2);
		cv_rightImg = correctGamma(cv_rightImg,1.2);
		double a = 1.25;
		double b = 128 * (1 - a) + 10;
		cv_leftImg.convertTo(cv_leftImg,-1,a,b);
		cv_rightImg.convertTo(cv_rightImg,-1,a,b);

		cv::Mat blur_left(cv::Size(dims[0], dims[1]), CV_8UC1);
		cv::Mat blur_right(cv::Size(dims[0], dims[1]), CV_8UC1);

		cv::GaussianBlur(cv_leftImg,blur_left,cv::Size(0,0),3);
		cv::GaussianBlur(cv_rightImg,blur_right,cv::Size(0,0),3);

		cv::addWeighted(cv_leftImg, 1.5, blur_left, -0.5, 0, cv_leftImg);
		cv::addWeighted(cv_rightImg, 1.5, blur_right, -0.5, 0, cv_rightImg);

		// cv::fastNlMeansDenoising(cv_leftImg,cv_leftImg);
		// cv::fastNlMeansDenoising(cv_rightImg,cv_rightImg);

		// auto clache = cv::createCLAHE(5,cv::Size(6,8));
		// clache->apply(cv_leftImg,cv_leftImg);
		// clache->apply(cv_rightImg,cv_rightImg);

		// cv::equalizeHist(cv_leftImg,cv_leftImg);
		// cv::equalizeHist(cv_rightImg,cv_rightImg);

		// cv::Mat leftbuffer(cv::Size(dims[0], dims[1]), CV_8UC1);
		// cv::Mat rightbuffer(cv::Size(dims[0], dims[1]), CV_8UC1);

		// clache->apply(cv_leftImg,leftbuffer);
		// clache->apply(cv_rightImg,rightbuffer);
		
		// cv::bilateralFilter(leftbuffer,cv_leftImg,10,5,3);
		// cv::bilateralFilter(rightbuffer,cv_rightImg,10,5,3);

		// on first run or when odometer got lost, only feed the odometer with 
		// images without retrieving data
		if (first_run ) // || got_lost_
		{
			visual_odometer_->process(cv_leftImg.data, cv_rightImg.data, dims);
			got_lost_ = false;
			// on first run publish zero once
			if (first_run)
			{
				tf::Transform delta_transform;
				delta_transform.setIdentity();
				integrateAndPublish(delta_transform, l_image_msg->header.stamp);
			}
		}
		else
		{
			//ROS_INFO_STREAM("Processing frame " << l_info_msg->header.seq);
			bool success = visual_odometer_->process(
					cv_leftImg.data, cv_rightImg.data, dims, change_reference_frame_); // change_reference_frame_ == replace, whereby previous frame holds the same, only current frame changes
			
			
			std::vector<Matcher::p_match> _matches = visual_odometer_->getMatches();
			auto _inlierIdx = visual_odometer_->getInlierIndices();

			cv::Mat outImg = cv_drawMatches(cv_window_name, cv_leftImg, cv_rightImg, _matches, _inlierIdx);

			auto num_matches = visual_odometer_->getNumberOfMatches();
			auto num_inliers = visual_odometer_->getNumberOfInliers();

			std::cout << "Matches: " << num_matches;
			std::cout << ", Inliers: " << 100.0*num_inliers/num_matches << " %" << std::endl;

			ROS_DEBUG("Found %i matches with %i inliers.", 
									visual_odometer_->getNumberOfMatches(),
									visual_odometer_->getNumberOfInliers());
				
			if (success)
			{
				Matrix motion = Matrix::inv(visual_odometer_->getMotion()); // getMotion is a change-of-frame transformation from t-1 to t
				// therefore, motion is the active transformation from t-1 to t;
				ROS_DEBUG_STREAM("libviso2 returned the following motion:\n" << motion);
				Matrix camera_motion; // active transformation of camera
				// if image was replaced due to small motion we have to subtract the 
				// last motion to get the increment
				if (change_reference_frame_) // HM: in this case, previous frame was kept, therefore use the old reference_motion_ to subtract
				{
					camera_motion = Matrix::inv(reference_motion_) * motion; // actual per-frame motion = reference^(-1) * measured motion
				}
				else
				{
					// image was not replaced, report full motion from odometer
					camera_motion = motion;
				}
				reference_motion_ = motion; // store last motion as reference

				tf::Matrix3x3 rot_mat(
					camera_motion.val[0][0], camera_motion.val[0][1], camera_motion.val[0][2],
					camera_motion.val[1][0], camera_motion.val[1][1], camera_motion.val[1][2],
					camera_motion.val[2][0], camera_motion.val[2][1], camera_motion.val[2][2]);
				tf::Vector3 t(camera_motion.val[0][3], camera_motion.val[1][3], camera_motion.val[2][3]);
				tf::Transform delta_transform(rot_mat, t);

				setPoseCovariance(STANDARD_POSE_COVARIANCE);
				setTwistCovariance(STANDARD_TWIST_COVARIANCE);

				integrateAndPublish(delta_transform, l_image_msg->header.stamp);

				if (point_cloud_pub_.getNumSubscribers() > 0)
				{
					cout << "Publishing PointCloud..." << endl;
					computeAndPublishPointCloud(l_info_msg, l_image_msg, r_info_msg, 
																			visual_odometer_->getMatches(), 
																			visual_odometer_->getInlierIndices());
				}
			}
			else
			{
				setPoseCovariance(BAD_COVARIANCE);
				setTwistCovariance(BAD_COVARIANCE);
				tf::Transform delta_transform;
				delta_transform.setIdentity();
				integrateAndPublish(delta_transform, l_image_msg->header.stamp);

				ROS_DEBUG("Call to VisualOdometryStereo::process() failed.");
				ROS_WARN("Visual Odometer got lost!");
				got_lost_ = true;
			}

			if(success)
			{

				// Proceed depending on the reference frame change method
				switch ( ref_frame_change_method_ )
				{
					case 1:
					{
						// calculate current feature flow
						double feature_flow = computeFeatureFlow(visual_odometer_->getMatches());
						change_reference_frame_ = (feature_flow < ref_frame_motion_threshold_);
						ROS_DEBUG_STREAM("Feature flow is " << feature_flow 
								<< ", marking last motion as " 
								<< (change_reference_frame_ ? "small." : "normal."));
						break;
					}
					case 2:
					{
						change_reference_frame_ = (visual_odometer_->getNumberOfInliers() > ref_frame_inlier_threshold_);
						break;
					}            
					default:
						change_reference_frame_ = false;
				}
				
			}
			else
				change_reference_frame_ = false;

			if(!change_reference_frame_)
				ROS_DEBUG_STREAM("Changing reference frame");

			// create and publish viso2 info msg
			VisoInfo info_msg;
			info_msg.header.stamp = l_image_msg->header.stamp;
			info_msg.got_lost = !success;
			info_msg.change_reference_frame = !change_reference_frame_; // !replace
			info_msg.num_matches = visual_odometer_->getNumberOfMatches();
			info_msg.num_inliers = visual_odometer_->getNumberOfInliers();
			ros::WallDuration time_elapsed = ros::WallTime::now() - start_time;
			info_msg.runtime = time_elapsed.toSec();
			info_pub_.publish(info_msg);
		}
	}

	double computeFeatureFlow(
			const std::vector<Matcher::p_match>& matches)
	{
		double total_flow = 0.0;
		for (size_t i = 0; i < matches.size(); ++i)
		{
			double x_diff = matches[i].u1c - matches[i].u1p;
			double y_diff = matches[i].v1c - matches[i].v1p;
			total_flow += sqrt(x_diff * x_diff + y_diff * y_diff);
		}
		return total_flow / matches.size();
	}

	void computeAndPublishPointCloud(
			const sensor_msgs::CameraInfoConstPtr& l_info_msg,
			const sensor_msgs::ImageConstPtr& l_image_msg,
			const sensor_msgs::CameraInfoConstPtr& r_info_msg, 
			const std::vector<Matcher::p_match>& matches,
			const std::vector<int32_t>& inlier_indices)
	{
		try
		{
			cv_bridge::CvImageConstPtr cv_ptr;
			cv_ptr = cv_bridge::toCvShare(l_image_msg, sensor_msgs::image_encodings::RGB8);
			// read calibration info from camera info message
			image_geometry::StereoCameraModel model;
			model.fromCameraInfo(*l_info_msg, *r_info_msg);
			PointCloud::Ptr point_cloud(new PointCloud());
			point_cloud->header.frame_id = getSensorFrameId();
			point_cloud->header.stamp = pcl_conversions::toPCL(l_info_msg->header).stamp;
			point_cloud->width = 1;
			point_cloud->height = inlier_indices.size();
			point_cloud->points.resize(inlier_indices.size());

			for (size_t i = 0; i < inlier_indices.size(); ++i)
			{
				const Matcher::p_match& match = matches[inlier_indices[i]];
				cv::Point2d left_uv;
				left_uv.x = match.u1c;
				left_uv.y = match.v1c;
				cv::Point3d point;
				double disparity = match.u1c - match.u2c;
				model.projectDisparityTo3d(left_uv, disparity, point);
				point_cloud->points[i].x = point.x;
				point_cloud->points[i].y = point.y;
				point_cloud->points[i].z = point.z;
				cv::Vec3b color = cv_ptr->image.at<cv::Vec3b>(left_uv.y,left_uv.x);
				point_cloud->points[i].r = color[0];
				point_cloud->points[i].g = color[1];
				point_cloud->points[i].b = color[2];
			}
			ROS_DEBUG("Publishing point cloud with %zu points.", point_cloud->size());
			point_cloud_pub_.publish(point_cloud);
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
		}
	}
};

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

cv::Mat correctGamma( cv::Mat& img, double gamma ) {
 double inverse_gamma = 1.0 / gamma;
 
 cv::Mat lut_matrix(1, 256, CV_8UC1 );
 uchar * ptr = lut_matrix.ptr();
 for( int i = 0; i < 256; i++ )
   ptr[i] = (int)( pow( (double) i / 255.0, inverse_gamma ) * 255.0 );
 
 cv::Mat result;
 LUT( img, lut_matrix, result );
 
 return result;
}

} // end of namespace


int main(int argc, char **argv)
{
	cv::namedWindow(viso2_ros::cv_window_name, cv::WINDOW_AUTOSIZE);
	ros::init(argc, argv, "stereo_odometer");
	//   if (ros::names::remap("stereo") == "stereo") {
	//     ROS_WARN("'stereo' has not been remapped! Example command-line usage:\n"
	//              "\t$ rosrun viso2_ros stereo_odometer stereo:=narrow_stereo image:=image_rect");
	//   }
	// if (ros::names::remap("image").find("rect") == std::string::npos) {
	//   ROS_WARN("stereo_odometer needs rectified input images. The used image "
	//            "topic is '%s'. Are you sure the images are rectified?",
	//            ros::names::remap("image").c_str());
	// }

	std::string transport = argc > 1 ? argv[1] : "raw";
	viso2_ros::StereoOdometer odometer(transport);
	
	ros::spin();
	return 0;
}
