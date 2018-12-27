#ifndef ODOMETRY_PARAMS_H
#define ODOMETRY_PARAMS_H

#include <ros/ros.h>
#include <viso2_eigen/viso2_eigen.h>


namespace viso2_ros
{
	enum FrameChangeMethod {
			FLOW,
			INLIER
		};
	
	struct Parameters{
		QuadMatcherParam::Parameters qm_param;
		StereoMotionEstimatorParam::Parameters sme_param;

		

		FrameChangeMethod ref_frame_change_method = FLOW; // change reference frame method. 0, 1 or 2. 0 means allways change. 1 and 2 explained below
		double ref_frame_motion_threshold; // method 1. Change the reference frame if last motion is small
		int ref_frame_inlier_threshold; // method 2. Change the reference frame if the number of inliers is low

		// double image_pre_scaling;
		// double image_pre_contrast;
		// double image_pre_brightness;

		double std_tCov;
		double std_rCov;
		// std::string moving_object_topics;

		//// Stereo Processor
		int queue_size;

		std::string left_topic, right_topic;
		std::string left_info_topic, right_info_topic;

		double variance_scale;

		//// Main()
		bool visualisation_on;
	};
	/// loads matcher params
	void loadParams(const ros::NodeHandle& local_nh, QuadMatcherParam::Parameters& params)
	{
		// local_nh.getParam("image_width",          			params.image_width);
		// local_nh.getParam("image_height",          			params.image_height);
		// local_nh.getParam("match_radius",           		params.match_radius);
		ROS_ASSERT(local_nh.getParam("number_of_buckets_in_width",   	params.n_bucket_width));
		ROS_ASSERT(local_nh.getParam("number_of_buckets_in_height", 	params.n_bucket_height));
		ROS_ASSERT(local_nh.getParam("epipolar_tolerance", 				params.epipolar_tolerance));
		ROS_ASSERT(local_nh.getParam("max_neighbor_ratio",            	params.max_neighbor_ratio));
		ROS_ASSERT(local_nh.getParam("use_bucketing",        			params.use_bucketing));
		ROS_ASSERT(local_nh.getParam("max_features_per_bucket",        			params.max_features_per_bucket));
		ROS_ASSERT(local_nh.getParam("compulte_scaled_keys",        	params.compulte_scaled_keys));
	}

	/// loads common & stereo specific params
	void loadParams(const ros::NodeHandle& local_nh, StereoMotionEstimatorParam::Parameters& params)
	{
		// loadParams(local_nh, params.match);
		ROS_ASSERT(local_nh.getParam("ransac_iters", 						params.ransac_iters));
		ROS_ASSERT(local_nh.getParam("inlier_threshold",      				params.inlier_threshold));
		ROS_ASSERT(local_nh.getParam("inlier_ratio_min",      				params.inlier_ratio_min));
		ROS_ASSERT(local_nh.getParam("reweighting",      					params.reweighting));
		ROS_ASSERT(local_nh.getParam("good_point_threshold_scale",      	params.good_point_threshold_scale));
		
		// local_nh.getParam("image_width",      				params.image_width);
		// local_nh.getParam("image_height",      				params.image_height);
		// Calibration
		// local_nh.getParam("baseline",     					params.baseline);
		// local_nh.getParam("calib_f",      					params.calib.f);
		// local_nh.getParam("calib_cu",      					params.calib.cu);
		// local_nh.getParam("calib_cv",      					params.calib.cv);
	}

	/// loads common & stereo specific params
	void loadParams(const ros::NodeHandle& local_nh, Parameters& params)
	{
		loadParams(local_nh, params.qm_param);
		loadParams(local_nh, params.sme_param);

		std::string str;
		ROS_ASSERT(local_nh.getParam("ref_frame_change_method", 		str));
		if (str == "flow")
			params.ref_frame_change_method = FLOW;
		else if (str == "inlier")
			params.ref_frame_change_method = INLIER;
		else
		{
			ROS_FATAL("ref_frame_change_method is not defined properly.");
			exit(-1);
		}
		
		ROS_ASSERT(local_nh.getParam("ref_frame_motion_threshold", 		params.ref_frame_motion_threshold));
		ROS_ASSERT(local_nh.getParam("ref_frame_inlier_threshold", 		params.ref_frame_inlier_threshold));
		// ROS_ASSERT(local_nh.getParam("image_pre_scaling", 				params.image_pre_scaling));
		// ROS_ASSERT(local_nh.getParam("image_pre_contrast", 				params.image_pre_contrast));
		// ROS_ASSERT(local_nh.getParam("image_pre_brightness", 			params.image_pre_brightness));

		ROS_ASSERT(local_nh.getParam("noise_translation", 				params.std_tCov));
		ROS_ASSERT(local_nh.getParam("noise_rotation", 					params.std_rCov));
		// ROS_ASSERT(local_nh.getParam("moving_object_polygons", 			params.moving_object_topics));

		//// Stereo Processor
		ROS_ASSERT(local_nh.getParam("queue_size", 						params.queue_size));
		ROS_ASSERT(local_nh.getParam("left_topic", 						params.left_topic));
		ROS_ASSERT(local_nh.getParam("right_topic", 					params.right_topic));
		ROS_ASSERT(local_nh.getParam("left_info_topic", 				params.left_info_topic));
		ROS_ASSERT(local_nh.getParam("right_info_topic", 				params.right_info_topic));

		ROS_ASSERT(local_nh.getParam("variance_scale", 				params.variance_scale));

		//// Main()
		ROS_ASSERT(local_nh.getParam("visualisation_on", 				params.visualisation_on));
		
	}



	

	std::ostream& operator<<(std::ostream& out, const QuadMatcherParam::Parameters& params)
	{
		out << "QuadMatcherParam parameters:" << std::endl;
		out << "  image_width                   = " << params.image_width << std::endl;
		out << "  image_height                  = " << params.image_height << std::endl;
		out << "  number_of_buckets_in_width    = " << params.n_bucket_width << std::endl;
		out << "  number_of_buckets_in_height   = " << params.n_bucket_height << std::endl;
		out << "  epipolar_tolerance            = " << params.epipolar_tolerance << std::endl;
		out << "  max_neighbor_ratio            = " << params.max_neighbor_ratio << std::endl;
		out << "  use_bucketing                 = " << params.use_bucketing << std::endl;
		out << "  compulte_scaled_keys          = " << params.compulte_scaled_keys << std::endl;
		return out;
	}

	std::ostream& operator<<(std::ostream& out, const StereoMotionEstimatorParam::Parameters& params)
	{
		out << "StereoMotionEstimatorParam parameters:" << std::endl;
		
		out << "  ransac_iters          = " << params.ransac_iters << std::endl;
		out << "  inlier_threshold      = " << params.inlier_threshold << std::endl;
		out << "  inlier_ratio_min      = " << params.inlier_ratio_min << std::endl;
		out << "  reweighting           = " << params.reweighting << std::endl;
		out << "  image_width           = " << params.image_width << std::endl;
		out << "  image_height          = " << params.image_height << std::endl;

		out << "  good_point_thres_scale= " << params.good_point_threshold_scale << std::endl;

		out << "  baseline              = " << params.calib.baseline << std::endl;
		out << "  calib_f               = " << params.calib.f << std::endl;
		out << "  calib_cu              = " << params.calib.cu << std::endl;
		out << "  calib_cv              = " << params.calib.cv << std::endl;
		return out;
	}

	std::ostream& operator<<(std::ostream& out, const Parameters& params)
	{
		out << params.qm_param;
		out << params.sme_param;
		out << "Stereo Odometer (Main) parameters:" << std::endl;
		out << "  ref_frame_change_method       = " << (params.ref_frame_change_method == FLOW ? "FLOW" : "INLIER") << std::endl;
		out << "  ref_frame_motion_threshold    = " << params.ref_frame_motion_threshold << std::endl;
		out << "  ref_frame_inlier_threshold    = " << params.ref_frame_inlier_threshold << std::endl;
		// out << "  image_pre_scaling             = " << params.image_pre_scaling << std::endl;
		// out << "  image_pre_contrast            = " << params.image_pre_contrast << std::endl;
		// out << "  image_pre_brightness          = " << params.image_pre_brightness << std::endl;

		out << "  noise_translation             = " << params.std_tCov << std::endl;
		out << "  noise_rotation                = " << params.std_rCov << std::endl;
		// out << "  moving_object_polygons		= " << params.moving_object_topics << std::endl;

		out << "  queue_size                    = " << params.queue_size << std::endl;
		out << "  left_topic                    = " << params.left_topic << std::endl;
		out << "  right_topic                   = " << params.right_topic << std::endl;
		out << "  left_info_topic               = " << params.left_info_topic << std::endl;
		out << "  right_info_topic              = " << params.right_info_topic << std::endl;

		out << "  visualisation_on              = " << params.visualisation_on << std::endl;

		out << "============================End of Paramters=================================" << std::endl;

		return out;
	}

} // end of namespace viso2_ros


#endif /* ODOMETRY_PARAMS_H */
