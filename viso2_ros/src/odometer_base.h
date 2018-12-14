#ifndef ODOMETER_BASE_H_
#define ODOMETER_BASE_H_

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2_eigen/tf2_eigen.h>

// #include <tf2_ros/transform_broadcaster.h>

#include <Eigen/Eigen>

#include "ros_publisher.hpp"

namespace viso2_ros
{

/**
 * Base class for odometers, handles tf's, odometry and pose
 * publishing. This can be used as base for any incremental pose estimating
 * sensor. Sensors that measure velocities cannot be used.
 */
class OdometerBase
{

private:

	// rosbag::Bag bag;

	// publisher
	ros::Publisher odom_pub_; // odometry using pure VO algorithm, no fusion at all
	ros::Publisher pose_pub_; // odometry in pose format using pure VO algorithm, no fusion at all
	ros::Publisher vel_pub_;

	ros::Subscriber ekf_sub_;
	

	StereoCameraPublisher debug_pub_;

	// ros::ServiceServer reset_service_;

	// tf related
	const std::string sensor_frame_id_ = "camera_frame";
	const std::string odom_frame_id_ = "visual_frame";
	const std::string base_link_frame_id_ = "imu_frame";

	// tf2_ros::TransformBroadcaster tf_broadcaster_;

	uint64_t global_start_;

	bool use_ekf_pose_as_initial;
	bool isGlobalPoseEnabled;
	std::string initial_pose_topic;

	// the current integrated camera pose
	Eigen::Affine3d integrated_vo_pose_;
	Eigen::Affine3d initial_imu_pose;
	Eigen::Affine3d Tr_ci; // this is read from parameter

	// covariances
	boost::array<double, 36> pose_covariance_ = {};
	boost::array<double, 36> twist_covariance_ = {};

public:

	OdometerBase() : global_start_(0), isGlobalPoseEnabled(false)
	{
	// Read local parameters
	ros::NodeHandle local_nh("~");
	ros::NodeHandle nh;
	
	// advertise
	odom_pub_ = local_nh.advertise<nav_msgs::Odometry>("odometry", 3);
	pose_pub_ = local_nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose", 3);
	vel_pub_ = local_nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("velocity", 3);

	

	ROS_ASSERT(local_nh.getParam("use_ekf_pose_as_initial", 						use_ekf_pose_as_initial));
	ROS_ASSERT(local_nh.getParam("initial_pose_topic", 						initial_pose_topic));

	initial_imu_pose.matrix().setZero();

	if (use_ekf_pose_as_initial)
		ekf_sub_ = nh.subscribe("/ekf_fusion/pose",3, &OdometerBase::ekfCallback, this);

	// bag.open("/tmp/viso_ros_pose.bag",rosbag::bagmode::Write);
	loadCameraImuTransform();
	resetPose();
	}

	void publishDebugImg(const cv::Mat &imageLeft_cv,const  cv::Mat &imageRight_cv,const  sensor_msgs::CameraInfoConstPtr &cameraInfo_left,
       const sensor_msgs::CameraInfoConstPtr &cameraInfo_right, const  ros::Time &sensor_timestamp){
		
		static ros::Time last_stamp = ros::Time(0);

		const bool noLimitRate = true;

		if (noLimitRate || last_stamp.isZero() || (sensor_timestamp - last_stamp).toSec() > 0.15 ){ // limit publishing rate
			debug_pub_.publish(imageLeft_cv, imageRight_cv, cameraInfo_left, cameraInfo_right, sensor_timestamp);
			last_stamp = sensor_timestamp;
		}
			
	}

protected:

	void setPoseCovariance(double tCov, double rCov)
	{
		pose_covariance_[0] = pose_covariance_[7] = pose_covariance_[14] = tCov;
		pose_covariance_[21] = pose_covariance_[28] = pose_covariance_[35] = rCov;
	}

	void setTwistCovariance(double tCov, double rCov)
	{
		twist_covariance_[0] = twist_covariance_[7] = twist_covariance_[14] = tCov;
		twist_covariance_[21] = twist_covariance_[28] = twist_covariance_[35] = rCov;
	}

	

	void integrateAndPublish(const Eigen::Affine3d &delta_transform, const uint64_t time_curr, const uint64_t time_pre)
	{

		// bool do_publish_for_ekf = true;
		// sanity check
		assert(time_pre > 0 && time_curr > time_pre); // negative time change in incoming sensor data!

		if (time_curr > time_pre + 1*1e9){ // timestamps between frames larger than 1 second!
			ROS_ERROR_STREAM("Detect big time difference between VO frames");
			return; // abort publish
		} 

		if ( global_start_ != 0 && global_start_ > time_pre )
		{
			ROS_WARN_STREAM("EKF initiated, but ignorning frames before global start time : " << ros::Time().fromNSec(time_pre));
			return;
		}

		//// HM: the integrated pose is with respect to the left camera center ////

		//// INTEGRATION ////

		//////////////////////////////////
		// STEP 1, publish visual odometry
		//////////////////////////////////

		// calculate VO integrated pose (Camera-centred)
		// delta_transform consist of active rotation matrix of frames t-1 to t

		// change of base from current -> previous, then from previous to time 0

		// attempt to initialise initial_imu_pose
		// if (!isGlobalPoseEnabled){
		// 	ros::NodeHandle nh;
		// 	ROS_WARN_STREAM_THROTTLE(5,"Attempt to initialise integrated pose from " << initial_pose_topic);
		// 	auto msgptr = ros::topic::waitForMessage<geometry_msgs::PoseWithCovarianceStamped>(initial_pose_topic,nh, ros::Duration(0.01)); // , ros::Duration(0.01)

		// }
		
		if (isGlobalPoseEnabled)
		{
			assert(!initial_imu_pose.matrix().isZero());
			integrated_vo_pose_ = integrated_vo_pose_ * delta_transform; // HM: behave like matrix multiplication in homogenous coordinates
			
			Eigen::Affine3d integrated_p_iw = initial_imu_pose * Tr_ci * integrated_vo_pose_;

			geometry_msgs::Pose integrated_vo_pose_msg = tf2::toMsg(integrated_p_iw);

			
			nav_msgs::Odometry odometry_msg;
			odometry_msg.header.stamp = ros::Time().fromNSec(time_curr);
			odometry_msg.header.frame_id = odom_frame_id_;  // this is the fixed "global" frame
			odometry_msg.child_frame_id = base_link_frame_id_; // this is the reference body frame
			odometry_msg.pose.pose = integrated_vo_pose_msg;
			odometry_msg.pose.covariance = pose_covariance_; // pose_covariance_ is defined outside
			// odometry_msg.twist.covariance = twist_covariance_;
			odom_pub_.publish(odometry_msg);

			// bag.write("odometry_msg",ros::Time::now(),odometry_msg);

			//////////////////////////////////
			// STEP 2, publish visual odometry (pose)
			//////////////////////////////////
			geometry_msgs::PoseWithCovarianceStamped pose_msg;
			pose_msg.header = odometry_msg.header;
			pose_msg.pose.covariance = pose_covariance_;
			pose_msg.pose.pose = integrated_vo_pose_msg;

			pose_pub_.publish(pose_msg);

			// bag.write("pose_msg",ros::Time::now(),odometry_msg);

		} // end of publishing integrated pose / odometry

		//// Also publish the corresponding velocity
		double delta_t = (time_curr - time_pre) / 1.0e9;
		assert (delta_t > 0);
		Eigen::Vector3d delta_translation = delta_transform.translation();
		delta_translation /= delta_t;

		geometry_msgs::PoseWithCovarianceStamped velocity_pose_msg;
		velocity_pose_msg.pose.pose.position.x = delta_translation(0);
		velocity_pose_msg.pose.pose.position.y = delta_translation(1);
		velocity_pose_msg.pose.pose.position.z = delta_translation(2);

		velocity_pose_msg.pose.covariance = pose_covariance_;

		velocity_pose_msg.header.frame_id = "camera_frame";
		velocity_pose_msg.header.stamp = ros::Time().fromNSec((time_curr + time_pre)/2);
		vel_pub_.publish(velocity_pose_msg);

	}

	void resetPose()
	// bool resetPose(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
	{
		integrated_vo_pose_.setIdentity();
		pose_covariance_.assign(0.0);
		twist_covariance_.assign(0.0);
		global_start_ = ros::Time::now().toNSec();

		if (use_ekf_pose_as_initial){
			ROS_WARN("Will wait for EKF initial pose, before publishing odometry and pose");
			isGlobalPoseEnabled = false;
		}else{
			ROS_WARN("Will use ENU as initial pose, publishing odometry and pose");
			isGlobalPoseEnabled = true;
			initial_imu_pose.setIdentity();
			Eigen::Matrix3d R_sw;
			R_sw << -1, 0 ,0,
					0, 1, 0,
					0 , 0 , -1;
			initial_imu_pose.rotate(R_sw);
		}
	}

	void loadCameraImuTransform(){

		ros::NodeHandle local_nh("~");
		Eigen::Quaternion<double> q_ci;
		ROS_ASSERT(local_nh.getParam("init/q_ci/w", q_ci.w()));
		ROS_ASSERT(local_nh.getParam("init/q_ci/x", q_ci.x()));
		ROS_ASSERT(local_nh.getParam("init/q_ci/y", q_ci.y()));
		ROS_ASSERT(local_nh.getParam("init/q_ci/z", q_ci.z()));

		Tr_ci.setIdentity();
		Tr_ci.rotate(q_ci);
		// Tr_ci.translation().setZero();

		ROS_INFO_STREAM(std::endl << "camera-imu transformation loaded Tr_ci:" << Tr_ci.matrix());
	}

	void ekfCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msgptr){
		if (isGlobalPoseEnabled == true)
			return;
		
		Eigen::Quaternion<double> q_iw = {msgptr->pose.pose.orientation.w, msgptr->pose.pose.orientation.x, 
					msgptr->pose.pose.orientation.y, msgptr->pose.pose.orientation.z};
		
		initial_imu_pose.setIdentity();
		initial_imu_pose.rotate(q_iw);
		ROS_WARN_STREAM("Initialise integrated pose using " << std::endl << initial_imu_pose.matrix() );

		isGlobalPoseEnabled = true;
	}

};

} // end of namespace

#endif

