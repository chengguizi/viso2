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

	rosbag::Bag bag;

	// publisher
	ros::Publisher odom_pub_; // odometry using pure VO algorithm, no fusion at all
	ros::Publisher pose_pub_; // odometry in pose format using pure VO algorithm, no fusion at all

	ros::ServiceServer reset_service_;

	// tf related
	const std::string sensor_frame_id_ = "camera_frame";
	const std::string odom_frame_id_ = "visual_frame";
	const std::string base_link_frame_id_ = "imu_frame";

	// tf2_ros::TransformBroadcaster tf_broadcaster_;

	uint64_t global_start_;

	bool isEKFEnabled;

	// the current integrated camera pose
	Eigen::Affine3d integrated_vo_pose_;

	// covariances
	boost::array<double, 36> pose_covariance_ = {};
	boost::array<double, 36> twist_covariance_ = {};

public:

	OdometerBase() : global_start_(0), isEKFEnabled(true)
	{
	// Read local parameters
	ros::NodeHandle local_nh("~");
	
	// advertise
	odom_pub_ = local_nh.advertise<nav_msgs::Odometry>("odometry", 3);
	pose_pub_ = local_nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose", 3);

	// reset service
	reset_service_ = local_nh.advertiseService("reset_pose", &OdometerBase::resetPose, this);

	bag.open("/tmp/viso_ros_pose.bag",rosbag::bagmode::Write);

	// reset outputs for publishers

	integrated_vo_pose_.setIdentity();
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

		assert(time_curr < time_pre + 2*1e9); // timestamps between frames larger than 2 seconds!

		if ( global_start_ != 0 && global_start_ > time_pre )
		{
			ROS_WARN_STREAM("[odometer] EKF initiated, but ignorning frames before global start time : " << time_pre);
			// do_publish_for_ekf = false;
		}

		//// HM: the integrated pose is with respect to the left camera center ////

		//// INTEGRATION ////

		//////////////////////////////////
		// STEP 1, publish visual odometry
		//////////////////////////////////

		// calculate VO integrated pose (Camera-centred)
		// delta_transform consist of active rotation matrix of frames t-1 to t

		// change of base from current -> previous, then from previous to time 0
		integrated_vo_pose_ = integrated_vo_pose_ * delta_transform; // HM: behave like matrix multiplication in homogenous coordinates
		geometry_msgs::Pose integrated_vo_pose_msg = tf2::toMsg(integrated_vo_pose_);

		
		nav_msgs::Odometry odometry_msg;
		odometry_msg.header.stamp = ros::Time().fromNSec(time_curr);
		odometry_msg.header.frame_id = odom_frame_id_;  // this is the fixed "global" frame
		odometry_msg.child_frame_id = base_link_frame_id_; // this is the reference body frame
		odometry_msg.pose.pose = integrated_vo_pose_msg;

		// calculate twist (not possible for first run as no delta_t can be computed)
		// tf2::Transform delta_base_transform = delta_transform ;
		// double delta_t = (time_curr - time_pre).toSec();
		// if (!time_pre.isZero())
		// {
			
		// 	if (delta_t)
		// 	{
		// 	odometry_msg.twist.twist.linear.x = delta_base_transform.getOrigin().getX() / delta_t;
		// 	odometry_msg.twist.twist.linear.y = delta_base_transform.getOrigin().getY() / delta_t;
		// 	odometry_msg.twist.twist.linear.z = delta_base_transform.getOrigin().getZ() / delta_t;
		// 	tf2::Quaternion delta_rot = delta_base_transform.getRotation();
		// 	tf2Scalar angle = delta_rot.getAngle();
		// 	tf2::Vector3 axis = delta_rot.getAxis();
		// 	tf2::Vector3 angular_twist = axis * angle / delta_t;
		// 	odometry_msg.twist.twist.angular.x = angular_twist.x();
		// 	odometry_msg.twist.twist.angular.y = angular_twist.y();
		// 	odometry_msg.twist.twist.angular.z = angular_twist.z();
		// 	}
		// }

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

		bag.write("pose_msg",ros::Time::now(),odometry_msg);


		//////////////////////////////////
		// STEP 3, publish world frame pose estimate (for EKF)
		//////////////////////////////////

		

		// if ( do_publish_for_ekf )
		// {
		// 	tf2::Transform world_transform = world_to_base * base_to_sensor * delta_transform * base_to_sensor.inverse();
		// 	if (!_change_delta_pose_to_velocity)
		// 	{
		// 		//ROS_INFO_STREAM( "world_to_base: " << world_to_base_msg);
		// 		// read from left to right, base_to_sensor.inverse() assume slow changing calibration
		// 		 // * base_to_sensor.inverse()
		// 		geometry_msgs::PoseWithCovarianceStamped pose_iw_msg;
		// 		pose_iw_msg.header.stamp = time_curr;
		// 		pose_iw_msg.header.frame_id = "world_frame";
		// 		pose_iw_msg.pose.covariance = twist_covariance_;

		// 		// std::cout << "origin:" <<  world_transform.getOrigin()  << std::endl;
		// 		// std::cout << "rotation:" <<  world_transform.getRotation()  << std::endl;
		// 		tf2::toMsg(world_transform, pose_iw_msg.pose.pose);	
		// 		pose_iw_pub_.publish(pose_iw_msg);

		// 		ROS_INFO_STREAM("[DELTA POSE] EKF Measurement Published! cov:" << twist_covariance_[0]) ;
		// 	}else if (delta_t > 0.0)
		// 	{
		// 		geometry_msgs::PoseWithCovarianceStamped pose_i_msg;
		// 		tf2::toMsg(world_transform, pose_i_msg.pose.pose); // we only preserve the rotation part of the pose

		// 		tf2::Transform imu_transform = base_to_sensor * delta_transform * base_to_sensor.inverse();
		// 		tf2::Transform imu_t1_to_t2;
		// 		imu_t1_to_t2.setIdentity();
		// 		imu_t1_to_t2.setRotation(imu_transform.getRotation());

		// 		imu_transform = imu_t1_to_t2.inverse() * imu_transform;
		// 		pose_i_msg.pose.pose.position.x = imu_transform.getOrigin().getX() / delta_t;
		// 		pose_i_msg.pose.pose.position.y = imu_transform.getOrigin().getY() / delta_t;
		// 		pose_i_msg.pose.pose.position.z = imu_transform.getOrigin().getZ() / delta_t;
		// 		pose_i_msg.header.stamp = time_curr;//time_pre + (time_curr - time_pre)*0.5;
		// 		pose_i_msg.header.frame_id = "imu_frame";
		// 		pose_i_msg.pose.covariance = twist_covariance_;

		// 		pose_iw_pub_.publish(pose_i_msg);
		// 		ROS_INFO_STREAM("[VELOCITY] EKF Measurement Published! cov:" << twist_covariance_[0]);
		// 	}
			
		// }

		// tf_broadcaster_.sendTransform(tf::StampedTransform(base_transform, time_curr,
		// 	odom_frame_id_, base_link_frame_id_));
	}


	bool resetPose(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
	{
		integrated_vo_pose_.setIdentity();
		pose_covariance_.assign(0.0);
		twist_covariance_.assign(0.0);
		global_start_ = ros::Time::now().toNSec();
		// isEKFEnabled = true;

		ROS_INFO("==========Request to Reset Pose, Completed==============");
		ROS_INFO_STREAM("==============Global Start Time: " << global_start_ <<"==============");
		// ROS_WARN_STREAM("EKF Output Mode Changed to " << (isEKFEnabled ?  "Enabled." : "Disabled.") );
		return true;
	}

};

} // end of namespace

#endif

