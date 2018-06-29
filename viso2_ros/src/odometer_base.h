#ifndef ODOMETER_BASE_H_
#define ODOMETER_BASE_H_

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Scalar.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <geometry_msgs/TransformStamped.h>

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

	
	// publisher
	ros::Publisher odom_pub_; // odometry using pure VO algorithm, no fusion at all
	ros::Publisher pose_pub_; // odometry in pose format using pure VO algorithm, no fusion at all
	ros::Publisher pose_iw_pub_; // pose change with respect to world frame

	ros::ServiceServer reset_service_;

	// tf related
	std::string sensor_frame_id_;
	std::string odom_frame_id_;
	std::string base_link_frame_id_;
	tf2_ros::Buffer tfBuffer;
	tf2_ros::TransformListener tf_listener_;
	tf2_ros::TransformBroadcaster tf_broadcaster_;
	bool publish_tf_;

	ros::Time global_start_;

	// the current integrated camera pose
	tf2::Transform integrated_vo_pose_;

	// covariances
	boost::array<double, 36> pose_covariance_;
	boost::array<double, 36> twist_covariance_;

public:

	OdometerBase() : tf_listener_(tfBuffer), global_start_(0)
	{
	// Read local parameters
	ros::NodeHandle local_nh("~");

	local_nh.param("odom_frame_id", odom_frame_id_, std::string("visual_frame"));
	local_nh.param("base_link_frame_id", base_link_frame_id_, std::string("imu_frame"));
	local_nh.param("sensor_frame_id", sensor_frame_id_, std::string("camera_frame"));
	local_nh.param("publish_tf", publish_tf_, true);

	ROS_INFO_STREAM("Basic Odometer Settings:" << std::endl <<
			"  odom_frame_id      = " << odom_frame_id_ << std::endl <<
			"  base_link_frame_id = " << base_link_frame_id_ << std::endl <<
			"  sensor_frame_id    = " << sensor_frame_id_ << std::endl <<
			"  publish_tf         = " << (publish_tf_ ? "true":"false"));
	
	// advertise
	odom_pub_ = local_nh.advertise<nav_msgs::Odometry>("odometry", 3);
	pose_pub_ = local_nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose", 3);
	pose_iw_pub_ = local_nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose_iw_", 3);

	// reset service
	reset_service_ = local_nh.advertiseService("reset_pose", &OdometerBase::resetPose, this);

	// reset outputs for publishers

	integrated_vo_pose_.setIdentity();
	pose_covariance_.assign(0.0);
	twist_covariance_.assign(0.0);
	
	}

protected:

	void setSensorFrameId(const std::string& frame_id)
	{
		sensor_frame_id_ = frame_id;
	}

	std::string getSensorFrameId() const
	{
		return sensor_frame_id_;
	}

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

	void integrateAndPublish(const tf2::Transform& delta_transform, const ros::Time& time_curr, const ros::Time& time_pre)
	{

		bool do_publish_for_ekf = true;
		// sanity check
		if (time_pre.isZero())
		{
			ROS_WARN("[odometer] time_pre is zero!");
			return;
		}
		if (  fabs((time_curr - time_pre).toSec()) > 5.0 )
		{
			ROS_WARN("[odometer] timestamps between frames larger than 5 seconds! consider to reset the odometry");
			return;
		}
		if (time_curr < time_pre)
		{
			ROS_WARN("[odometer] saw negative time change in incoming sensor data! consider to reset the odometry");
			return;
		}
		if (!global_start_.isZero() && global_start_ > time_pre )
		{
			ROS_WARN_STREAM("[odometer] EKF initiated, but ignorning frames before global start time : " << time_pre);
			do_publish_for_ekf = false;
		}

		//// HM: the integrated pose is with respect to the left camera center ////

		//// INTEGRATION ////

		

		//std::cout << "delta_transform: " << tf2::toMsg(delta_transform) << std::endl;

		//////////////////////////////////
		// STEP 1, publish visual odometry
		//////////////////////////////////

		// calculate VO integrated pose (Camera-centred)
		// delta_transform consist of active rotation matrix of frames t-1 to t
		integrated_vo_pose_ *= delta_transform; // HM: behave like matrix multiplication in homogenous coordinates

		// calculate VO integrated pose (Camera-centred)
		tf2::Transform base_transform = integrated_vo_pose_;

		nav_msgs::Odometry odometry_msg;
		odometry_msg.header.stamp = time_curr;
		odometry_msg.header.frame_id = odom_frame_id_;  // this is the fixed "global" frame
		odometry_msg.child_frame_id = base_link_frame_id_; // this is the reference body frame
		tf2::toMsg(base_transform, odometry_msg.pose.pose);

		// calculate twist (not possible for first run as no delta_t can be computed)
		tf2::Transform delta_base_transform = delta_transform ;
		if (!time_pre.isZero())
		{
			double delta_t = (time_curr - time_pre).toSec();
			if (delta_t)
			{
			odometry_msg.twist.twist.linear.x = delta_base_transform.getOrigin().getX() / delta_t;
			odometry_msg.twist.twist.linear.y = delta_base_transform.getOrigin().getY() / delta_t;
			odometry_msg.twist.twist.linear.z = delta_base_transform.getOrigin().getZ() / delta_t;
			tf2::Quaternion delta_rot = delta_base_transform.getRotation();
			tf2Scalar angle = delta_rot.getAngle();
			tf2::Vector3 axis = delta_rot.getAxis();
			tf2::Vector3 angular_twist = axis * angle / delta_t;
			odometry_msg.twist.twist.angular.x = angular_twist.x();
			odometry_msg.twist.twist.angular.y = angular_twist.y();
			odometry_msg.twist.twist.angular.z = angular_twist.z();
			}
		}

		odometry_msg.pose.covariance = pose_covariance_; // pose_covariance_ is defined outside
		odometry_msg.twist.covariance = twist_covariance_;
		odom_pub_.publish(odometry_msg);

		//////////////////////////////////
		// STEP 2, publish visual odometry (pose)
		//////////////////////////////////
		geometry_msgs::PoseWithCovarianceStamped pose_msg;
		pose_msg.header.stamp = odometry_msg.header.stamp;
		pose_msg.header.frame_id = odometry_msg.header.frame_id;
		pose_msg.pose.covariance = pose_covariance_;
		pose_msg.pose.pose = odometry_msg.pose.pose;

		pose_pub_.publish(pose_msg);


		//////////////////////////////////
		// STEP 3, publish world frame pose estimate (for EKF)
		//////////////////////////////////
		tf2::Stamped<tf2::Transform> base_to_sensor;
		
		try
		{
			geometry_msgs::TransformStamped base_to_sensor_msg = tfBuffer.lookupTransform(
				base_link_frame_id_, // target_frame
				sensor_frame_id_, // source_frame
				time_pre ); // target to source

			tf2::fromMsg(base_to_sensor_msg, base_to_sensor);
			ROS_INFO_STREAM_THROTTLE(0.5, "base_to_sensor: " << base_to_sensor_msg );
			
		}
		catch (tf2::TransformException ex){
			if (!global_start_.isZero())
				ROS_ERROR( "base_to_sensor: %s",ex.what());	
			base_to_sensor.setIdentity();
			do_publish_for_ekf = false;
		}

		tf2::Stamped<tf2::Transform> world_to_base;
		
		try
		{
			geometry_msgs::TransformStamped world_to_base_msg = tfBuffer.lookupTransform(
				"world_frame", // target_frame
				base_link_frame_id_, // source_frame
				time_pre); // target to source

			tf2::fromMsg(world_to_base_msg, world_to_base);
			ROS_INFO_STREAM_THROTTLE(1, "world_to_base: " << world_to_base_msg);
			
		}
		catch (tf2::TransformException ex){
			if (!global_start_.isZero())
				ROS_ERROR("world_to_base: %s",ex.what());
			world_to_base.setIdentity();
			do_publish_for_ekf = false;
		}

		

		if ( do_publish_for_ekf )
		{
			static int _seq = 0;

			// read from left to right, base_to_sensor.inverse() assume slow changing calibration
			tf2::Transform world_transform = world_to_base * base_to_sensor * delta_transform * base_to_sensor.inverse(); // * base_to_sensor.inverse()
			geometry_msgs::PoseWithCovarianceStamped pose_iw_msg;
			pose_iw_msg.header.stamp = time_curr;
			pose_iw_msg.header.seq = _seq++;
			pose_iw_msg.header.frame_id = "world_frame";
			pose_iw_msg.pose.covariance = twist_covariance_;

			// std::cout << "origin:" <<  world_transform.getOrigin()  << std::endl;
			// std::cout << "rotation:" <<  world_transform.getRotation()  << std::endl;
			tf2::toMsg(world_transform, pose_iw_msg.pose.pose);	
			pose_iw_pub_.publish(pose_iw_msg);

			ROS_INFO_STREAM("EKF Measurement Published! seq = " << pose_iw_msg.header.seq);
		}

		// tf_broadcaster_.sendTransform(tf::StampedTransform(base_transform, time_curr,
		// 	odom_frame_id_, base_link_frame_id_));
	}


	bool resetPose(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
	{
		integrated_vo_pose_.setIdentity();
		pose_covariance_.assign(0.0);
		twist_covariance_.assign(0.0);
		global_start_ = ros::Time::now();

		ROS_INFO("==========Request to Reset Pose, Completed==============");
		ROS_INFO_STREAM("==============Global Start Time: " << global_start_ <<"==============");
		return true;
	}

};

} // end of namespace

#endif

