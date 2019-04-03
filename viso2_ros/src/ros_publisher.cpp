// ROS Node for Realsense D415 Streams
// Cheng Huimin, June 2018
//
//
#include "ros_publisher.hpp"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <iostream>

#define BUFFER_SIZE 10


StereoCameraPublisher::StereoCameraPublisher()
{
    ros::NodeHandle _nh("~");
    init();
}


StereoCameraPublisher::StereoCameraPublisher(ros::NodeHandle& nh) : _nh(nh)
{
    init();
}


StereoCameraPublisher::~StereoCameraPublisher()
{
    std::cout << "StereoCameraPublisher() destructor." << std::endl;
}

void StereoCameraPublisher::init()
{
    _it = new image_transport::ImageTransport(_nh);
    _pubLeft = new auto( _it->advertiseCamera("left/debug_left",BUFFER_SIZE));
    _pubRight = new auto( _it->advertiseCamera("right/debug_right",BUFFER_SIZE));
    std::cout << "Publisher initialised." << std::endl;
}

void StereoCameraPublisher::publish(const cv::Mat &imageLeft_cv,const  cv::Mat &imageRight_cv,const  sensor_msgs::CameraInfoConstPtr &cameraInfo_left,
       const sensor_msgs::CameraInfoConstPtr &cameraInfo_right, const  ros::Time &sensor_timestamp)
{
    std_msgs::Header header;
    header.stamp = sensor_timestamp;
    // std::cout << "Publishing " << std::endl;
    // // convert to pointer format
    // sensor_msgs::CameraInfoConstPtr cameraInfoPtr_left = boost::make_shared<sensor_msgs::CameraInfo>(cameraInfo_left);
    // sensor_msgs::CameraInfoConstPtr cameraInfoPtr_right = boost::make_shared<sensor_msgs::CameraInfo>(cameraInfo_right);

    // publish left image
    cv_bridge::CvImage imageLeft_bridge = cv_bridge::CvImage(header, \
                sensor_msgs::image_encodings::BGR8, imageLeft_cv);

    _pubLeft->publish(imageLeft_bridge.toImageMsg(),cameraInfo_left);

    // publish right image
    cv_bridge::CvImage imageRight_bridge = cv_bridge::CvImage(header, \
                sensor_msgs::image_encodings::BGR8, imageRight_cv);

    _pubRight->publish(imageRight_bridge.toImageMsg(),cameraInfo_right);

    // std::cout << "Done Publish " << std::endl;
}