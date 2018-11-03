// ROS Node for Realsense D415 Streams
// Cheng Huimin, July 2018
//
// ROS Interface for publishing images

#ifndef ROS_PUBLISHER_H
#define ROS_PUBLISHER_H

#include <ros/ros.h>
#include <opencv2/opencv.hpp>   // Include OpenCV API
#include <sensor_msgs/CameraInfo.h>

namespace image_transport
{
    class ImageTransport;
    class CameraPublisher;
}

class StereoCameraPublisher
{
    public:
        StereoCameraPublisher();
        StereoCameraPublisher(ros::NodeHandle& nh);
        ~StereoCameraPublisher();

        void publish(const cv::Mat &imageLeft_cv,const  cv::Mat &imageRight_cv,const  sensor_msgs::CameraInfoConstPtr &cameraInfo_left,
       const sensor_msgs::CameraInfoConstPtr &cameraInfo_right, const  ros::Time &sensor_timestamp);
    private:
        ros::NodeHandle _nh;
        image_transport::ImageTransport* _it;
        image_transport::CameraPublisher* _pubLeft;
        image_transport::CameraPublisher* _pubRight;
        void init();
};

#endif /* ROS_PUBLISHER_H */
