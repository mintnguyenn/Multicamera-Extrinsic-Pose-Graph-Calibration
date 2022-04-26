#ifndef WRAPPER_H
#define WRAPPER_H

#include "ros/ros.h"
#include "camera.h"

#include <sstream>
#include <iostream>
#include <string>

#include <thread>
#include <mutex>
#include <deque>
#include <atomic>
#include <condition_variable>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/aruco.hpp>
#include <image_transport/image_transport.h>

//! The class we have developed included here in our node

class Wrapper
{

public:
    /*! @brief RosWrapper constructor.
     *
     *  Will take the node handle and initialise the callbacks and internal variables
     */
    Wrapper(ros::NodeHandle nh);

    /*! @brief RosWrapper destructor.
     *
     *  Will tear down the object
     */
    ~Wrapper();

private:
    void camera7ImageCallback(const sensor_msgs::ImageConstPtr &msg);
    // void camera7InfoCallback(const sensor_msgs::CameraInfoConstPtr &info);

    void camera8ImageCallback(const sensor_msgs::ImageConstPtr &msg);
    // void camera8InfoCallback(const sensor_msgs::CameraInfoConstPtr &info);

    void camera9ImageCallback(const sensor_msgs::ImageConstPtr &msg);
    // void camera9InfoCallback(const sensor_msgs::CameraInfoConstPtr &info);

    void camera10ImageCallback(const sensor_msgs::ImageConstPtr &msg);
    // void camera10InfoCallback(const sensor_msgs::CameraInfoConstPtr &info);

private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    ros::Subscriber cam_info_sub_7_, cam_info_sub_8_, cam_info_sub_9_, cam_info_sub_10_; // Subscriber
    image_transport::Subscriber image_sub_7_, image_sub_8_, image_sub_9_, image_sub_10_; // Subscriber

    std::shared_ptr<CameraInterface> cam7_, cam8_, cam9_, cam10_;

    bool flag7 = false, flag8 = false, flag9 = false, flag10 = false; // Use this flag to allow get intrinsic from /camera_info once

    std::atomic<bool> ready_, ready2_;

    CameraData cam7__, cam8__, cam9__, cam10__;
};

#endif // ROS_WRAPPER_H