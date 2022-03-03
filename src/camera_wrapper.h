#ifndef ROS_WRAPPER_H
#define ROS_WRAPPER_H

#include "ros/ros.h"
#include "camera.h"

#include <sstream>
#include <iostream>
#include <string>

#include <thread>
#include <mutex>
#include <deque>
#include <atomic>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/aruco.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>

//! The class we have developed included here in our node

class CameraWrapper
{

public:
    /*! @brief RosWrapper constructor.
     *
     *  Will take the node handle and initialise the callbacks and internal variables
     */
    CameraWrapper(ros::NodeHandle nh, std::shared_ptr<CameraInterface> cam, unsigned int camera_index);

    /*! @brief RosWrapper destructor.
     *
     *  Will tear down the object
     */
    ~CameraWrapper();

    void setCamera(std::vector<std::shared_ptr<CameraInterface>> camPtr);

private:
    void cameraImageCallback(const sensor_msgs::ImageConstPtr &msg);

    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &info);

private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    ros::Subscriber cam_info_sub_; // Subscribers
    image_transport::Subscriber image_sub_;

    std::shared_ptr<CameraInterface> cam_;

    bool flag = false; // Use this flag to allow get intrinsic from /camera_info once
};

#endif // ROS_WRAPPER_H