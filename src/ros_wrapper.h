#ifndef ROS_WRAPPER_H
#define ROS_WRAPPER_H

#include "ros/ros.h"
#include "camera.h"

#include <sstream>
#include <iostream>
#include <string>

#include "sensor_msgs/Image.h"

#include <thread>
#include <mutex>
#include <deque>
#include <atomic>

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/dictionary.hpp>
#include <opencv2/calib3d.hpp>

//! The class we have developed included here in our node

class RosWrapper
{

public:
    /*! @brief RosWrapper constructor.
     *
     *  Will take the node handle and initialise the callbacks and internal variables
     */
    RosWrapper(ros::NodeHandle nh, std::vector<std::shared_ptr<CameraInterface>> cameras);

    /*! @brief RosWrapper destructor.
     *
     *  Will tear down the object
     */
    ~RosWrapper();

    void setCamera(std::vector<std::shared_ptr<CameraInterface>> camPtr);

private:
    void camera1ImageCallback(const sensor_msgs::ImageConstPtr &msg);

    void camera2ImageCallback(const sensor_msgs::ImageConstPtr &msg);

    void camera1InfoCallback(const sensor_msgs::CameraInfoConstPtr &info);

    void camera2InfoCallback(const sensor_msgs::CameraInfoConstPtr &info);

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub1_, sub2_, sub3_, sub4_; // Subscribers

    std::shared_ptr<CameraInterface> cam1_, cam2_;
    std::vector<std::shared_ptr<CameraInterface>> cameras_;

    bool flag_1 = false; // Use this flag to allow get intrinsic from /camera_info once
    bool flag_2 = false; // Use this flag to allow get intrinsic from /camera_info once
};

#endif // ROS_WRAPPER_H