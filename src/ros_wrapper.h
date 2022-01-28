#ifndef ROS_WRAPPER_H
#define ROS_WRAPPER_H

#include "ros/ros.h"
#include "calibration.h"

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
    RosWrapper(ros::NodeHandle nh, std::shared_ptr<ExtrinsicCalibrationInterface> calibPtr);

    /*! @brief RosWrapper destructor.
     *
     *  Will tear down the object
     */
    ~RosWrapper();

    void setCamera(std::vector<std::shared_ptr<ExtrinsicCalibrationInterface>> camPtr);

private:
    void cameraImageCallback(const sensor_msgs::ImageConstPtr &msg);

    void camera2Callback(const sensor_msgs::ImageConstPtr &msg);

    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &info);

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub1_, sub2_;

    std::shared_ptr<ExtrinsicCalibrationInterface> calibPtr_;
    std::vector<std::shared_ptr<ExtrinsicCalibrationInterface>> cameras_;

    bool flag_1 = false;
};

#endif // ROS_WRAPPER_H