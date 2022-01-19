#ifndef ROS_WRAPPER_H
#define ROS_WRAPPER_H

#include "ros/ros.h"

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

#include <yaml-cpp/yaml.h>
#include <unistd.h>

//! The class we have developed included here in our node

class RosWrapper
{

public:
    /*! @brief RosWrapper constructor.
     *
     *  Will take the node handle and initialise the callbacks and internal variables
     */
    RosWrapper(ros::NodeHandle nh);

    /*! @brief RosWrapper destructor.
     *
     *  Will tear down the object
     */
    ~RosWrapper();

    /*! @brief seperate thread.
     *
     *  The main processing thread that will run continously and utilise the data
     *  When data needs to be combined then running a thread seperate to callback will gurantee data is processed
     */
    void seperateThread();

private:
    void imageCallback(const sensor_msgs::ImageConstPtr &msg);

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;

    sensor_msgs::ImageConstPtr img_;

    std::mutex mtx_;

    cv::Ptr<cv::aruco::Dictionary> dictionary;
    std::vector<int> ids;
    std::vector<std::vector<cv::Point3f>> objPoints;
};

#endif // ROS_WRAPPER_H