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
#include <image_transport/image_transport.h>

//! The class we have developed included here in our node


struct BoardCameraExtrinsic{
        cv::Vec4d quaternion; 
        cv::Vec3d tvec;
        uint camIndex;
        uint boardIndex;
        ros::Time stamp;
    };

class CameraWrapper
{

public:


    /*! @brief RosWrapper constructor.
     *
     *  Will take the node handle and initialise the callbacks and internal variables
     */
    CameraWrapper(ros::NodeHandle nh, unsigned int camera_index);

    /*! @brief RosWrapper destructor.
     *
     *  Will tear down the object
     */
    ~CameraWrapper();

    void setCamera(std::shared_ptr<CameraInterface> cam);

    std::vector<BoardCameraExtrinsic> getcopyboardextrinsicdata();

private:
    void cameraImageCallback(const sensor_msgs::ImageConstPtr &msg);

    // void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &info);
    
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    // ros::Subscriber cam_info_sub_;          // Subscriber
    image_transport::Subscriber image_sub_; // Subscriber

    std::shared_ptr<CameraInterface> cam_;

    // bool flag = false; // Use this flag to allow get intrinsic from /camera_info once
    int camera_index_;

    int board_index_;

    std::vector<BoardCameraExtrinsic> boardExtrinsics;
};

#endif // ROS_WRAPPER_H