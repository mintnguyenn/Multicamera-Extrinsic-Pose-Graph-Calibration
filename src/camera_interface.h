#ifndef CAMERA_INTERFACE_H
#define CAMERA_INTERFACE_H

// #include <iostream>
#include <thread>
#include <mutex>
#include <atomic>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <Eigen/Geometry>

#include "yaml_reader.h"

struct BoardConfiguration{
    std::vector<std::vector<cv::Point3f>> objPoints;
    cv::Ptr<cv::aruco::Dictionary> dictionary;
    std::vector<int> ids;
}; 

class CameraInterface{
public:
    CameraInterface(){}; // Default constructor

    virtual void setCameraImage(cv::Mat input_image) = 0;

    /**
    Create and run the threads of this class
    */
    virtual void runThreads() = 0;

    virtual void boardEstimationTesting() = 0;

    virtual bool arucoBoardDetection(cv::Mat image, cv::Vec4d &quaternion, cv::Vec3d &tvec) = 0;
};

#endif // CAMERA_INTERFACE_H