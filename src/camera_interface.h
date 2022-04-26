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



struct BoardConfig{
    std::vector<std::vector<cv::Point3f>> objPoints;
    cv::Ptr<cv::aruco::Dictionary> dictionary;
    std::vector<int> ids;
}; 

struct CameraData{
    cv::Mat image;
    std::mutex img_mtx;
    cv::Mat instrinsic;
    std::mutex i_mtx;
};

class CameraInterface{
public:
    CameraInterface(){}; // Default constructor

    virtual void setCameraMatrix(cv::Mat camera_matrix) = 0;

    virtual void setCameraImage(cv::Mat input_image) = 0;

    /**
    Create and run the threads of this class
    */
    virtual void runThreads() = 0;

    virtual void extrinsicCalibration() = 0;

    virtual void boardDetection(cv::Mat image, cv::Mat intrinsic, cv::Vec4d &quaternion, cv::Vec3d &tvec) = 0;

    virtual cv::Mat getTransformationMatrix() = 0;
};

#endif // CAMERA_INTERFACE_H