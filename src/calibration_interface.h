#ifndef CALIBRATION_INTERFACE_H
#define CALIBRATION_INTERFACE_H

#include <iostream>

#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/dictionary.hpp>
#include <opencv2/calib3d.hpp>

#include <yaml-cpp/yaml.h>
#include <unistd.h>

struct BoardConfiguration{
    std::vector<std::vector<cv::Point3f>> objPoints;
    cv::Ptr<cv::aruco::Dictionary> dictionary;
    std::vector<int> ids;
}; 

struct Camera{
    cv::Mat image;
    std::mutex img_mtx;
    cv::Mat instrinsic;
    std::mutex i_mtx;
};

class ExtrinsicCalibrationInterface{
public:
    ExtrinsicCalibrationInterface(){}; // Default constructor

    virtual void setCameraMatrix(cv::Mat camera_matrix) = 0;

    virtual void setCameraImage(cv::Mat input_image) = 0;

    virtual bool Read_ArUco_YAML(const std::string &fileName, cv::Ptr<cv::aruco::Dictionary> &dictionary,
                                 std::vector<int> &ids, std::vector<std::vector<cv::Point3f>> &objPoints) = 0;

    /**
    Create and run the threads of this class
    */
    virtual void runThreads() = 0;

    virtual void thread() = 0;
};

#endif // CALIBRATION_INTERFACE_H