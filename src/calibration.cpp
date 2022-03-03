#include "calibration.h"

ExtrinsicCalibration::ExtrinsicCalibration(std::vector<std::shared_ptr<CameraInterface>> cameras) : cameras_(cameras)
{
    running_ = true;                                                                // Indicate the threads should be running
    threads_.push_back(std::thread(&ExtrinsicCalibrationInterface::threads, this)); // Create the thread and push it to our vector of threads
}

ExtrinsicCalibration::~ExtrinsicCalibration()
{
    running_ = false;
    for (auto &t : threads_)
        t.join(); // Join threads
}

void ExtrinsicCalibration::threads()
{
    std::vector<cv::Mat> tf_vec;
    while (running_)
    {
        tf_vec.clear();
        for (auto e : cameras_)
        {
            tf_vec.push_back(e->getTransformationMatrix());
        }

        cv::Mat extrinsic = cv::Mat::zeros(4, 4, CV_32F);
        if (tf_vec.size() > 1)
        {
            cv::Mat tf1 = tf_vec.at(0);
            cv::Mat tf2;
            if (!tf_vec.at(0).empty() && !tf_vec.at(1).empty())
            {
                cv::invert(tf_vec.at(1), tf2);
                extrinsic = tf1 * tf2;
            }
        }

        // std::cout << extrinsic << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        if (!running_)
            break;
    }
}