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
    while (running_)
    {
        for (auto e : cameras_){
            std::cout << e->getTransformationMatrix() << std::endl;
        }
        std::cout << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        if (!running_)
            break;
    }
}