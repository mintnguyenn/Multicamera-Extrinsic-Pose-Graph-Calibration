#ifndef CALIBRATION_H
#define CALIBRATION_H

#include "calibration_interface.h"

class ExtrinsicCalibration : public ExtrinsicCalibrationInterface {
public:
    ExtrinsicCalibration(std::vector<std::shared_ptr<CameraInterface>> cameras); // Default constructor
    ~ExtrinsicCalibration();

    void threads();

private:
    std::vector<std::thread> threads_; //!< Container of threads to be able to terminate then in destructor
    std::atomic<bool> running_; //!< We use this to indicate the loop in threads should still be running

    std::vector<std::shared_ptr<CameraInterface>> cameras_; 
};

#endif // CAMERA_H