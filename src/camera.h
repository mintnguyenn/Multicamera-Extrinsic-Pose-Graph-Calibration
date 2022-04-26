#ifndef CAMERA_H
#define CAMERA_H

#include "camera_interface.h"

class Camera : public CameraInterface {
public:
    Camera();

    Camera(bool show, unsigned int index); // Default constructor
    ~Camera();

    void setCameraMatrix(cv::Mat camera_matrix);

    void setCameraImage(cv::Mat input_image);

    /**
    Create and run the threads of this class
    */
    void runThreads();

    void extrinsicCalibration();

    cv::Mat boardDetection(cv::Mat image, cv::Mat intrinsic);

    cv::Mat getTransformationMatrix();

private:
    bool show_;
    unsigned int index_;

    BoardConfiguration board_config_;

    CameraData camera_;

    cv::Mat tf_;
    std::mutex tf_mtx_, mtxx_;

    std::vector<std::thread> threads_; //!< Container of threads to be able to terminate then in destructor
    std::atomic<bool> running_; //!< We use this to indicate the loop in threads should still be running

    std::atomic<bool> ready_;
};

#endif // CAMERA_H