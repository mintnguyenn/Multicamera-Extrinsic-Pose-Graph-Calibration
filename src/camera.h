#ifndef CAMERA_H
#define CAMERA_H

#include "camera_interface.h"

class Camera : public CameraInterface {
public:
    Camera(); // Default constructor
    Camera(bool show, unsigned int index); 
    ~Camera();

    void setCameraMatrix(cv::Mat camera_matrix);

    void setCameraImage(cv::Mat input_image);

    /**
    Create and run the threads of this class
    */
    void runThreads();

    void extrinsicCalibration();

    void boardDetection(cv::Mat image, cv::Mat intrinsic, cv::Vec4d &quaternion, cv::Vec3d &tvec);

    cv::Mat getTransformationMatrix();

private:
    bool show_;
    unsigned int index_;

    BoardConfig board_;

    CameraData camera_;

    cv::Mat tf_;
    std::mutex tf_mtx_, mtxx_;

    std::vector<std::thread> threads_; //!< Container of threads to be able to terminate then in destructor
    std::atomic<bool> running_; //!< We use this to indicate the loop in threads should still be running

    std::atomic<bool> ready_;
};

#endif // CAMERA_H