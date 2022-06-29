#ifndef CAMERA_H
#define CAMERA_H

#include "camera_interface.h"

class Camera : public CameraInterface
{
public:
    Camera(); // Default constructor
    Camera(unsigned int index);
    ~Camera();

    void setCameraImage(cv::Mat input_image);

    /**
    Create and run the threads of this class
    */
    void runThreads();

    void boardEstimationTesting();

    bool arucoBoardDetection(cv::Mat image, cv::Vec4d &quaternion, cv::Vec3d &tvec);

private:
    unsigned int index_;

    cv::Mat image_;
    std::mutex img_mtx_;
    cv::Mat intrinsic_;

    BoardConfiguration aruco_board_;
    cv::Ptr<cv::aruco::DetectorParameters> parameters_;
    cv::Ptr<cv::aruco::Board> board_;

    std::vector<std::thread> threads_; //!< Container of threads to be able to terminate then in destructor
    std::atomic<bool> running_;        //!< We use this to indicate the loop in threads should still be running
    std::atomic<bool> ready_;
};

#endif // CAMERA_H