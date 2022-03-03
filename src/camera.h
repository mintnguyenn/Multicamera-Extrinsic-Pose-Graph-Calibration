#ifndef CAMERA_H
#define CAMERA_H

#include "camera_interface.h"

class Camera : public CameraInterface {
public:
    Camera();
    
    Camera(std::string message, bool show); // Default constructor
    ~Camera();

    void setCameraMatrix(cv::Mat camera_matrix);

    void setCameraImage(cv::Mat input_image);

    bool Read_ArUco_YAML(const std::string &fileName, cv::Ptr<cv::aruco::Dictionary> &dictionary,
                                               std::vector<int> &ids, std::vector<std::vector<cv::Point3f>> &objPoints);

    /**
    Create and run the threads of this class
    */
    void runThreads();

    void extrinsicCalibration();

    cv::Mat getTransformationMatrix();

private:
    int mode_;
    std::string name_;
    bool show_;

    BoardConfiguration board_config_;

    CameraData camera1_;

    cv::Mat tf_;
    std::mutex tf_mtx_;

    std::vector<std::thread> threads_; //!< Container of threads to be able to terminate then in destructor
    std::atomic<bool> running_; //!< We use this to indicate the loop in threads should still be running

};

#endif // CAMERA_H