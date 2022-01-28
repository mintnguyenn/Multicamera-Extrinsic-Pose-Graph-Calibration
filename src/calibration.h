#ifndef CALIBRATION_H
#define CALIBRATION_H

#include "calibration_interface.h"

class ExtrinsicCalibration : public ExtrinsicCalibrationInterface {
public:
    ExtrinsicCalibration(); // Default constructor
    ~ExtrinsicCalibration();

    void setCameraMatrix(cv::Mat camera_matrix);

    void setCameraImage(cv::Mat input_image);

    bool Read_ArUco_YAML(const std::string &fileName, cv::Ptr<cv::aruco::Dictionary> &dictionary,
                                               std::vector<int> &ids, std::vector<std::vector<cv::Point3f>> &objPoints);

    /**
    Create and run the threads of this class
    */
    void runThreads();

    void extrinsicCalibration();

private:
    int mode_;

    BoardConfiguration board_config_;

    Camera camera1_;

    std::vector<std::thread> threads_; //!< Container of threads to be able to terminate then in destructor
    std::atomic<bool> running_; //!< We use this to indicate the loop in threads should still be running

};

#endif // CALIBRATION_H