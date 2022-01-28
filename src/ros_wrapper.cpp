// roslaunch realsense2_camera rs_multiple_devices.launch serial_no_camera1:=801212071175 serial_no_camera2:=817612071347
// camera/color/image_raw
//

#include "ros_wrapper.h"

RosWrapper::RosWrapper(ros::NodeHandle nh, std::shared_ptr<ExtrinsicCalibrationInterface> calibPtr) : nh_(nh), calibPtr_(calibPtr)
{
  sub1_ = nh_.subscribe("camera15/infra1/image_rect_raw", 1000, &RosWrapper::cameraImageCallback, this);
  // sub1_ = nh_.subscribe("camera/color/image_raw", 1000, &RosWrapper::camera1Callback, this);
  sub2_ = nh_.subscribe("camera15/infra1/camera_info", 1000, &RosWrapper::cameraInfoCallback, this);
}

RosWrapper::~RosWrapper() {}

void RosWrapper::setCamera(std::vector<std::shared_ptr<ExtrinsicCalibrationInterface>> camPtr){
  cameras_ = camPtr;
}

void RosWrapper::cameraImageCallback(const sensor_msgs::ImageConstPtr &msg){
  cv::Mat input_image = cv_bridge::toCvShare(msg, "bgr8")->image;
  calibPtr_->setCameraImage(input_image);
}

void RosWrapper::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &info)
{
  if (!flag_1)
  {
    cv::Mat camera_matrix = cv::Mat::zeros(3, 3, CV_32F);
    camera_matrix.at<float>(0, 0) = info->K[0];
    camera_matrix.at<float>(0, 2) = info->K[2];
    camera_matrix.at<float>(1, 1) = info->K[4];
    camera_matrix.at<float>(1, 2) = info->K[5];
    camera_matrix.at<float>(2, 2) = 1;

    calibPtr_->setCameraMatrix(camera_matrix);

    flag_1 = true;
  }
}