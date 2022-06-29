// roslaunch realsense2_camera rs_multiple_devices.launch serial_no_camera1:=801212071175 serial_no_camera2:=817612071347

#include "camera_wrapper.h"

CameraWrapper::CameraWrapper(ros::NodeHandle nh, unsigned int camera_index) : nh_(nh), it_(nh)
{
  std::string image_message = "camera" + std::to_string(camera_index) + "/infra1/image_rect_raw";
  // std::string cam_info_message = "camera" + std::to_string(camera_index) + "/infra1/camera_info";

  image_sub_ = it_.subscribe(image_message, 1000, &CameraWrapper::cameraImageCallback, this);
  // cam_info_sub_ = nh_.subscribe(cam_info_message, 1000, &CameraWrapper::cameraInfoCallback, this);

  camera_index_ = camera_index;
}

// Default destructor
CameraWrapper::~CameraWrapper() {}

void CameraWrapper::cameraImageCallback(const sensor_msgs::ImageConstPtr &msg)
{
  cv::Mat input_image = cv_bridge::toCvShare(msg, "bgr8")->image; // Convert sensor_msgs/Image to cv::Mat
  cam_->setCameraImage(input_image);

  cv::Vec4d quaternion; cv::Vec3d tvec;
  cam_->arucoBoardDetection(input_image, quaternion, tvec);

  std::cout << camera_index_ << "," << tvec[0] << "," << tvec[1] << "," << tvec[2] << "," << quaternion[0] << "," << quaternion[1] << "," << quaternion[2] << "," << quaternion[3] << std::endl;
}

void CameraWrapper::setCamera(std::shared_ptr<CameraInterface> cam){
  cam_ = cam;
}