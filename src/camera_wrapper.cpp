// roslaunch realsense2_camera rs_multiple_devices.launch serial_no_camera1:=801212071175 serial_no_camera2:=817612071347

#include "camera_wrapper.h"

CameraWrapper::CameraWrapper(ros::NodeHandle nh, unsigned int camera_index) : nh_(nh), it_(nh)
{
  std::string image_message = "camera" + std::to_string(camera_index) + "/infra1/image_rect_raw";
  std::string cam_info_message = "camera" + std::to_string(camera_index) + "/infra1/camera_info";

  image_sub_ = it_.subscribe(image_message, 1000, &CameraWrapper::cameraImageCallback, this);
  cam_info_sub_ = nh_.subscribe(cam_info_message, 1000, &CameraWrapper::cameraInfoCallback, this);

  std::shared_ptr<CameraInterface> camera(new Camera(true, camera_index));
  cam_ = camera;
}

CameraWrapper::~CameraWrapper() {}

void CameraWrapper::cameraImageCallback(const sensor_msgs::ImageConstPtr &msg)
{
  cv::Mat input_image = cv_bridge::toCvShare(msg, "bgr8")->image; // Convert sensor_msgs/Image to cv::Mat
  cam_->setCameraImage(input_image);
}

void CameraWrapper::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &info)
{
  if (!flag){
    cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << info->K[0], 0, info->K[2],
                                                       0, info->K[4], info->K[5],
                                                       0, 0, 1);

    cam_->setCameraMatrix(camera_matrix);

    flag = true;
  }
}