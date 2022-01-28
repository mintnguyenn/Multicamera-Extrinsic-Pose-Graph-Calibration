// roslaunch realsense2_camera rs_multiple_devices.launch serial_no_camera1:=801212071175 serial_no_camera2:=817612071347
// camera/color/image_raw
//

#include "ros_wrapper.h"

RosWrapper::RosWrapper(ros::NodeHandle nh, std::vector<std::shared_ptr<CameraInterface>> cameras) : nh_(nh), cameras_(cameras)
{
  // sub1_ = nh_.subscribe("camera/color/image_raw", 1000, &RosWrapper::camera1Callback, this);
  sub1_ = nh_.subscribe("camera15/infra1/image_rect_raw", 1000, &RosWrapper::camera1ImageCallback, this);

  sub2_ = nh_.subscribe("camera15/infra1/camera_info", 1000, &RosWrapper::camera1InfoCallback, this);

  sub3_ = nh_.subscribe("camera16/infra1/image_rect_raw", 1000, &RosWrapper::camera2ImageCallback, this);
  sub4_ = nh_.subscribe("camera16/infra1/camera_info", 1000, &RosWrapper::camera2InfoCallback, this);
}

RosWrapper::~RosWrapper() {}

void RosWrapper::setCamera(std::vector<std::shared_ptr<CameraInterface>> camPtr)
{
  cameras_ = camPtr;
}

void RosWrapper::camera1ImageCallback(const sensor_msgs::ImageConstPtr &msg)
{
  const int camera_index = 1; // Camera number 1
  if (camera_index <= cameras_.size()) // Avoid core dumped, if there is only one camera, callback for camera number 2 will not be called
  {
    cv::Mat input_image = cv_bridge::toCvShare(msg, "bgr8")->image; // Convert ros topic to image matrix
    cameras_.at(0)->setCameraImage(input_image);
  }
}

void RosWrapper::camera1InfoCallback(const sensor_msgs::CameraInfoConstPtr &info)
{
  const int camera_index = 1;
  if (!flag_1 && camera_index <= cameras_.size())
  {
    cv::Mat camera_matrix = cv::Mat::zeros(3, 3, CV_32F);
    camera_matrix.at<float>(0, 0) = info->K[0];
    camera_matrix.at<float>(0, 2) = info->K[2];
    camera_matrix.at<float>(1, 1) = info->K[4];
    camera_matrix.at<float>(1, 2) = info->K[5];
    camera_matrix.at<float>(2, 2) = 1;

    cameras_.at(0)->setCameraMatrix(camera_matrix);

    flag_1 = true;
  }
}

void RosWrapper::camera2ImageCallback(const sensor_msgs::ImageConstPtr &msg2)
{
  const int camera_index = 2;
  if (camera_index <= cameras_.size())
  {
    cv::Mat input_image = cv_bridge::toCvShare(msg2, "bgr8")->image;
    cameras_.at(1)->setCameraImage(input_image);
  }
}

void RosWrapper::camera2InfoCallback(const sensor_msgs::CameraInfoConstPtr &info2)
{
  const int camera_index = 2;
  if (!flag_2 && camera_index <= cameras_.size())
  {
    cv::Mat camera_matrix = cv::Mat::zeros(3, 3, CV_32F);
    camera_matrix.at<float>(0, 0) = info2->K[0];
    camera_matrix.at<float>(0, 2) = info2->K[2];
    camera_matrix.at<float>(1, 1) = info2->K[4];
    camera_matrix.at<float>(1, 2) = info2->K[5];
    camera_matrix.at<float>(2, 2) = 1;

    cameras_.at(1)->setCameraMatrix(camera_matrix);

    flag_2 = true;
  }
}