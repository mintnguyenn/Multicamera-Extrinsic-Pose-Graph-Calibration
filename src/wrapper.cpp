#include "wrapper.h"

// #include "read_yaml.cpp"

Wrapper::Wrapper(ros::NodeHandle nh) : nh_(nh), it_(nh)
{
  image_sub_7_ = it_.subscribe("camera7/infra1/image_rect_raw", 1000, &Wrapper::camera7ImageCallback, this);
  // cam_info_sub_7_ = nh_.subscribe("camera7/infra1/camera_info", 1000, &Wrapper::camera7InfoCallback, this);

  image_sub_8_ = it_.subscribe("camera8/infra1/image_rect_raw", 1000, &Wrapper::camera8ImageCallback, this);
  // cam_info_sub_8_ = nh_.subscribe("camera8/infra1/camera_info", 1000, &Wrapper::camera8InfoCallback, this);

  image_sub_9_ = it_.subscribe("camera9/infra1/image_rect_raw", 1000, &Wrapper::camera9ImageCallback, this);
  // cam_info_sub_9_ = nh_.subscribe("camera9/infra1/camera_info", 1000, &Wrapper::camera9InfoCallback, this);

  image_sub_10_ = it_.subscribe("camera10/infra1/image_rect_raw", 1000, &Wrapper::camera10ImageCallback, this);
  // cam_info_sub_10_ = nh_.subscribe("camera10/infra1/camera_info", 1000, &Wrapper::camera10InfoCallback, this);

  std::shared_ptr<CameraInterface> camera7(new Camera(true, 7));
  std::shared_ptr<CameraInterface> camera8(new Camera(false, 8));
  std::shared_ptr<CameraInterface> camera9(new Camera(false, 9));
  std::shared_ptr<CameraInterface> camera10(new Camera(false, 10));
  cam7_ = camera7;
  cam8_ = camera8;
  cam9_ = camera9;
  cam10_ = camera10;

  ready_ = false;

  const std::string fileName = "/home/mintnguyen/Documents/multi-cameras-calibration/yaml/camera_info.yaml";
  std::vector<cv::Mat> camera_info_vec = yaml::Read_Intrinsic(fileName);
  cam7__.instrinsic  = camera_info_vec.at(0);
  cam8__.instrinsic  = camera_info_vec.at(1);
  cam9__.instrinsic  = camera_info_vec.at(2);
  cam10__.instrinsic = camera_info_vec.at(3);
}

Wrapper::~Wrapper() {}

void Wrapper::camera7ImageCallback(const sensor_msgs::ImageConstPtr &msg){
  cv::Mat input_image = cv_bridge::toCvShare(msg, "bgr8")->image; // Convert sensor_msgs/Image to cv::Mat
  // cam7_->setCameraImage(input_image);

  cv::Vec4d quaternion; cv::Vec3d tvec;
  cam7_->boardDetection(input_image, cam7__.instrinsic, quaternion, tvec);
  
  std::cout << std::endl;
  std::cout << "7: " << msg->header.stamp << std::endl;
  std::cout << "7: " << quaternion << "; " << tvec << std::endl;

  ready_ = true;
}

void Wrapper::camera8ImageCallback(const sensor_msgs::ImageConstPtr &msg){
  if (ready_){
    cv::Mat input_image = cv_bridge::toCvShare(msg, "bgr8")->image; // Convert sensor_msgs/Image to cv::Mat
    // cam8_->setCameraImage(input_image);

    cv::Vec4d quaternion; cv::Vec3d tvec;
    cam8_->boardDetection(input_image, cam8__.instrinsic, quaternion, tvec);

    std::cout << "8: " << msg->header.stamp << std::endl;
    std::cout << "8: " << quaternion << "; " << tvec << std::endl;
  }
}

void Wrapper::camera9ImageCallback(const sensor_msgs::ImageConstPtr &msg){
  if (ready_){
    cv::Mat input_image = cv_bridge::toCvShare(msg, "bgr8")->image; // Convert sensor_msgs/Image to cv::Mat
    // cam9_->setCameraImage(input_image);

    cv::Vec4d quaternion; cv::Vec3d tvec;
    cam9_->boardDetection(input_image, cam9__.instrinsic, quaternion, tvec);

    std::cout << "9: " << msg->header.stamp << std::endl;
    std::cout << "9: " << quaternion << "; " << tvec << std::endl;
  }
}

void Wrapper::camera10ImageCallback(const sensor_msgs::ImageConstPtr &msg){
  if (ready_){
    cv::Mat input_image = cv_bridge::toCvShare(msg, "bgr8")->image; // Convert sensor_msgs/Image to cv::Mat
    // cam10_->setCameraImage(input_image);

    cv::Vec4d quaternion; cv::Vec3d tvec;
    cam10_->boardDetection(input_image, cam10__.instrinsic, quaternion, tvec);

    std::cout << "10:" << msg->header.stamp << std::endl;
    std::cout << "10:" << quaternion << "; " << tvec << std::endl;
  }
}

// void Wrapper::camera7InfoCallback(const sensor_msgs::CameraInfoConstPtr &info){
//   if (!flag7){
//     cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << info->K[0], 0, info->K[2],
//                              0, info->K[4], info->K[5],
//                              0, 0, 1);

//     cam7_->setCameraMatrix(camera_matrix);

//     flag7 = true;
//   }
// }

// void Wrapper::camera8InfoCallback(const sensor_msgs::CameraInfoConstPtr &info){
//   if (!flag8){
//     cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << info->K[0], 0, info->K[2],
//                              0, info->K[4], info->K[5],
//                              0, 0, 1);

//     cam8_->setCameraMatrix(camera_matrix);

//     flag8 = true;
//   }
// }

// void Wrapper::camera9InfoCallback(const sensor_msgs::CameraInfoConstPtr &info){
//   if (!flag9){
//     cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << info->K[0], 0, info->K[2],
//                              0, info->K[4], info->K[5],
//                              0, 0, 1);

//     cam9_->setCameraMatrix(camera_matrix);

//     flag9 = true;
//   }
// }

// void Wrapper::camera10InfoCallback(const sensor_msgs::CameraInfoConstPtr &info){
//   if (!flag10){
//     cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << info->K[0], 0, info->K[2],
//                              0, info->K[4], info->K[5],
//                              0, 0, 1);

//     cam10_->setCameraMatrix(camera_matrix);

//     flag10 = true;
//   }
// }