// roslaunch realsense2_camera rs_multiple_devices.launch serial_no_camera1:=801212071175 serial_no_camera2:=817612071347
// camera/color/image_raw

#include "ros_wrapper.h"

RosWrapper::RosWrapper(ros::NodeHandle nh, std::shared_ptr<ExtrinsicCalibrationInterface> calibPtr) : nh_(nh), calibPtr_(calibPtr)
{
  // sub1_ = nh_.subscribe("camera15/infra1/image_rect_raw", 1000, &RosWrapper::camera1Callback, this);
  sub1_ = nh_.subscribe("camera/color/image_raw", 1000, &RosWrapper::camera1Callback, this);
  sub2_ = nh_.subscribe("camera/color/camera_info", 1000, &RosWrapper::cameraInfoCallback, this);
}

RosWrapper::~RosWrapper() {}

void RosWrapper::camera1Callback(const sensor_msgs::ImageConstPtr &msg)
{
  cv::Mat input_image = cv_bridge::toCvShare(msg, "bgr8")->image;
  calibPtr_->setCameraImage(input_image);
}

void RosWrapper::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &info)
{
  if (!flag_1)
  {
    camera_matrix_1_ = cv::Mat::zeros(3, 3, CV_32F);
    camera_matrix_1_.at<float>(0, 0) = info->K[0];
    camera_matrix_1_.at<float>(0, 2) = info->K[2];
    camera_matrix_1_.at<float>(1, 1) = info->K[4];
    camera_matrix_1_.at<float>(1, 2) = info->K[5];
    camera_matrix_1_.at<float>(2, 2) = 1;

    calibPtr_->setCameraMatrix(camera_matrix_1_);

    flag_1 = true;
  }
}

void RosWrapper::seperateThread()
{
  ros::Duration(1.0).sleep(); // Sleep for one seconds
  ros::Rate rate_limiter(60);

  cv::Mat cameraMatrix, distCoeffs;

  const std::string fileName = "/home/mintnguyen/Documents/multi-cameras-calibration/aruco-board-markers.yaml";

  while (ros::ok() && false)
  {
    sensor_msgs::ImageConstPtr img;
    std::unique_lock<std::mutex> lck(mtx1_);
    img = img1_;
    lck.unlock();

    cv::Mat inputImage = cv_bridge::toCvShare(img, "bgr8")->image;
    // cv::Mat inputImage = cv::imread("/home/mintnguyen/Pictures/aruco1.png", 1);
    cv::Mat outputImage = inputImage.clone();

    // Marker detection
    std::vector<int> markerIds; // Create a vector contains marker id
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
    cv::Ptr<cv::aruco::Board> board = cv::aruco::Board::create(objPoints, dictionary, ids);

    cv::aruco::detectMarkers(inputImage, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);

    if (!markerIds.empty())
      cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);

    std::vector<cv::Point3f> objPointss;
    std::vector<cv::Point2f> imgPointss;
    cv::aruco::getBoardObjectAndImagePoints(board, markerCorners, markerIds, objPointss, imgPointss);
    cv::Vec3d rvecc, tvecc;
    if (!objPointss.empty() && !imgPointss.empty())
    {
      cv::solvePnP(objPointss, imgPointss, camera_matrix_1_, distCoeffs, rvecc, tvecc, false, 0);
      cv::aruco::drawAxis(outputImage, camera_matrix_1_, distCoeffs, rvecc, tvecc, 0.1);
    }

    cv::imshow("view", outputImage);
    cv::waitKey(30);

    rate_limiter.sleep();
  }
}