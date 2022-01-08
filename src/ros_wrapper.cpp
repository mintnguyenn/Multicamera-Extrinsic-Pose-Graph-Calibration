#include "ros_wrapper.h"

RosWrapper::RosWrapper(ros::NodeHandle nh) : nh_(nh)
{

  sub_ = nh_.subscribe("camera/color/image_raw", 1000, &RosWrapper::imageCallback, this);
}

RosWrapper::~RosWrapper() {}

void RosWrapper::imageCallback(const sensor_msgs::ImageConstPtr &msg)
{

  std::unique_lock<std::mutex> lck(mtx_);
  img_ = msg;
  lck.unlock();

  // try
  // {
  //   cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
  //   cv::waitKey(30);
  //   // cv::imwrite("image.jpg", cv_bridge::toCvShare(msg, "bgr8")->image);
  // }
  // catch (cv_bridge::Exception &e)

  // {
  //   ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  // }
}

void RosWrapper::seperateThread()
{
  ros::Duration(1.0).sleep(); // Sleep for one seconds
  ros::Rate rate_limiter(60);

  cv::Mat cameraMatrix, distCoeffs;
  cameraMatrix = cv::Mat::zeros(3, 3, CV_32F);
  cameraMatrix.at<float>(0, 0) = 929.6135864257812;
  cameraMatrix.at<float>(0, 2) = 638.3839111328125;
  cameraMatrix.at<float>(1, 1) = 930.523193359375;
  cameraMatrix.at<float>(1, 2) = 363.7060852050781;
  cameraMatrix.at<float>(2, 2) = 1;

  while (ros::ok())
  {
    sensor_msgs::ImageConstPtr img;
    std::unique_lock<std::mutex> lck(mtx_);
    img = img_;
    lck.unlock();

    cv::Mat inputImage = cv_bridge::toCvShare(img, "bgr8")->image;
    // cv::Mat inputImage = cv::imread("/home/mintnguyen/Pictures/aruco1.png", 1);

    // Marker detection
    std::vector<int> markerIds; // Create a vector contains marker id
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
    cv::Ptr<cv::aruco::GridBoard> board = cv::aruco::GridBoard::create(7, 4, 0.04, 0.02, dictionary);

    cv::aruco::detectMarkers(inputImage, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);

    cv::Mat outputImage = inputImage.clone();

    if (!markerIds.empty())
    {
      cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);

      // Pose estimation

      cv::Vec3d rvec, tvec;
      int valid = cv::aruco::estimatePoseBoard(markerCorners, markerIds, board, cameraMatrix, distCoeffs, rvec, tvec);
      if (valid > 0)
        cv::aruco::drawAxis(outputImage, cameraMatrix, distCoeffs, rvec, tvec, 0.1);

std::cout << rvec << "; " << tvec << std::endl;
      // std::vector<cv::Vec3d> rvecs, tvecs;
      // cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.04, cameraMatrix, distCoeffs, rvecs, tvecs);
      // for (int i = 0; i < rvecs.size(); ++i)
      // {
      //   auto rvec = rvecs[i];
      //   auto tvec = tvecs[i];
      //   cv::aruco::drawAxis(outputImage, cameraMatrix, distCoeffs, rvec, tvec, 0.01);
      // }
    }

    cv::imshow("view", outputImage);
    cv::waitKey(30);

    rate_limiter.sleep();
  }
}