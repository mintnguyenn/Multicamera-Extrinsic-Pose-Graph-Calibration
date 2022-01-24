// roslaunch realsense2_camera rs_multiple_devices.launch serial_no_camera1:=801212071175 serial_no_camera2:=817612071347

#include "ros_wrapper.h"

RosWrapper::RosWrapper(ros::NodeHandle nh) : nh_(nh)
{

  sub1_ = nh_.subscribe("camera/color/image_raw", 1000, &RosWrapper::camera1Callback, this);
}

RosWrapper::~RosWrapper() {}

void RosWrapper::camera1Callback(const sensor_msgs::ImageConstPtr &msg)
{
  std::unique_lock<std::mutex> lck(mtx1_);
  img1_ = msg;
  lck.unlock();
}

bool Read_ArUco_YAML(const std::string &fileName, cv::Ptr<cv::aruco::Dictionary> dictionary,
                     std::vector<int> &ids, std::vector<std::vector<cv::Point3f>> &objPoints)
{
  // the default dictionary (not using any others)
  dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100);

  YAML::Node config = YAML::LoadFile(fileName);

  std::vector<std::vector<cv::Point3f>> markerConers;
  if (config["objPoints"])
  {
    std::vector<std::vector<double>> objPointss = config["objPoints"].as<std::vector<std::vector<double>>>();
    markerConers.resize(objPointss.size());
    for (unsigned int i = 0; i < objPointss.size(); i++)
    {
      if (!objPointss.empty())
        for (unsigned int j = 0; j < objPointss.at(i).size(); j += 3)
        {
          cv::Point3f point;
          point.x = objPointss.at(i).at(j);
          point.y = objPointss.at(i).at(j + 1);
          point.z = objPointss.at(i).at(j + 2);
          markerConers.at(i).push_back(point);
        }
    }
    objPoints = markerConers;

    for (unsigned i = 28; i < objPoints.size(); i++)
    {
      std::iter_swap(objPoints.at(i).begin(), objPoints.at(i).begin() + 1);
      std::iter_swap(objPoints.at(i).begin() + 2, objPoints.at(i).begin() + 3);
    }
  }

  if (config["ids"])
    ids = config["ids"].as<std::vector<int>>();

  return true;
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

  const std::string fileName = "/home/mintnguyen/Documents/multi-cameras-calibration/aruco-board-markers.yaml";
  Read_ArUco_YAML(fileName, dictionary, ids, objPoints);

  // for (auto element : objPoints)
  // {
  //   for (auto e : element)
  //   {
  //     std::cout << e << " ";
  //   }
  //   std::cout << std::endl;
  // }

  while (ros::ok())
  {
    sensor_msgs::ImageConstPtr img;
    std::unique_lock<std::mutex> lck(mtx1_);
    img = img1_;
    lck.unlock();

    cv::Mat inputImage = cv_bridge::toCvShare(img, "bgr8")->image;
    // cv::Mat inputImage = cv::imread("/home/mintnguyen/Pictures/aruco1.png", 1);

    // Marker detection
    std::vector<int> markerIds; // Create a vector contains marker id
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100);
    cv::Ptr<cv::aruco::GridBoard> board = cv::aruco::GridBoard::create(7, 4, 0.04, 0.02, dictionary);
    cv::Ptr<cv::aruco::Board> board2 = cv::aruco::Board::create(objPoints, dictionary, ids);

    // for (unsigned int i = 0; i < 28; i++)
    // {
    //   std::cout << "ID: " << i << std::endl;
    //   std::cout << std::setprecision(2) << board->objPoints.at(i) << std::endl;
    //   std:: cout << "---" << std::endl;
    //   std::cout << std::setprecision(2) << board2->objPoints.at(i) << std::endl;
    //   std::cout << std::endl;
    // }

    cv::aruco::detectMarkers(inputImage, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);

    cv::Mat outputImage = inputImage.clone();

    if (!markerIds.empty())
    {
      cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);

      // Pose estimation

      cv::Vec3d rvec, tvec;
      int valid = cv::aruco::estimatePoseBoard(markerCorners, markerIds, board2, cameraMatrix, distCoeffs, rvec, tvec);
      if (valid > 0)
        cv::aruco::drawAxis(outputImage, cameraMatrix, distCoeffs, rvec, tvec, 0.1);
    }

    cv::imshow("view", outputImage);
    cv::waitKey(30);

    rate_limiter.sleep();
  }
}