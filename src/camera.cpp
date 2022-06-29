#include "camera.h"

cv::Mat rvectvecToTransformationMatrix(cv::Vec3d &rvec, cv::Vec3d &tvec)
{
  cv::Mat R;              // Rotation matrix
  cv::Rodrigues(rvec, R); // Convert rvec (1x3) to rotation matrix (3x3)

  cv::Mat tf = (cv::Mat_<double>(4, 4) << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), tvec[0],
                R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), tvec[1],
                R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), tvec[2],
                0, 0, 0, 1);

  return tf;
}

cv::Vec4d rvecToQuaternion(cv::Vec3d rvec)
{
  cv::Mat R;
  cv::Rodrigues(rvec, R);
  Eigen::Matrix3d e_R;
  e_R << R.at<double>(0), R.at<double>(1), R.at<double>(2),
      R.at<double>(3), R.at<double>(4), R.at<double>(5),
      R.at<double>(6), R.at<double>(7), R.at<double>(8);

  Eigen::Quaternion<double> quaternion(e_R);

  cv::Vec4d q;
  q[0] = quaternion.x();
  q[1] = quaternion.y();
  q[2] = quaternion.z();
  q[3] = quaternion.w();

  return q;
}

Camera::Camera(unsigned int index) : index_(index)
{
  const std::string aruco_board_markers = "/home/mintnguyen/Documents/Multicamera-Extrinsic-Pose-Graph-Calibration/yaml/aruco-board-markers.yaml";
  yaml::Read_ArUco(aruco_board_markers, aruco_board_.dictionary, aruco_board_.ids, aruco_board_.objPoints);

  const std::string fileName = "/home/mintnguyen/Documents/Multicamera-Extrinsic-Pose-Graph-Calibration/yaml/camera_info.yaml";
  std::vector<cv::Mat> intrinsic_vectors = yaml::Read_Intrinsic(fileName);
  intrinsic_ = intrinsic_vectors.at(index);

  runThreads();

  // Initialise the ArUco board configuration
  parameters_ = cv::aruco::DetectorParameters::create();
  board_ = cv::aruco::Board::create(aruco_board_.objPoints, aruco_board_.dictionary, aruco_board_.ids);
}

Camera::~Camera()
{
  running_ = false;
  for (auto &t : threads_)
    t.join(); // Join threads
}

void Camera::runThreads(void)
{
  running_ = true;                                                                 // Indicate the threads should be running
  threads_.push_back(std::thread(&CameraInterface::boardEstimationTesting, this)); // Create the thread and push it to our vector of threads
}

// Set image
void Camera::setCameraImage(cv::Mat input_image)
{
  std::unique_lock<std::mutex> lck(img_mtx_);
  image_ = input_image;

  // ready_ = true;
}

bool Camera::arucoBoardDetection(cv::Mat image, cv::Vec4d &quaternion, cv::Vec3d &tvec)
{
  if (!image.empty() && !intrinsic_.empty())
  {
    cv::Mat distCoeffs;
    // Marker detection
    std::vector<int> markerIds; // Create a vector contains marker id
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
    // cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
    // cv::Ptr<cv::aruco::Board> board = cv::aruco::Board::create(board_.objPoints, board_.dictionary, board_.ids);
    cv::aruco::detectMarkers(image, aruco_board_.dictionary, markerCorners, markerIds, parameters_, rejectedCandidates);

    std::vector<cv::Point3f> objPoints;
    std::vector<cv::Point2f> imgPoints;
    cv::aruco::getBoardObjectAndImagePoints(board_, markerCorners, markerIds, objPoints, imgPoints);

    cv::Vec3d rvec;
    if (!objPoints.empty() && !imgPoints.empty())
      cv::solvePnP(objPoints, imgPoints, intrinsic_, distCoeffs, rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);

    quaternion = rvecToQuaternion(rvec);

    return true;
  }

  else
  {
    quaternion = 0;
    tvec = 0;
    return false;
  };
}

void Camera::boardEstimationTesting(void)
{
  while (running_)
  {
    std::unique_lock<std::mutex> lck1(img_mtx_);
    cv::Mat input_image = image_; // Create a copy of image
    lck1.unlock();

    if (!input_image.empty() && !intrinsic_.empty())
    {
      cv::Mat output_image = input_image.clone();

      cv::Mat distCoeffs;
      // Marker detection
      std::vector<int> markerIds; // Create a vector contains marker id
      std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
      // cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
      // cv::Ptr<cv::aruco::Board> board = cv::aruco::Board::create(board_.objPoints, board_.dictionary, board_.ids);
      cv::aruco::detectMarkers(input_image, aruco_board_.dictionary, markerCorners, markerIds, parameters_, rejectedCandidates);

      if (!markerIds.empty())
        cv::aruco::drawDetectedMarkers(output_image, markerCorners, markerIds);

      std::vector<cv::Point3f> objPoints;
      std::vector<cv::Point2f> imgPoints;
      cv::aruco::getBoardObjectAndImagePoints(board_, markerCorners, markerIds, objPoints, imgPoints);
      cv::Vec3d rvec, tvec;
      if (!objPoints.empty() && !imgPoints.empty())
      {
        cv::solvePnP(objPoints, imgPoints, intrinsic_, distCoeffs, rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
        cv::aruco::drawAxis(output_image, intrinsic_, distCoeffs, rvec, tvec, 0.1);
      }

      std::string name = std::to_string(index_);
      cv::imshow(name, output_image);
      cv::waitKey(10);
    }

    if (!running_)
      break;
  }
}