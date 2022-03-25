#include "camera.h"
#include "read_yaml.cpp"

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

Camera::Camera(bool show, unsigned int index) : show_(show), index_(index)
{
  const std::string fileName = "/home/mintnguyen/Documents/multi-cameras-calibration/aruco-board-markers.yaml";
  Read_ArUco_YAML(fileName, board_config_.dictionary, board_config_.ids, board_config_.objPoints);
  runThreads();
}

Camera::~Camera()
{
  running_ = false;
  for (auto &t : threads_)
    t.join(); // Join threads
}

void Camera::runThreads(void)
{
  running_ = true;                                                               // Indicate the threads should be running
  threads_.push_back(std::thread(&CameraInterface::extrinsicCalibration, this)); // Create the thread and push it to our vector of threads
}

// Set camera matrix (instrinsic) for the camera
void Camera::setCameraMatrix(cv::Mat camera_matrix)
{
  std::unique_lock<std::mutex> lck(camera_.i_mtx);
  camera_.instrinsic = camera_matrix;
}

// Set image
void Camera::setCameraImage(cv::Mat input_image)
{
  std::unique_lock<std::mutex> lck(camera_.img_mtx);
  camera_.image = input_image;
}

cv::Mat Camera::getTransformationMatrix()
{
  std::unique_lock<std::mutex> lck(tf_mtx_);
  cv::Mat tf = tf_;
  lck.unlock();

  return tf;
}

void Camera::extrinsicCalibration()
{
  while (running_)
  {
    std::unique_lock<std::mutex> lck1(camera_.img_mtx);
    cv::Mat input_image = camera_.image; // Create a copy of image matrix
    lck1.unlock();

    std::unique_lock<std::mutex> lck2(camera_.i_mtx);
    cv::Mat instrinsic = camera_.instrinsic; // Create a copy of camera matrix
    lck2.unlock();

    if (!input_image.empty() && !instrinsic.empty())
    {
      cv::Mat output_image = input_image.clone();

      cv::Mat distCoeffs;
      // Marker detection
      std::vector<int> markerIds; // Create a vector contains marker id
      std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
      cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
      cv::Ptr<cv::aruco::Board> board = cv::aruco::Board::create(board_config_.objPoints, board_config_.dictionary, board_config_.ids);
      cv::aruco::detectMarkers(input_image, board_config_.dictionary, markerCorners, markerIds, parameters, rejectedCandidates);

      if (!markerIds.empty())
        cv::aruco::drawDetectedMarkers(output_image, markerCorners, markerIds);

      std::vector<cv::Point3f> objPoints;
      std::vector<cv::Point2f> imgPoints;
      cv::aruco::getBoardObjectAndImagePoints(board, markerCorners, markerIds, objPoints, imgPoints);
      cv::Vec3d rvec, tvec;
      if (!objPoints.empty() && !imgPoints.empty())
      {
        cv::solvePnP(objPoints, imgPoints, instrinsic, distCoeffs, rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
        cv::aruco::drawAxis(output_image, instrinsic, distCoeffs, rvec, tvec, 0.1);
      }

      cv::Mat tf = rvectvecToTransformationMatrix(rvec, tvec); // Transformation homogeneous, combine rotation and translation

      std::cout << "Camera " << index_ << ": " << tf << std::endl;

      std::unique_lock<std::mutex> lck3(tf_mtx_);
      tf_ = tf; // Save to member variable tf
      lck3.unlock();

      if (show_)
      {
        std::string name = std::to_string(index_);
        cv::imshow(name, output_image);
        cv::waitKey(10);
      }
    }

    if (!running_)
      break;
  }
}