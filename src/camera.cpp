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

cv::Vec4d rvecToQuaternion(cv::Vec3d rvec){
  
  cv::Mat R;
  cv::Rodrigues(rvec, R);
  Eigen::Matrix3d e_R;
  e_R << R.at<double>(0), R.at<double>(1), R.at<double>(2), 
         R.at<double>(3), R.at<double>(4), R.at<double>(5), 
         R.at<double>(6), R.at<double>(7), R.at<double>(8);

  Eigen::Quaternion<double> quaternion(e_R);

  cv::Vec4d q;
  q[0] = quaternion.x(); q[1] = quaternion.y(); q[2] = quaternion.z(); q[3] = quaternion.w();

  return q; 
}

Camera::Camera(bool show, unsigned int index) : show_(show), index_(index)
{
  const std::string aruco_board_markers = "/home/mintnguyen/Documents/multi-cameras-calibration/yaml/aruco-board-markers.yaml";
  yaml::Read_ArUco(aruco_board_markers, board_.dictionary, board_.ids, board_.objPoints);

  const std::string fileName = "/home/mintnguyen/Documents/multi-cameras-calibration/yaml/camera_info.yaml";
  std::vector<cv::Mat> camera_info_vec = yaml::Read_Intrinsic(fileName);

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

// Set image
void Camera::setCameraImage(cv::Mat input_image)
{
  std::unique_lock<std::mutex> lck(camera_.img_mtx);
  camera_.image = input_image;

  ready_ = true;
}

void Camera::boardDetection(cv::Mat image, cv::Mat intrinsic, cv::Vec4d &quaternion, cv::Vec3d &tvec){

  // cv::Quate
  if (!image.empty() && !intrinsic.empty()){

    cv::Mat distCoeffs;
    // Marker detection
    std::vector<int> markerIds; // Create a vector contains marker id
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
    cv::Ptr<cv::aruco::Board> board = cv::aruco::Board::create(board_.objPoints, board_.dictionary, board_.ids);
    cv::aruco::detectMarkers(image, board_.dictionary, markerCorners, markerIds, parameters, rejectedCandidates);

    std::vector<cv::Point3f> objPoints;
    std::vector<cv::Point2f> imgPoints;
    cv::aruco::getBoardObjectAndImagePoints(board, markerCorners, markerIds, objPoints, imgPoints);
    cv::Vec3d rvec;
    if (!objPoints.empty() && !imgPoints.empty())
      cv::solvePnP(objPoints, imgPoints, intrinsic, distCoeffs, rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);

    quaternion = rvecToQuaternion(rvec);
  }
  
  else {
    quaternion = 0;
    tvec = 0;
  };
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

    if (!input_image.empty() && !instrinsic.empty()){
      cv::Mat output_image = input_image.clone();

      cv::Mat distCoeffs;
      // Marker detection
      std::vector<int> markerIds; // Create a vector contains marker id
      std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
      cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
      cv::Ptr<cv::aruco::Board> board = cv::aruco::Board::create(board_.objPoints, board_.dictionary, board_.ids);
      cv::aruco::detectMarkers(input_image, board_.dictionary, markerCorners, markerIds, parameters, rejectedCandidates);

      if (!markerIds.empty())
        cv::aruco::drawDetectedMarkers(output_image, markerCorners, markerIds);

      std::vector<cv::Point3f> objPoints;
      std::vector<cv::Point2f> imgPoints;
      cv::aruco::getBoardObjectAndImagePoints(board, markerCorners, markerIds, objPoints, imgPoints);
      cv::Vec3d rvec, tvec;
      if (!objPoints.empty() && !imgPoints.empty()){
        cv::solvePnP(objPoints, imgPoints, instrinsic, distCoeffs, rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
        cv::aruco::drawAxis(output_image, instrinsic, distCoeffs, rvec, tvec, 0.1);
      }

      cv::Mat tf = rvectvecToTransformationMatrix(rvec, tvec); // Transformation homogeneous, combine rotation and translation

      if (show_){
        std::cout << "Camera " << index_ << ": " << std::endl;
        rvecToQuaternion(rvec);
      }

      std::unique_lock<std::mutex> lck3(tf_mtx_);
      tf_ = tf; // Save to member variable tf
      lck3.unlock();

      if (show_){
        std::string name = std::to_string(index_);
        cv::imshow(name, output_image);
        cv::waitKey(10);
      }
    }

    if (!running_)
      break;
  }
}

// Set camera matrix (instrinsic) for the camera
void Camera::setCameraMatrix(cv::Mat camera_matrix){
  std::unique_lock<std::mutex> lck(camera_.i_mtx);
  camera_.instrinsic = camera_matrix;
}

cv::Mat Camera::getTransformationMatrix(){
  std::unique_lock<std::mutex> lck(tf_mtx_);
  cv::Mat tf = tf_;
  lck.unlock();

  return tf;
}