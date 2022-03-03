#include "camera.h"

cv::Mat rvectvecToTransformation(cv::Vec3d &rvec, cv::Vec3d &tvec)
{
  cv::Mat R;              // Rotation matrix
  cv::Rodrigues(rvec, R); // Convert rvec (1x3) to rotation matrix (3x3)

  cv::Mat tf = (cv::Mat_<double>(4, 4) << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), tvec[0],
                R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), tvec[1],
                R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), tvec[2],
                0, 0, 0, 1);

  return tf;
}

Camera::Camera(std::string name, bool show)
{
  show_ = show;
  name_ = name;
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

// Read ArUco board configuration in a yaml file
bool Camera::Read_ArUco_YAML(const std::string &fileName, cv::Ptr<cv::aruco::Dictionary> &dictionary,
                             std::vector<int> &ids, std::vector<std::vector<cv::Point3f>> &objPoints)
{
  // Default dictionary (not using any others)
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

// Set camera matrix (instrinsic) for the camera
void Camera::setCameraMatrix(cv::Mat camera_matrix)
{
  std::unique_lock<std::mutex> lck(camera1_.i_mtx);
  camera1_.instrinsic = camera_matrix;
}

// Set image
void Camera::setCameraImage(cv::Mat input_image)
{
  std::unique_lock<std::mutex> lck(camera1_.img_mtx);
  camera1_.image = input_image;
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
    std::unique_lock<std::mutex> lck1(camera1_.img_mtx);
    cv::Mat input_image = camera1_.image; // Create a copy of image matrix
    lck1.unlock();

    std::unique_lock<std::mutex> lck2(camera1_.i_mtx);
    cv::Mat instrinsic = camera1_.instrinsic; // Create a copy of camera matrix
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

      cv::Mat tf = rvectvecToTransformation(rvec, tvec); // Transformation homogeneous, combine rotation and translation

      std::unique_lock<std::mutex> lck3(tf_mtx_);
      tf_ = tf; // Save to member variable tf
      lck3.unlock();

      if (show_)
      {
        cv::imshow(name_, output_image);
        cv::waitKey(30);
      }
    }

    if (!running_)
      break;
  }
}