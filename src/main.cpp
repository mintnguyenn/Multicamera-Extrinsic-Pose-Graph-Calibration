#include "ros/ros.h"
#include "camera_wrapper.h"
#include "camera.h"
#include "calibration.h"

int main(int argc, char **argv){

  // XInitThreads();

  ros::init(argc, argv, "camera_calibration");

  ros::NodeHandle nh;

  std::vector<std::shared_ptr<CameraInterface>> cameras;

  // for (unsigned int i = 15; i < 17; i++){
    std::shared_ptr<CameraWrapper> camera7(new CameraWrapper(nh, 7));
    // std::shared_ptr<CameraWrapper> camera8(new CameraWrapper(nh, 8));
    // cameras.push_back(camera);
  // }
  
  // std::shared_ptr<ExtrinsicCalibrationInterface> ptr(new ExtrinsicCalibration(cameras));
  ros::spin();

  /**
   * Let's cleanup everything, shutdown ros and join the thread
   */
  ros::shutdown();

  return 0;
}
