#include "ros/ros.h"
#include "camera_wrapper.h"
#include "camera.h"
#include "calibration.h"

int main(int argc, char **argv)
{

  ros::init(argc, argv, "camera_calibration");

  ros::NodeHandle nh;

  std::shared_ptr<CameraInterface> cam15(new Camera("view1", true));
  std::shared_ptr<CameraWrapper> rosPtr(new CameraWrapper(nh, cam15, 17));
  // std::shared_ptr<ExtrinsicCalibrationInterface> ptr(new ExtrinsicCalibration(cameras));

  ros::spin();

  /**
   * Let's cleanup everything, shutdown ros and join the thread
   */
  ros::shutdown();

  return 0;
}
