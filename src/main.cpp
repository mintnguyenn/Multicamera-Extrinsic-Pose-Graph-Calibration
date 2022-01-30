#include "ros/ros.h"
#include "ros_wrapper.h"
#include "camera.h"
#include "calibration.h"

int main(int argc, char **argv)
{

  ros::init(argc, argv, "camera_calibration");

  ros::NodeHandle nh;

  std::shared_ptr<CameraInterface> cam1(new Camera("view1"));
  std::shared_ptr<CameraInterface> cam2(new Camera("view2"));
  std::vector<std::shared_ptr<CameraInterface>> cameras;
  unsigned int camera_number = 2;
  for (unsigned int i = 0; i < camera_number; i++)
  {
    std::shared_ptr<CameraInterface> cam1(new Camera("view1"));
    cameras.push_back(cam1);
  }
  std::shared_ptr<RosWrapper> rosPtr(new RosWrapper(nh, cameras));
  std::shared_ptr<ExtrinsicCalibrationInterface> ptr(new ExtrinsicCalibration(cameras));

  ros::spin();

  /**
   * Let's cleanup everything, shutdown ros and join the thread
   */
  ros::shutdown();

  return 0;
}
