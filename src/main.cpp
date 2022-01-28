#include "ros/ros.h"
#include "ros_wrapper.h"
#include "calibration.h"

int main(int argc, char **argv)
{

  ros::init(argc, argv, "camera_calibration");

  ros::NodeHandle nh;

  std::shared_ptr<ExtrinsicCalibrationInterface> calibPtr(new ExtrinsicCalibration());
  std::vector<std::shared_ptr<ExtrinsicCalibrationInterface>> cameras;
  for (unsigned int i = 0; i < 1; i++)
  {
    // cameras.push_back(std::make_shared<ExtrinsicCalibrationInterface>(new ExtrinsicCalibration()));
  }
  std::shared_ptr<RosWrapper> rosPtr(new RosWrapper(nh, calibPtr));

  ros::spin();

  /**
   * Let's cleanup everything, shutdown ros and join the thread
   */
  ros::shutdown();

  return 0;
}
