#include "ros/ros.h"
#include "camera_wrapper.h"
#include "camera.h"
#include "calibration.h"
#include "wrapper.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "camera_calibration");

  ros::NodeHandle nh;

  std::shared_ptr<Wrapper> cam(new Wrapper(nh));

  ros::spin();

  /**
   * Let's cleanup everything, shutdown ros and join the thread
   */
  ros::shutdown();

  return 0;
}
