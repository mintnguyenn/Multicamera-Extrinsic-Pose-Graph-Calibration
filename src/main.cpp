#include "ros/ros.h"
#include "ros_wrapper.h"


int main(int argc, char **argv)
{

  ros::init(argc, argv, "camera_calibration");

  ros::NodeHandle nh;
  std::shared_ptr<RosWrapper> rosPtr(new RosWrapper(nh));

  std::thread t(&RosWrapper::seperateThread,rosPtr);


  ros::spin();


  /**
   * Let's cleanup everything, shutdown ros and join the thread
   */
  ros::shutdown();
  t.join();

  return 0;
}

