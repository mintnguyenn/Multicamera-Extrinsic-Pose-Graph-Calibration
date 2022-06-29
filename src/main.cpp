#include "ros/ros.h"
#include "camera_wrapper.h"
#include "camera.h"
#include "calibration.h"
#include "wrapper.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "camera_calibration");

  ros::NodeHandle nh;
  const unsigned int index = 10;

  // std::shared_ptr<Wrapper> cam(new Wrapper(nh));
  std::shared_ptr<CameraInterface> cam(new Camera(index));
  std::shared_ptr<CameraWrapper> cam_wrp(new CameraWrapper(nh, index));
  cam_wrp->setCamera(cam);

  ros::spin();

  /**
   * Let's cleanup everything, shutdown ros and join the thread
   */
  ros::shutdown();

  return 0;
}