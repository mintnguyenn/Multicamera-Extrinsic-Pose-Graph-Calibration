#include "ros_wrapper.h"

RosWrapper::~RosWrapper(){}

void RosWrapper::seperateThread(){
    ros::Duration(1.0).sleep(); // Sleep for one seconds

    ros::Rate rate_limiter(60);
    while(ros::ok()){


        rate_limiter.sleep();
    }
}