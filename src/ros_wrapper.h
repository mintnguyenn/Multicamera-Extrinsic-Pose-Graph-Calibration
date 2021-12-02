#ifndef ROS_WRAPPER_H
#define ROS_WRAPPER_H

#include "ros/ros.h"

#include <sstream>
#include <iostream>
#include <string>

#include <thread>
#include <mutex>
#include <deque>
#include <atomic>

//! The class we have developed included here in our node

/*!
 *  \brief     Ros Wrapper Class
 *  \details
 *  This class is used as a wrapper for ros of the project
 *  \author    Huy Nhat Minh Nguyen - SID 13734569
 *  \date      2021-11-11
 */

class RosWrapper{

public:
   /*! @brief RosWrapper constructor.
   *
   *  Will take the node handle and initialise the callbacks and internal variables
   */
    RosWrapper(){};

   /*! @brief RosWrapper destructor.
   *
   *  Will tear down the object
   */
    ~RosWrapper();

    /*! @brief seperate thread.
    *
    *  The main processing thread that will run continously and utilise the data
    *  When data needs to be combined then running a thread seperate to callback will gurantee data is processed
    */
    void seperateThread();


private:

};

#endif //ROS_WRAPPER_H