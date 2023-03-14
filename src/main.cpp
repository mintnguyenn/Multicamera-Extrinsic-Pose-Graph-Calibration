#include "ros/ros.h"
#include "camera_wrapper.h"
#include "camera.h"
// #include "calibration.h"
// #include "wrapper.h"
#include <vector>

#include <iostream>
#include <fstream>


// Function to write vector to csv file 
void writeVectorToCSV(std::string filename, std::vector<BoardCameraExtrinsic> &vec) 
{ 
   std::cout << "writing csv ..." <<  std::endl;
    std::ofstream outfile; 
    outfile.open(filename); 

    for (int i = 0; i < vec.size(); i++) { 

        // If the element is the last in the vector, don't add a comma after it.  
        if (i == vec.size() - 1) {  

            outfile << vec.at(i).camIndex << "," << vec.at(i).stamp;
            
            auto vecT = vec.at(i).tvec;
            for (int i = 0; i < 3; i++)
                outfile << "," << vecT(i);

            auto vecQ = vec.at(i).quaternion;
            for (int i = 0; i < 4; i++)
                outfile << "," << vecQ(i);

            outfile << "\n";  

        } else {
            outfile << vec.at(i).camIndex << "," << vec.at(i).stamp;
            
            auto vecT = vec.at(i).tvec;
            for (int i = 0; i < 3; i++)
                outfile << "," << vecT(i);

            auto vecQ = vec.at(i).quaternion;
            for (int i = 0; i < 4; i++)
                outfile << "," << vecQ(i);

            outfile << "," << "\n";  

            // outfile << vec.at(i).camIndex << "," << vec.at(i).stamp << "," << vec.at(i).tvec << "," << vec.at(i).quaternion << "," << "\n";  

        }  

    }  

    outfile.close(); 
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "camera_calibration");

  // ros process input argument
  ros::NodeHandle nh;
  // int index = 11;
  // nh.getParam("id", index);
  unsigned int camIndexStart = 7;
  unsigned int camIndexEnd = 22;

  std::vector<std::shared_ptr<CameraWrapper>> camWrappers;
  std::vector<std::shared_ptr<CameraInterface>> cams;

  for (int i = camIndexStart; i <= camIndexEnd; i++){
    std::shared_ptr<CameraInterface> cam(new Camera(i));
    cams.push_back(cam);

    std::shared_ptr<CameraWrapper> cam_wrp(new CameraWrapper(nh, i));
    cam_wrp->setCamera(cam);
    camWrappers.push_back(cam_wrp);
  }

  // std::shared_ptr<CameraInterface> cam(new Camera(index));
  // std::shared_ptr<CameraWrapper> cam_wrp(new CameraWrapper(nh, index));
  // cam_wrp->setCamera(cam);

  ros::spin();

  std::vector<BoardCameraExtrinsic> allData;

  int i = camIndexStart;
  for (auto c : camWrappers){
    std::vector<BoardCameraExtrinsic> curData = c->getcopyboardextrinsicdata();
    std::cout << "Camera " << i << " has " << curData.size() << " poses" << std::endl;
    i++;
    allData.insert(std::end(allData), std::begin(curData), std::end(curData));
  }

  std::cout << "Number of estimated poses across all cameras: " << allData.size() << std::endl;

  writeVectorToCSV("test.csv", allData);
    // allData.push_back();
    // a.insert(std::end(a), std::begin(b), std::end(b));

// function to write vector to csv file

  ros::shutdown();

  return 0;
}

// function that loads a rosbag

// function that determines the closest ros message using time between two ros topics