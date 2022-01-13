#include <string>
#include <iostream>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <unistd.h>

/*
 * Read the ArUco board parameters from a yaml file. This file should contain the IDs present on the board and their respective corners.
 *
 * INPUTS:
 *      fileName - Name of yaml file [string]
 *      dictionary - dictionary from which the ArUco tags are from (should be cv::aruco::DICT_4X4_100)
 *      ids - ID of ArUco markers
 *      objPoints - The corner points for each of the markers. The order goes as follows [top-left, top-right, bottom-right, bottom-left].
 *                  Each point is of type cv::Point3f comprised of the X,Y,Z coordinates.
 *
 *
 * OUTPUT:
 *      if the YAML file was successfully read.
 */
bool Read_ArUco_YAML(const std::string &fileName, cv::Ptr<cv::aruco::Dictionary> dictionary,
                     std::vector<int> &ids, std::vector<std::vector<cv::Point3f>> &objPoints)
{
    // the default dictionary (not using any others)
    dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100);

    YAML::Node config = YAML::LoadFile(fileName);

    std::vector<std::vector<cv::Point3f>> markerConers;
    if (config["objPoints"])
    {
        std::vector<std::vector<double>> objPointss = config["objPoints"].as<std::vector<std::vector<double>>>();
        markerConers.resize(objPointss.size());
        for (unsigned int i = 0; i < objPointss.size(); i++)
        {
            if (!objPointss.empty())
                for (unsigned int j = 0; j < objPointss.at(i).size(); j += 3)
                {
                    cv::Point3f point;
                    point.x = objPointss.at(i).at(j);
                    point.y = objPointss.at(i).at(j + 1);
                    point.z = objPointss.at(i).at(j + 2);
                    markerConers.at(i).push_back(point);
                }
        }
        objPoints = markerConers;
        // std::cout << markerConers.at(0).size() << std::endl;
    }

    if (config["ids"])
        ids = config["ids"].as<std::vector<int>>();

    return true;
}

int main(int argc, char **argv)
{
    printf("Current working directory: %s\n", get_current_dir_name());
    const std::string fileName = "/home/mintnguyen/Documents/multi-cameras-calibration/aruco_yaml/aruco-board-markers.yaml";
    std::cout << "Reading in YAML file " << fileName << std::endl;

    // local board variables
    cv::Ptr<cv::aruco::Dictionary> dictionary;
    std::vector<int> ids;
    std::vector<std::vector<cv::Point3f>> objPoints;

    if (Read_ArUco_YAML(fileName, dictionary, ids, objPoints))
        std::cout << "Read YAML successfully" << std::endl;
    else
        std::cout << "Could not read YAML" << std::endl;

    for (auto element : objPoints)
    {
        for (auto e : element)
        {
            std::cout << e << " ";
        }
        std::cout << std::endl;
    }

    return 0;
}
