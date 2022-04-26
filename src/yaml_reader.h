#ifndef YAML_READER_H
#define YAML_READER_H

#include <vector>
#include <string>
#include <opencv2/aruco/dictionary.hpp>
#include <yaml-cpp/yaml.h>
#include <unistd.h>

namespace yaml
{
    std::vector<cv::Mat> Read_Intrinsic(const std::string filename);

    bool Read_ArUco(const std::string &fileName, cv::Ptr<cv::aruco::Dictionary> &dictionary,
                    std::vector<int> &ids, std::vector<std::vector<cv::Point3f>> &objPoints);
}

#endif