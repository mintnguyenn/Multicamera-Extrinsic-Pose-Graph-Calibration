#include "yaml_reader.h"

std::vector<cv::Mat> yaml::Read_Intrinsic(const std::string filename){
    std::vector<cv::Mat> intrinsic;

    YAML::Node config = YAML::LoadFile(filename);
    if (config["intrinsic"]){
        std::vector<std::vector<double>> vec = config["intrinsic"].as<std::vector<std::vector<double>>>();
        for (unsigned int i = 0; i < vec.size(); i++){
            cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << vec.at(i).at(0), 0, vec.at(i).at(2),
                                     0, vec.at(i).at(4), vec.at(i).at(5),
                                     0, 0, 1);
            intrinsic.push_back(camera_matrix);
        }
    }
    // std::cout << intrinsic.size() << std::endl;
    return intrinsic;
}

bool yaml::Read_ArUco(const std::string &fileName, cv::Ptr<cv::aruco::Dictionary> &dictionary,
                      std::vector<int> &ids, std::vector<std::vector<cv::Point3f>> &objPoints){
    // Default dictionary (not using any others)
    dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100);

    YAML::Node config = YAML::LoadFile(fileName);

    std::vector<std::vector<cv::Point3f>> markerConers;
    if (config["objPoints"]){
        std::vector<std::vector<double>> objPoints_ = config["objPoints"].as<std::vector<std::vector<double>>>();
        markerConers.resize(objPoints_.size());
        for (unsigned int i = 0; i < objPoints_.size(); i++){
            if (!objPoints_.empty())
                for (unsigned int j = 0; j < objPoints_.at(i).size(); j += 3){
                    cv::Point3f point;
                    point.x = objPoints_.at(i).at(j);
                    point.y = objPoints_.at(i).at(j + 1);
                    point.z = objPoints_.at(i).at(j + 2);
                    markerConers.at(i).push_back(point);
                }
        }
        objPoints = markerConers;

        for (unsigned i = 28; i < objPoints.size(); i++){
            std::iter_swap(objPoints.at(i).begin(), objPoints.at(i).begin() + 1);
            std::iter_swap(objPoints.at(i).begin() + 2, objPoints.at(i).begin() + 3);
        }
    }
    if (config["ids"])
        ids = config["ids"].as<std::vector<int>>();

    if (!objPoints.empty() && !ids.empty()) return true;
    else return false;
}