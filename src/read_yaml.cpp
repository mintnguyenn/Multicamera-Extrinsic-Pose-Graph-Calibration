#include <string>
#include <opencv2/aruco/dictionary.hpp>
#include <yaml-cpp/yaml.h>
#include <unistd.h>

bool Read_ArUco_YAML(const std::string &fileName, cv::Ptr<cv::aruco::Dictionary> &dictionary,
                     std::vector<int> &ids, std::vector<std::vector<cv::Point3f>> &objPoints)
{

    // Default dictionary (not using any others)
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

        for (unsigned i = 28; i < objPoints.size(); i++)
        {
            std::iter_swap(objPoints.at(i).begin(), objPoints.at(i).begin() + 1);
            std::iter_swap(objPoints.at(i).begin() + 2, objPoints.at(i).begin() + 3);
        }
    }
    if (config["ids"])
        ids = config["ids"].as<std::vector<int>>();

    return true;
}