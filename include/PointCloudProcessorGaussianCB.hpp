#ifndef POINTCLOUDPROCESSORGAUSSIANCB_HPP
#define POINTCLOUDPROCESSORGAUSSIANCB_HPP


#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>
#include "IOManager.hpp"
#include "ThreadPool.h"

class PointCloudProcessorGaussianCB {
public:
    explicit PointCloudProcessorGaussianCB(const cv::FileStorage &fSettings);

    void Run();

private:
    const std::string poseFileName;
    const std::string downSampledPointCloudDir;
    const std::string sparsePointCloudDir;
    const std::string distributionDir;
    std::string distributionFileName;
    const std::string referenceMapDir;
    const int layoutFlag;
    const std::string mapDir;
    const std::string mapComparisonDir;
    const std::string parameterListDir;
    const std::string testParameterListDir;
    const bool isDownSampledPointCloud;
    const bool isOctoMap;
    const bool writeMapFile;
    const double resReferenceMap;
    octomap::ColorOcTree referenceMap;

    std::ofstream resultFile;

    size_t processedNum = 0;
    size_t totalNum{};

    std::mutex CBLock;
    std::mutex threadLock;

    void CompareMap(std::ifstream &CBList, size_t &CBNum);

    void ThreadCompareMap(const std::string &PNumber, const std::string &PParameter, const std::string &MNumber,
                          const std::string &MParameter);

    void GetDistribution(const std::string &PNumber, const std::string &MParameter, double &mu, double &sigma);

    static int FindNthOccur(std::string str, char ch, int N);
};

#endif //POINTCLOUDPROCESSORGAUSSIANCB_HPP
