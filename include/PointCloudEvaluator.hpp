#ifndef POINTCLOUDEVALUATOR_HPP
#define POINTCLOUDEVALUATOR_HPP

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>
#include <fstream>
#include "IOManager.hpp"
#include "MapComparer.hpp"
#include "ThreadPool.h"

class PointCloudEvaluator {
public:
    explicit PointCloudEvaluator(const cv::FileStorage &fSettings);

    void Run();

private:
    const std::string referenceMapDir;
    const int layoutFlag;
    const std::string pointCloudEvaluationDir;
    const std::string parameterListDir;
    const std::string downSampledPointCloudDir;
    const std::string sparsePointCloudDir;
    const std::string poseFileName;
    const bool isDownSampledPointCloud;

    octomap::ColorOcTree referenceMap;

    const double fL;
    double squaredMaxRange;

    std::ofstream resultFile;

    size_t processedNum = 0;
    size_t totalNum{};
    std::mutex PLock;
    std::mutex threadLock;

    void EvaluatePointCloud(std::ifstream &PList, size_t &PNum);

    void ThreadEvaluatePointCloud(const std::string &DNumber, const std::string &PParameter);
};

#endif //POINTCLOUDEVALUATOR_HPP
