#ifndef AVERAGEDISTANCEGENERATOR_HPP
#define AVERAGEDISTANCEGENERATOR_HPP

#include "KNNMappingGaussian.hpp"

#include <iostream>
#include <string>
#include <cmath>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/octree/octree_search.h>
#include "IOManager.hpp"
#include "ThreadPool.h"

class AverageDistanceGenerator {
public:
    explicit AverageDistanceGenerator(const cv::FileStorage &fSettings);

    void Run();

private:
    const std::string poseFileName;
    const std::string downSampledPointCloudDir;
    const std::string sparsePointCloudDir;
    const std::string parameterListDir;
    const std::string allDistanceDir;
    const std::string frameDistanceDir;
    const bool isDownSampledPointCloud;

    const int factor;
    const int fLStep;
    const int fLMin;
    const int fLMax;
    const int kStep;
    const int kMin;
    const int kMax;
    const int resSStep;
    const int resSMin;
    const int resSMax;

    size_t processedNum = 0;
    size_t totalNum{};

    std::mutex PLock;
    std::mutex threadLock;

    void GenerateAverageDistance(std::ifstream &PList, size_t &PNum);

    void ThreadGenerateAverageDistance(const std::string &PNumber, const std::string &PParameter, const double &resS,
                                       const double &fL);

    void CalculateDistance(const std::string &PNumber, const double &resS, const double &fL,
                           const std::string &pointCloudDir);
};

#endif //AVERAGEDISTANCEGENERATOR_HPP
