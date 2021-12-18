#ifndef DISTRIBUTIONGENERATOR_HPP
#define DISTRIBUTIONGENERATOR_HPP

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

class DistributionGenerator {
public:
    explicit DistributionGenerator(const cv::FileStorage &fSettings);

    void Run();

private:
    const std::string poseFileName;
    const std::string downSampledPointCloudDir;
    const std::string sparsePointCloudDir;
    const std::string parameterListDir;
    const std::string distributionDir;
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

    std::ofstream resultFile;

    size_t processedNum = 0;
    size_t totalNum{};

    std::mutex PLock;
    std::mutex threadLock;

    void GenerateDistribution(std::ifstream &PList, size_t &PNum);

    void ThreadGenerateDistribution(const std::string &PNumber, const std::string &PParameter, const int &k,
                                    const double &resS, const double &fL);

    static void AddPointToDistribution(const double &d, size_t &N, double &sum, double &mu);

    static void UpdateStandardDeviation(const size_t &N, const double &sum, double &sigma);

    void CalculateDistribution(const int &k, const double &resS, const double &fL, const std::string &pointCloudDir,
                               double &mu, double &sigma);
};

#endif //DISTRIBUTIONGENERATOR_HPP
