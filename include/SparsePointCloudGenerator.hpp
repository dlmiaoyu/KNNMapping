#ifndef SPARSEPOINTCLOUDGENERATOR_HPP
#define SPARSEPOINTCLOUDGENERATOR_HPP

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <boost/filesystem.hpp>
#include "IOManager.hpp"
#include "ThreadPool.h"

class SparsePointCloudGenerator {
public:
    explicit SparsePointCloudGenerator(const cv::FileStorage &fSettings);

    void Run();

private:
    const std::string sourceSparsePointCloudDir;
    const std::string sparsePointCloudDir;
    const std::string poseFileName;
    const std::string parameterListDir;
    const std::string runTimeDir;

    std::ofstream singleRunTimeFile;

    size_t processedNum = 0;
    size_t totalNum{};

    std::mutex SLock;
    std::mutex threadLock;
    void GenerateSparsePointCloud(std::ifstream &SList, const size_t &SNum);

    void ThreadGenerateSparsePointCloud(const std::string &SNumber, const std::string &SParameter);
};

#endif //SPARSEPOINTCLOUDGENERATOR_HPP
