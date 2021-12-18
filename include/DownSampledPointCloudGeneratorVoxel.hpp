#ifndef DOWNSAMPLEDPOINTCLOUDGENERATORVOXEL_HPP
#define DOWNSAMPLEDPOINTCLOUDGENERATORVOXEL_HPP

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <fstream>
#include <sstream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/octree/octree_search.h>
#include <pcl/octree/octree_pointcloud_density.h>
#include <pcl/octree/octree_impl.h>
#include <pcl/common/transforms.h>
#include <pcl/common/io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <random>
#include <boost/filesystem.hpp>
#include "IOManager.hpp"
#include "ThreadPool.h"

class DownSampledPointCloudGeneratorVoxel {
public:
    explicit DownSampledPointCloudGeneratorVoxel(const cv::FileStorage &fSettings);

    void Run();

private:
    const std::string imageDir;
    const std::string timeStampFileName;
    const std::string downSampledPointCloudDir;
    const std::string poseFileName;
    const std::string parameterListDir;
    const std::string runTimeDir;

    std::ofstream singleRunTimeFile;

    cv::Size imageSize;
    cv::Mat K1;
    cv::Mat K2;
    cv::Mat D1;
    cv::Mat D2;
    cv::Mat R;
    cv::Mat T;

    const double dist;
    const double resMap;
    double f;

    //Stereo rectification
    cv::Mat R1;
    cv::Mat R2;
    cv::Mat P1;
    cv::Mat P2;
    cv::Mat Q;

    size_t processedNum = 0;
    size_t totalNum{};
    std::mutex DLock;
    std::mutex threadLock;

    void GenerateDownSampledPointCloud(std::ifstream &DList, const size_t &DNum);

    void ThreadGenerateDownSampledPointCloud(const std::string &DNumber, const std::string &DParameter);

    void ParameterRectification();

    static cv::Mat
    GetDisparitySGBM(const cv::Mat &leftColorImage, const cv::Mat &rightColorImage, const int &mD, const int &nD,
                     const int &bS, const int &DP1, const int &DP2, const int &dMD, const int &pFC, const int &uR,
                     const int &sWS, const int &sR, const bool &mode);

    double GetDisparityLimit(const double &zL);

    void GenerateDensePointCloud(const cv::Mat &disparity, const double &dL,
                                 pcl::PointCloud<pcl::PointXYZ>::Ptr &densePointCloud);
};

#endif //DOWNSAMPLEDPOINTCLOUDGENERATORVOXEL_HPP
