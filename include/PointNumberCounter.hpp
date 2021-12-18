#ifndef POINTNUMBERCOUNTER_HPP
#define POINTNUMBERCOUNTER_HPP

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <fstream>
#include <sstream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <boost/filesystem.hpp>

#include "IOManager.hpp"

class PointNumberCounter {
public:
    explicit PointNumberCounter(const cv::FileStorage &fSettings);

    void Run();

private:
    const std::string poseFileName;
    const std::string parameterListDir;
    const std::string downSampledPointCloudDir;
    const std::string sparsePointCloudDir;
    const std::string pointNumberDir;
    const bool isDownSampledPointCloud;

    const double fL;

    std::ofstream resultFile;

    void GetPointNumber(const std::string &PNumber);
};

#endif //POINTNUMBERCOUNTER_HPP