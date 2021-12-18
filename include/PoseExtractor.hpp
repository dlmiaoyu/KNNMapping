#ifndef POSEEXTRACTOR_HPP
#define POSEEXTRACTOR_HPP

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <iostream>
#include <sstream>
#include <boost/filesystem.hpp>

class PoseExtractor {
public:
    explicit PoseExtractor(const cv::FileStorage &fSettings);

    void Run();

private:
    const int num;

    const std::string trajectoryDir;
    const std::string poseDir;
    const std::string basePoseFile;
};

#endif //POSEEXTRACTOR_HPP
