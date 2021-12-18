#ifndef TIMESTAMPGENERATOR_HPP
#define TIMESTAMPGENERATOR_HPP

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <fstream>
#include <sstream>

#include "IOManager.hpp"

class TimeStampGenerator {
public:
    explicit TimeStampGenerator(const cv::FileStorage &fSettings);

    void Run();

private:
    const std::string sourceImageDir;
    const std::string imageDir;
    const std::string poseFileName;
    const std::string timeStampFileName;

    std::ifstream poseFile;
    std::ofstream timeStampFile;
};

#endif //TIMESTAMPGENERATOR_HPP
