#include "PointCloudProcessorGaussianCB.hpp"
#include "IOManager.hpp"
#include <boost/filesystem.hpp>
#include <cstdlib>

void ProcessPointCloudGaussianCB(const cv::FileStorage &fSettings, const std::string &totalRunTimeDir) {
    bool isDownSampledPointCloud(static_cast<int>(fSettings["IO"]["isDownSampledPointCloud"]) != 0);
    bool isOctoMap(static_cast<int>(fSettings["IO"]["isOctoMap"]) != 0);
    std::string fileName;
    if (isDownSampledPointCloud) {
        if (isOctoMap) {
            fileName = "CBDOctoMapTotalRunTime.txt";
        } else {
            fileName = "CBDKNNTotalRunTime.txt";
        }
    } else {
        if (isOctoMap) {
            fileName = "CBSOctoMapTotalRunTime.txt";
        } else {
            fileName = "CBSKNNTotalRunTime.txt";
        }
    }

    std::ofstream file;
    file.open(totalRunTimeDir + "/" + fileName);
    struct timespec startTime{}, endTime{};
    clock_gettime(CLOCK_MONOTONIC, &startTime);

    PointCloudProcessorGaussianCB objPCPGCB(fSettings);
    objPCPGCB.Run();

    clock_gettime(CLOCK_MONOTONIC, &endTime);
    long double runTime = (endTime.tv_sec - startTime.tv_sec);
    runTime += (endTime.tv_nsec - startTime.tv_nsec) / static_cast<long double>(1000000000.0);
    IOManager::WriteResultFile(file, runTime);
    file.close();
}

int main(int argc, char **argv) {
    std::string settingsFile = argv[1];
    cv::FileStorage fSettings(settingsFile, cv::FileStorage::READ);
    if (!fSettings.isOpened()) {
        std::cout << "Failed to open the settings file!" << std::endl;
        exit(-1);
    }

    std::string runTimeDir = fSettings["IO"]["runTimeDir"];

    boost::filesystem::create_directories(runTimeDir);

    ProcessPointCloudGaussianCB(fSettings, runTimeDir);

    return 0;
}
