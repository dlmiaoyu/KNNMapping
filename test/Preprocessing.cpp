#include "PointCloudEvaluator.hpp"
#include "DistributionGenerator.hpp"
#include "AverageDistanceGenerator.hpp"
#include <boost/filesystem.hpp>

void EvaluatePointCloud(const cv::FileStorage &fSettings) {
    PointCloudEvaluator objPCE(fSettings);
    objPCE.Run();
}

void GenerateDistribution(const cv::FileStorage &fSettings, const std::string &totalRunTimeDir) {
    bool isDownSampledPointCloud(static_cast<int>(fSettings["IO"]["isDownSampledPointCloud"]) != 0);
    std::string fileName;
    if (isDownSampledPointCloud) {
        fileName = "DownSampledPointCloudDistributionTotalRunTime.txt";
    } else {
        fileName = "SparsePointCloudDistributionTotalRunTime.txt";
    }

    std::ofstream file;
    file.open(totalRunTimeDir + "/" + fileName);
    struct timespec startTime{}, endTime{};
    clock_gettime(CLOCK_MONOTONIC, &startTime);

    DistributionGenerator objDG(fSettings);
    objDG.Run();

    clock_gettime(CLOCK_MONOTONIC, &endTime);
    long double runTime = (endTime.tv_sec - startTime.tv_sec);
    runTime += (endTime.tv_nsec - startTime.tv_nsec) / static_cast<long double>(1000000000.0);
    IOManager::WriteResultFile(file, runTime);
    file.close();
}

void GenerateAverageDistance(const cv::FileStorage &fSettings) {
    bool isDownSampledPointCloud(static_cast<int>(fSettings["IO"]["isDownSampledPointCloud"]) != 0);
    AverageDistanceGenerator objADG(fSettings);
    objADG.Run();
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

    EvaluatePointCloud(fSettings);

    GenerateDistribution(fSettings, runTimeDir);

    GenerateAverageDistance(fSettings);

    return 0;
}
