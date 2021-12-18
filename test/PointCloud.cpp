#include "DownSampledPointCloudGeneratorVoxel.hpp"
#include "SparsePointCloudGenerator.hpp"
#include <boost/filesystem.hpp>

void GenerateDownSampledPointCloudVoxel(const cv::FileStorage &fSettings, const std::string &totalRunTimeDir) {
    std::ofstream file;
    file.open(totalRunTimeDir + "/DownSampledPointCloudGeneratorTotalRunTime.txt");
    struct timespec startTime{}, endTime{};
    clock_gettime(CLOCK_MONOTONIC, &startTime);

    DownSampledPointCloudGeneratorVoxel objDSPCGV(fSettings);
    objDSPCGV.Run();

    clock_gettime(CLOCK_MONOTONIC, &endTime);
    long double runTime = (endTime.tv_sec - startTime.tv_sec);
    runTime += (endTime.tv_nsec - startTime.tv_nsec) / static_cast<long double>(1000000000.0);
    IOManager::WriteResultFile(file, runTime);
    file.close();
}

void GenerateSparsePointCloud(const cv::FileStorage &fSettings, const std::string &totalRunTimeDir) {
    std::ofstream file;
    file.open(totalRunTimeDir + "/SparsePointCloudGeneratorTotalRunTime.txt");
    struct timespec startTime{}, endTime{};
    clock_gettime(CLOCK_MONOTONIC, &startTime);

    SparsePointCloudGenerator objSPCG(fSettings);
    objSPCG.Run();

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


    GenerateDownSampledPointCloudVoxel(fSettings, runTimeDir);

//    GenerateSparsePointCloud(fSettings, runTimeDir);

    return 0;
}
