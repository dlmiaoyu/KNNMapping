#include "PointNumberCounter.hpp"
#include "IOManager.hpp"
#include <boost/filesystem.hpp>
#include <cstdlib>

void CountPointNumber(const cv::FileStorage &fSettings) {
    PointNumberCounter objPNC(fSettings);
    objPNC.Run();
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

    CountPointNumber(fSettings);

    return 0;
}
