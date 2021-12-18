#include "TimeStampGenerator.hpp"
#include <boost/filesystem.hpp>

void GenerateTimeStamp(const cv::FileStorage &fSettings) {
    TimeStampGenerator objTSG(fSettings);
    objTSG.Run();
}

int main(int argc, char **argv) {
    std::string settingsFile = argv[1];
    cv::FileStorage fSettings(settingsFile, cv::FileStorage::READ);
    if (!fSettings.isOpened()) {
        std::cout << "Failed to open the settings file!" << std::endl;
        exit(-1);
    }

    GenerateTimeStamp(fSettings);

    return 0;
}
