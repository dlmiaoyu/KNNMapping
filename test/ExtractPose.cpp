#include "PoseExtractor.hpp"
#include <boost/filesystem.hpp>

void ExtractPose(const cv::FileStorage &fSettings) {
    PoseExtractor objPE(fSettings);
    objPE.Run();
}

int main(int argc, char **argv) {
    std::string settingsFile = argv[1];
    cv::FileStorage fSettings(settingsFile, cv::FileStorage::READ);
    if (!fSettings.isOpened()) {
        std::cout << "Failed to open the settings file!" << std::endl;
        exit(-1);
    }

    ExtractPose(fSettings);

    return 0;
}
