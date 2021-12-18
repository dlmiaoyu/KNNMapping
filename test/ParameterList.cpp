#include "ParameterListGenerator.hpp"
#include <boost/filesystem.hpp>

void GenerateParameterList(const cv::FileStorage &fSettings) {
    ParameterListGenerator objPLG(fSettings);
    objPLG.Run();
}

int main(int argc, char **argv) {
    std::string settingsFile = argv[1];
    cv::FileStorage fSettings(settingsFile, cv::FileStorage::READ);
    if (!fSettings.isOpened()) {
        std::cout << "Failed to open the settings file!" << std::endl;
        exit(-1);
    }

    GenerateParameterList(fSettings);

    return 0;
}
