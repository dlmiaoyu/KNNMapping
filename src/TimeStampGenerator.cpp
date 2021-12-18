#include "TimeStampGenerator.hpp"

TimeStampGenerator::TimeStampGenerator(const cv::FileStorage &fSettings)
        : sourceImageDir(fSettings["IO"]["sourceImageDir"]),
          imageDir(fSettings["IO"]["imageDir"]),
          poseFileName(fSettings["IO"]["poseFileName"]),
          timeStampFileName(fSettings["IO"]["timeStampFileName"]) {}

void TimeStampGenerator::Run() {
    poseFile.open(poseFileName);
    if (!poseFile) {
        std::cout << "Failed to open the pose file!" << std::endl;
        return;
    }
    size_t pNum = IOManager::GetLineNumber(poseFile);
    timeStampFile.open(timeStampFileName);
    boost::filesystem::create_directories(imageDir);

    std::string fTimesName = sourceImageDir + "/ORBSLAMTimeStamp.txt";
    std::ifstream fTimes;
    fTimes.open(fTimesName);
    std::cout << "Generating time stamp file: " << std::endl;
    IOManager::DisplayProgressBar(0, 1);
    for (size_t i = 1; i <= pNum; ++i) {
        std::string poseTimeStamp;
        poseFile >> poseTimeStamp;

        double pose[7];
        for (double &d:pose)
            poseFile >> d;

        while (!fTimes.eof()) {
            std::string s;
            getline(fTimes, s);
            if (!s.empty()) {
                std::stringstream ss;
                ss << s;
                double t;
                ss >> t;
                std::stringstream ssimageTimeStamp;
                ssimageTimeStamp << std::fixed << std::setprecision(6) << t / 1e9;
                if (ssimageTimeStamp.str() == poseTimeStamp) {
                    timeStampFile << ss.str().substr(0, 10) + "." + ss.str().substr(10, 9) << std::endl;
                    std::string srcLeftImageFileName =
                            sourceImageDir + "/l_" + ss.str().substr(0, 10) + "." + ss.str().substr(10, 9) + ".png";
                    std::string srcRightImageFileName =
                            sourceImageDir + "/r_" + ss.str().substr(0, 10) + "." + ss.str().substr(10, 9) + ".png";
                    std::string destLeftImageFileName =
                            imageDir + "/l_" + ss.str().substr(0, 10) + "." + ss.str().substr(10, 9) + ".png";
                    std::string destRightImageFileName =
                            imageDir + "/r_" + ss.str().substr(0, 10) + "." + ss.str().substr(10, 9) + ".png";
                    boost::filesystem::copy_file(srcLeftImageFileName, destLeftImageFileName,
                                                 boost::filesystem::copy_option::overwrite_if_exists);
                    boost::filesystem::copy_file(srcRightImageFileName, destRightImageFileName,
                                                 boost::filesystem::copy_option::overwrite_if_exists);
                }
            }
        }
        fTimes.clear();
        fTimes.seekg(0, std::ios::beg);
    }
    IOManager::DisplayProgressBar(1, 1);
    poseFile.close();
    fTimes.close();
    timeStampFile.close();
}
