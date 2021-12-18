#include "PoseExtractor.hpp"

PoseExtractor::PoseExtractor(const cv::FileStorage &fSettings)
        : num(fSettings["num"]),
          trajectoryDir(fSettings["trajectoryDir"]),
          poseDir(fSettings["poseDir"]),
          basePoseFile(fSettings["basePoseFile"]) {
}

void PoseExtractor::Run() {
    std::ifstream basePose;
    basePose.open(basePoseFile);

    for (int i = 1; i <= num; ++i) {
        std::stringstream fileNum;
        fileNum << std::setfill('0') << std::setw(6) << i;
        std::ifstream trajectory(trajectoryDir + "/" + fileNum.str() + "/CameraTrajectory.txt");
        boost::filesystem::create_directories(poseDir + "/" + fileNum.str());
        std::ofstream poseFile(poseDir + "/" + fileNum.str() + "/Pose.txt");
        poseFile << std::fixed;
        std::string baseLine;
        while (getline(basePose, baseLine)) {
            std::string timeStamp(baseLine, 0, 17);

            std::string line;
            while (getline(trajectory, line)) {
                std::string stamp(line, 0, 17);
                if (timeStamp == stamp) {
                    std::stringstream ss(line);
                    std::string lineStamp;
                    double pose[7];
                    ss >> lineStamp;
                    for (double &d:pose) {
                        ss >> d;
                    }
                    poseFile << lineStamp << " " << std::setprecision(7) << pose[0] << " " << pose[1] << " " << pose[2]
                             << " " << pose[3] << " " << pose[4] << " " << pose[5] << " " << pose[6] << std::endl;
                    break;
                }
            }
            trajectory.clear();
            trajectory.seekg(0, std::ios::beg);
        }
        trajectory.close();
        poseFile.close();
        basePose.clear();
        basePose.seekg(0, std::ios::beg);
    }
    basePose.close();
}