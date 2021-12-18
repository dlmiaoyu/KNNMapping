#include "OctoMapMapping.hpp"

#include <utility>

OctoMapMapping::OctoMapMapping(const double &resMp, const double &cTMax, const double &pH, const double &oT,
                               const double &pM, const double &cTMin, const double &fL, std::string pointCloudDir,
                               std::string poseFileName)
        : map(resMp), mresMp(resMp), mcTMax(cTMax), mpH(pH), moT(oT), mpM(pM), mcTMin(cTMin), mfL(fL),
          mpointCloudDir(std::move(pointCloudDir)), mposeFileName(std::move(poseFileName)) {
    map.setClampingThresMax(mcTMax);
    map.setProbHit(mpH);
    map.setOccupancyThres(moT);
    map.setProbMiss(mpM);
    map.setClampingThresMin(mcTMin);
}

void OctoMapMapping::Run() {
    std::ifstream poseFile(mposeFileName);
    if (!poseFile) {
        std::cout << "Failed to open the pose file!" << std::endl;
        return;
    }

    while (true) {
        std::string timeStamp;
        poseFile >> timeStamp;
        if (poseFile.eof()) {
            break;
        }
        double pose[7];
        for (double &d:pose) {
            poseFile >> d;
        }

        auto keyFramePointCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        std::ostringstream keyFramePointCloudFile;
        keyFramePointCloudFile << mpointCloudDir << "/key_" << timeStamp << ".pcd";
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(keyFramePointCloudFile.str(), *keyFramePointCloud) == -1) {
            std::cout << "Could not read " << keyFramePointCloudFile.str() << "!" << std::endl;
        }

        octomap::point3d origin(static_cast<float>(pose[0]), static_cast<float>(pose[1]), static_cast<float>(pose[2]));
        octomap::Pointcloud keyFrameCloud;
        for (auto point:*keyFramePointCloud) {
            octomap::point3d pt(point.x, point.y, point.z);
            keyFrameCloud.push_back(pt);
        }

        map.insertPointCloud(keyFrameCloud, origin, mfL, false, false);
        map.updateInnerOccupancy();
    }
    map.expand();
}