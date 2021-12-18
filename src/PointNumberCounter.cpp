#include "PointNumberCounter.hpp"

PointNumberCounter::PointNumberCounter(const cv::FileStorage &fSettings)
        : poseFileName(fSettings["IO"]["poseFileName"]),
          parameterListDir(fSettings["IO"]["parameterListDir"]),
          downSampledPointCloudDir(fSettings["IO"]["downSampledPointCloudDir"]),
          sparsePointCloudDir(fSettings["IO"]["sparsePointCloudDir"]),
          pointNumberDir(fSettings["IO"]["pointNumberDir"]),
          isDownSampledPointCloud(static_cast<int>(fSettings["IO"]["isDownSampledPointCloud"])!=0),
          fL(fSettings["CloudEvaluation"]["fL"]) {}

void PointNumberCounter::Run() {
    std::string PListName;
    if (isDownSampledPointCloud) {
        PListName = parameterListDir + "/DownSampledPointCloudParameterList.txt";
    } else {
        PListName = parameterListDir + "/SparsePointCloudParameterList.txt";
    }

    std::ifstream PList;
    PList.open(PListName);
    if (!PList) {
        if (isDownSampledPointCloud) {
            std::cout << "Failed to open the down sampled point cloud parameter list!" << std::endl;
        } else {
            std::cout << "Failed to open the sparse point cloud parameter list!" << std::endl;
        }
        return;
    }

    boost::filesystem::create_directories(pointNumberDir);
    std::string resultFileName;
    if (isDownSampledPointCloud) {
        resultFileName = pointNumberDir + "/DownSampledPointNumber.txt";
    } else {
        resultFileName = pointNumberDir + "/SparsePointNumber.txt";
    }
    resultFile.open(resultFileName);
    size_t PNum = IOManager::GetLineNumber(PList);
    for (size_t i = 1; i <= PNum; ++i) {
        std::string PLine;
        getline(PList, PLine);
        std::string PNumber(PLine, 0, 6);
        GetPointNumber(PNumber);
    }
    resultFile.close();
}

void PointNumberCounter::GetPointNumber(const std::string &PNumber) {
    size_t innerNumber = 0;
    size_t totalNumber = 0;
    std::string pointCloudDir, pointCloudID;
    if (isDownSampledPointCloud) {
        pointCloudID.append("D").append(PNumber);
        pointCloudDir.append(downSampledPointCloudDir).append("/").append(pointCloudID);
    } else {
        pointCloudID.append("S").append(PNumber);
        pointCloudDir.append(sparsePointCloudDir).append("/").append(pointCloudID);
    }

    std::ifstream poseFile(poseFileName);
    if (!poseFile) {
        std::cout << "Failed to open the pose file!" << std::endl;
        return;
    }

    double squaredLimit = pow(fL, 2);

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
        keyFramePointCloudFile << pointCloudDir << "/key_" << timeStamp << ".pcd";
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(keyFramePointCloudFile.str(), *keyFramePointCloud) == -1) {
            std::cout << "Could not read " << keyFramePointCloudFile.str() << "!" << std::endl;
        }

        for (auto point:*keyFramePointCloud) {
            if (pow(point.x - pose[0], 2) + pow(point.y - pose[1], 2) + pow(point.z - pose[2], 2) <= squaredLimit) {
                ++innerNumber;
            }
        }
        totalNumber += keyFramePointCloud->size();
    }
    poseFile.close();
    IOManager::WritePointNumber(resultFile, pointCloudID, innerNumber, totalNumber);
}