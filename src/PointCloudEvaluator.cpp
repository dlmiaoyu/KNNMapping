#include "PointCloudEvaluator.hpp"

PointCloudEvaluator::PointCloudEvaluator(const cv::FileStorage &fSettings)
        : referenceMapDir(fSettings["IO"]["referenceMapDir"]),
          layoutFlag(fSettings["ReferenceMap"]["layoutFlag"]),
          pointCloudEvaluationDir(fSettings["IO"]["pointCloudEvaluationDir"]),
          parameterListDir(fSettings["IO"]["parameterListDir"]),
          downSampledPointCloudDir(fSettings["IO"]["downSampledPointCloudDir"]),
          sparsePointCloudDir(fSettings["IO"]["sparsePointCloudDir"]),
          poseFileName(fSettings["IO"]["poseFileName"]),
          referenceMap(0.1),
          isDownSampledPointCloud(static_cast<int>(fSettings["IO"]["isDownSampledPointCloud"]) != 0),
          fL(fSettings["CloudEvaluation"]["fL"]) {
    squaredMaxRange = pow(fL, 2);
}

void PointCloudEvaluator::Run() {
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

    size_t PNum = IOManager::GetLineNumber(PList);
    totalNum = PNum;

    std::string boxLayout;
    switch (layoutFlag) {
        case 1:
            boxLayout = "I";
            break;
        case 2:
            boxLayout = "O";
            break;
        case 3:
            boxLayout = "T";
            break;
        case 4:
            boxLayout = "L";
            break;
        case 5:
            boxLayout = "S";
            break;
        default:
            std::cout << "Invalid layout!" << std::endl;
    }

    std::string referenceMapName = referenceMapDir + "/ReferenceMap" + boxLayout + ".ot";
    IOManager::SetReferenceMap(referenceMapName, referenceMap);

    boost::filesystem::create_directories(pointCloudEvaluationDir);
    std::string pointCloudEvaluationFileName;
    if (isDownSampledPointCloud) {
        pointCloudEvaluationFileName = pointCloudEvaluationDir + "/DownSampledPointCloudEvaluation.txt";
    } else {
        pointCloudEvaluationFileName = pointCloudEvaluationDir + "/SparsePointCloudEvaluation.txt";
    }
    resultFile.open(pointCloudEvaluationFileName);
    EvaluatePointCloud(PList, PNum);
    PList.close();
    resultFile.close();
}

void PointCloudEvaluator::EvaluatePointCloud(std::ifstream &PList, size_t &PNum) {
    const unsigned long maxThreads = std::thread::hardware_concurrency();
    ThreadPool pool(maxThreads);
    std::cout << "Evaluating map point clouds:" << std::endl;
    IOManager::DisplayProgressBar(processedNum, totalNum);
    for (size_t i = 1; i <= PNum; ++i) {
        std::string DLine;
        PLock.lock();
        getline(PList, DLine);
        PLock.unlock();
        std::string PNumber(DLine, 0, 6);
        std::string PParameter(DLine, 6, DLine.size() - 6);
        pool.enqueue(&PointCloudEvaluator::ThreadEvaluatePointCloud, this, PNumber, PParameter);
    }
}

void PointCloudEvaluator::ThreadEvaluatePointCloud(const std::string &PNumber, const std::string &PParameter) {
    std::string pointCloudDir;
    if (isDownSampledPointCloud) {
        pointCloudDir.append(downSampledPointCloudDir).append("/D").append(PNumber);
    } else {
        pointCloudDir.append(sparsePointCloudDir).append("/S").append(PNumber);
    }

    std::ifstream poseFile;
    poseFile.open(poseFileName);
    octomap::ColorOcTree map(referenceMap.getResolution());

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
            if (pow(point.x - pose[0], 2) + pow(point.y - pose[1], 2) + pow(point.z - pose[2], 2) <=
                squaredMaxRange) {
                octomap::point3d pt(point.x, point.y, point.z);
                octomap::ColorOcTreeNode *node = map.search(pt);
                if (node == nullptr) {
                    map.updateNode(pt, true, false);
                } else if (!map.isNodeOccupied(node)) {
                    map.deleteNode(pt);
                    map.updateNode(pt, true, false);
                }
            }
        }

        octomap::point3d origin(static_cast<float>(pose[0]), static_cast<float>(pose[1]), static_cast<float>(pose[2]));
        octomap::Pointcloud keyFrameCloud;
        for (auto point:*keyFramePointCloud) {
            octomap::point3d pt(point.x, point.y, point.z);
            keyFrameCloud.push_back(pt);
        }
        octomap::KeySet freeCells, occupiedCells;
        map.computeUpdate(keyFrameCloud, origin, freeCells, occupiedCells, fL);
        for (const auto &cell:freeCells) {
            octomap::ColorOcTreeNode *node = map.search(cell);
            if (node == nullptr) {
                map.updateNode(cell, false, false);
            }
        }
    }
    map.updateInnerOccupancy();
    map.expand();
    poseFile.close();

    MapComparer objMC(map, referenceMap, false);
    objMC.Run();

    std::string mapInfo;
    if (isDownSampledPointCloud) {
        mapInfo.append("D").append(PNumber).append(PParameter);
    } else {
        mapInfo.append("S").append(PNumber).append(PParameter);
    }

    std::string mappingInfo = IOManager::GetMappingInfo(mapInfo, objMC);
    threadLock.lock();
    ++processedNum;
    IOManager::WriteResultFile(resultFile, mappingInfo);
    IOManager::DisplayProgressBar(processedNum, totalNum);
    threadLock.unlock();
}