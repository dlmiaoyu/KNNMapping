#include "AverageDistanceGenerator.hpp"

AverageDistanceGenerator::AverageDistanceGenerator(const cv::FileStorage &fSettings)
        : poseFileName(fSettings["IO"]["poseFileName"]),
          downSampledPointCloudDir(fSettings["IO"]["downSampledPointCloudDir"]),
          sparsePointCloudDir(fSettings["IO"]["sparsePointCloudDir"]),
          parameterListDir(fSettings["IO"]["parameterListDir"]),
          allDistanceDir(fSettings["IO"]["allDistanceDir"]),
          frameDistanceDir(fSettings["IO"]["frameDistanceDir"]),
          isDownSampledPointCloud(static_cast<int>(fSettings["IO"]["isDownSampledPointCloud"]) != 0),
          factor(fSettings["Factor"]),
          fLStep(fSettings["Filter"]["fLStep"]),
          fLMin(fSettings["Filter"]["fLMin"]),
          fLMax(fSettings["Filter"]["fLMax"]),
          kStep(fSettings["PointSearch"]["kStep"]),
          kMin(fSettings["PointSearch"]["kMin"]),
          kMax(fSettings["PointSearch"]["kMax"]),
          resSStep(fSettings["PointSearch"]["resSStep"]),
          resSMin(fSettings["PointSearch"]["resSMin"]),
          resSMax(fSettings["PointSearch"]["resSMax"]) {}

void AverageDistanceGenerator::Run() {
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
    int nresS = IOManager::F(resSMax, resSMin, resSStep);
    int nfL = IOManager::F(fLMax, fLMin, fLStep);
    totalNum = PNum * nresS * nfL;

    GenerateAverageDistance(PList, PNum);

    PList.close();
}

void AverageDistanceGenerator::GenerateAverageDistance(std::ifstream &PList, size_t &PNum) {
    const unsigned long maxThreads = std::thread::hardware_concurrency();
    ThreadPool pool(maxThreads);
    std::cout << "Generating average distance: " << std::endl;
    IOManager::DisplayProgressBar(processedNum, totalNum);
    for (size_t i = 1; i <= PNum; ++i) {
        std::string PLine;
        PLock.lock();
        getline(PList, PLine);
        PLock.unlock();
        std::string PNumber(PLine, 0, 6);
        std::string PParameter(PLine, 6, PLine.size() - 6);

        for (int resS = resSMin; resS <= resSMax; resS += resSStep) {
            double dresS = static_cast<double>(resS) / factor;
            for (int fL = fLMin; fL <= fLMax; fL += fLStep) {
                double dfL = static_cast<double>(fL) / factor;
                pool.enqueue(&AverageDistanceGenerator::ThreadGenerateAverageDistance, this, PNumber, PParameter, dresS,
                             dfL);
            }
        }
    }
}

void AverageDistanceGenerator::ThreadGenerateAverageDistance(const std::string &PNumber, const std::string &PParameter,
                                                             const double &resS, const double &fL) {
    std::string pointCloudDir;
    if (isDownSampledPointCloud) {
        pointCloudDir.append(downSampledPointCloudDir).append("/D").append(PNumber);
    } else {
        pointCloudDir.append(sparsePointCloudDir).append("/S").append(PNumber);
    }

    CalculateDistance(PNumber, resS, fL, pointCloudDir);

    threadLock.lock();
    ++processedNum;
    IOManager::DisplayProgressBar(processedNum, totalNum);
    threadLock.unlock();
}

void AverageDistanceGenerator::CalculateDistance(const std::string &PNumber, const double &resS, const double &fL,
                                                 const std::string &pointCloudDir) {
    double squaredLimit = pow(fL, 2);

    for (int k = kMin; k <= kMax; k += kStep) {
        std::ifstream poseFile(poseFileName);
        if (!poseFile) {
            std::cout << "Failed to open the pose file!" << std::endl;
            return;
        }

        std::string allDistanceDirK;
        if (isDownSampledPointCloud) {
            allDistanceDirK = allDistanceDir + "/D" + PNumber + "/" + std::to_string(k);
        } else {
            allDistanceDirK = allDistanceDir + "/S" + PNumber + "/" + std::to_string(k);
        }
        boost::filesystem::create_directories(allDistanceDirK);

        std::ofstream allDistanceFile;
        std::string allDistanceFileName;
        allDistanceFileName.append(allDistanceDirK).append("/AverageDistance.txt");

        allDistanceFile.open(allDistanceFileName);
        allDistanceFile << std::fixed << std::setprecision(6);

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

            auto keyFramePointCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ >>();
            std::ostringstream keyFramePointCloudFile;
            keyFramePointCloudFile << pointCloudDir << "/key_" << timeStamp << ".pcd";
            if (pcl::io::loadPCDFile<pcl::PointXYZ>(keyFramePointCloudFile.str(), *keyFramePointCloud) == -1) {
                std::cout << "Could not read " << keyFramePointCloudFile.str() << "!" << std::endl;
            }

            auto innerPointCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
            KNNMappingGaussian::GenerateInnerPointCloud(keyFramePointCloud, pose, squaredLimit, innerPointCloud);

            pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> keyFramePointCloudOctree(resS);
            keyFramePointCloudOctree.setInputCloud(keyFramePointCloud);
            keyFramePointCloudOctree.addPointsFromInputCloud();

            std::string frameDistanceDirK;
            if (isDownSampledPointCloud) {
                frameDistanceDirK = frameDistanceDir + "/D" + PNumber + "/" + std::to_string(k);
            } else {
                frameDistanceDirK = frameDistanceDir + "/S" + PNumber + "/" + std::to_string(k);
            }
            boost::filesystem::create_directories(frameDistanceDirK);

            std::ofstream frameDistanceFile;
            std::string frameDistanceFileName;
            frameDistanceFileName.append(frameDistanceDirK).append("/").append("key_").append(timeStamp).append(".txt");
            frameDistanceFile.open(frameDistanceFileName);
            frameDistanceFile << std::fixed << std::setprecision(6);

            for (auto point:*innerPointCloud) {
                double d;
                KNNMappingGaussian::GetKNNAndDistance(keyFramePointCloudOctree, point, k, d);
                frameDistanceFile << point.x << " " << point.y << " " << point.z << " " << d << std::endl;
                allDistanceFile << d << std::endl;
            }
            frameDistanceFile.close();
        }
        allDistanceFile.close();
        poseFile.close();
    }
}