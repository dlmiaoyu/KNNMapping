#include "DistributionGenerator.hpp"

DistributionGenerator::DistributionGenerator(const cv::FileStorage &fSettings)
        : poseFileName(fSettings["IO"]["poseFileName"]),
          downSampledPointCloudDir(fSettings["IO"]["downSampledPointCloudDir"]),
          sparsePointCloudDir(fSettings["IO"]["sparsePointCloudDir"]),
          parameterListDir(fSettings["IO"]["parameterListDir"]),
          distributionDir(fSettings["IO"]["distributionDir"]),
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

void DistributionGenerator::Run() {
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
    int nk = IOManager::F(kMax, kMin, kStep);
    int nresS = IOManager::F(resSMax, resSMin, resSStep);
    int nfL = IOManager::F(fLMax, fLMin, fLStep);
    totalNum = PNum * nk * nresS * nfL;

    boost::filesystem::create_directories(distributionDir);
    std::string resultFileName;
    if (isDownSampledPointCloud) {
        resultFileName = distributionDir + "/DownSampledPointCloudDistribution.txt";
    } else {
        resultFileName = distributionDir + "/SparsePointCloudDistribution.txt";
    }
    resultFile.open(resultFileName);

    GenerateDistribution(PList, PNum);

    PList.close();
    resultFile.close();
}

void DistributionGenerator::GenerateDistribution(std::ifstream &PList, size_t &PNum) {
    const unsigned long maxThreads = std::thread::hardware_concurrency();
    ThreadPool pool(maxThreads);
    std::cout << "Generating distribution: " << std::endl;
    IOManager::DisplayProgressBar(processedNum, totalNum);
    for (size_t i = 1; i <= PNum; ++i) {
        std::string PLine;
        PLock.lock();
        getline(PList, PLine);
        PLock.unlock();
        std::string PNumber(PLine, 0, 6);
        std::string PParameter(PLine, 6, PLine.size() - 6);

        for (int k = kMin; k <= kMax; k += kStep) {
            for (int resS = resSMin; resS <= resSMax; resS += resSStep) {
                double dresS = static_cast<double>(resS) / factor;
                for (int fL = fLMin; fL <= fLMax; fL += fLStep) {
                    double dfL = static_cast<double>(fL) / factor;
                    pool.enqueue(&DistributionGenerator::ThreadGenerateDistribution, this, PNumber, PParameter, k,
                                 dresS, dfL);
                }
            }
        }
    }
}

void DistributionGenerator::ThreadGenerateDistribution(const std::string &PNumber, const std::string &PParameter,
                                                       const int &k, const double &resS, const double &fL) {
    struct timespec singleRunStartTime{}, singleRunEndTime{};
    clock_gettime(CLOCK_MONOTONIC, &singleRunStartTime);

    std::string pointCloudDir, pointCloudInfo;
    if (isDownSampledPointCloud) {
        pointCloudDir.append(downSampledPointCloudDir).append("/D").append(PNumber);
    } else {
        pointCloudDir.append(sparsePointCloudDir).append("/S").append(PNumber);
    }
    pointCloudInfo.append(PNumber).append(PParameter);

    double mu = 0;
    double sigma = 0;

    CalculateDistribution(k, resS, fL, pointCloudDir, mu, sigma);

    clock_gettime(CLOCK_MONOTONIC, &singleRunEndTime);
    long double singleRunTime = (singleRunEndTime.tv_sec - singleRunStartTime.tv_sec);
    singleRunTime +=
            (singleRunEndTime.tv_nsec - singleRunStartTime.tv_nsec) / static_cast<long double>(1000000000.0);

    threadLock.lock();
    ++processedNum;
    IOManager::WriteResultFile(resultFile, pointCloudInfo, k, resS, fL, mu, sigma, singleRunTime);
    IOManager::DisplayProgressBar(processedNum, totalNum);
    threadLock.unlock();
}

void DistributionGenerator::AddPointToDistribution(const double &d, size_t &N, double &sum, double &mu) {
    N = N + 1;
    double prvMu = mu;
    mu = mu + (d - mu) / static_cast<double>(N);
    sum = sum + (d - prvMu) * (d - mu);
}

void DistributionGenerator::UpdateStandardDeviation(const size_t &N, const double &sum, double &sigma) {
    sigma = sqrt(sum / static_cast<double>(N));
}

void DistributionGenerator::CalculateDistribution(const int &k, const double &resS, const double &fL,
                                                  const std::string &pointCloudDir, double &mu, double &sigma) {
    std::ifstream poseFile(poseFileName);
    if (!poseFile) {
        std::cout << "Failed to open the pose file!" << std::endl;
        return;
    }

    double squaredLimit = pow(fL, 2);
    double sum = 0;
    size_t N = 0;

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

        pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> innerPointCloudOctree(resS);
        innerPointCloudOctree.setInputCloud(innerPointCloud);
        innerPointCloudOctree.addPointsFromInputCloud();

        for (auto point:*innerPointCloud) {
            double d;
            KNNMappingGaussian::GetKNNAndDistance(innerPointCloudOctree, point, k, d);
            AddPointToDistribution(d, N, sum, mu);
        }
    }
    UpdateStandardDeviation(N, sum, sigma);
}