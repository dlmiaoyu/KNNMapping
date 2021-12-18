#include "SparsePointCloudGenerator.hpp"

SparsePointCloudGenerator::SparsePointCloudGenerator(const cv::FileStorage &fSettings)
        : sourceSparsePointCloudDir(fSettings["IO"]["sourceSparsePointCloudDir"]),
          sparsePointCloudDir(fSettings["IO"]["sparsePointCloudDir"]), poseFileName(fSettings["IO"]["poseFileName"]),
          parameterListDir(fSettings["IO"]["parameterListDir"]), runTimeDir(fSettings["IO"]["runTimeDir"]) {}

void SparsePointCloudGenerator::Run() {
    std::ifstream SList;
    std::string SListName = parameterListDir + "/SparsePointCloudParameterList.txt";
    SList.open(SListName);
    if (!SList) {
        std::cout << "Failed to open the sparse point cloud parameter list!" << std::endl;
        return;
    }
    boost::filesystem::create_directories(runTimeDir);
    std::string singleRunTimeFileName = runTimeDir + "/SparsePointCloudGeneratorSingleRunTime.txt";
    singleRunTimeFile.open(singleRunTimeFileName);

    size_t SNum = IOManager::GetLineNumber(SList);
    totalNum = SNum;

    GenerateSparsePointCloud(SList, SNum);

    SList.close();
    singleRunTimeFile.close();
}

void SparsePointCloudGenerator::GenerateSparsePointCloud(std::ifstream &SList, const size_t &SNum) {
    const unsigned long maxThreads = std::thread::hardware_concurrency();
    ThreadPool pool(maxThreads);
    std::cout << "Generating sparse point clouds: " << std::endl;
    IOManager::DisplayProgressBar(processedNum, totalNum);
    for (size_t i = 1; i <= SNum; ++i) {
        std::string SLine;
        SLock.lock();
        getline(SList, SLine);
        SLock.unlock();
        std::string SNumber(SLine, 0, 6);
        std::string SParameter(SLine, 6, SLine.size() - 6);
        pool.enqueue(&SparsePointCloudGenerator::ThreadGenerateSparsePointCloud, this, SNumber, SParameter);
    }
}

void
SparsePointCloudGenerator::ThreadGenerateSparsePointCloud(const std::string &SNumber, const std::string &SParameter) {
    struct timespec singleRunStartTime{}, singleRunEndTime{};
    clock_gettime(CLOCK_MONOTONIC, &singleRunStartTime);

    std::ifstream poseFile;
    poseFile.open(poseFileName);

    std::stringstream sParameter(SParameter);
    double zL;
    sParameter >> zL;
    std::string sparsePointCloudFolderDir;
    sparsePointCloudFolderDir.append(sparsePointCloudDir).append("/S").append(SNumber);
    boost::filesystem::create_directories(sparsePointCloudFolderDir);
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
        keyFramePointCloudFile << sourceSparsePointCloudDir << "/key_" << timeStamp << ".pcd";
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(keyFramePointCloudFile.str(), *keyFramePointCloud) == -1) {
            std::cout << "Could not read " << keyFramePointCloudFile.str() << "!" << std::endl;
        }

        auto newKeyFramePointCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        for (auto &point : *keyFramePointCloud) {
            if (point.z <= zL) {
                newKeyFramePointCloud->push_back(point);
            }
        }

        newKeyFramePointCloud->width = newKeyFramePointCloud->size();
        newKeyFramePointCloud->height = 1;

        IOManager::WriteKeyFramePointCloudFile(*newKeyFramePointCloud, sparsePointCloudFolderDir, timeStamp);
    }
    poseFile.close();
    clock_gettime(CLOCK_MONOTONIC, &singleRunEndTime);
    long double singleRunTime = (singleRunEndTime.tv_sec - singleRunStartTime.tv_sec);
    singleRunTime += (singleRunEndTime.tv_nsec - singleRunStartTime.tv_nsec) / static_cast<long double>(1000000000.0);

    threadLock.lock();
    ++processedNum;
    IOManager::WriteResultFile(singleRunTimeFile, SNumber + SParameter, singleRunTime);
    IOManager::DisplayProgressBar(processedNum, totalNum);
    threadLock.unlock();
}