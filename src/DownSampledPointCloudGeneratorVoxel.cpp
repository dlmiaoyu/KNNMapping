#include "DownSampledPointCloudGeneratorVoxel.hpp"

DownSampledPointCloudGeneratorVoxel::DownSampledPointCloudGeneratorVoxel(const cv::FileStorage &fSettings)
        : imageDir(fSettings["IO"]["imageDir"]),
          timeStampFileName(fSettings["IO"]["timeStampFileName"]),
          downSampledPointCloudDir(fSettings["IO"]["downSampledPointCloudDir"]),
          poseFileName(fSettings["IO"]["poseFileName"]),
          parameterListDir(fSettings["IO"]["parameterListDir"]),
          runTimeDir(fSettings["IO"]["runTimeDir"]),
          dist(fSettings["Sampling"]["dist"]),
          resMap(fSettings["Sampling"]["resMap"]) {
    fSettings["Camera"]["width"] >> imageSize.width;
    fSettings["Camera"]["height"] >> imageSize.height;
    fSettings["Camera"]["K1"] >> K1;
    fSettings["Camera"]["D1"] >> D1;
    fSettings["Camera"]["K2"] >> K2;
    fSettings["Camera"]["D2"] >> D2;
    fSettings["Camera"]["R"] >> R;
    fSettings["Camera"]["T"] >> T;
    f = K1.at<double>(0, 0);
}

void DownSampledPointCloudGeneratorVoxel::Run() {
    std::ifstream DList;
    std::string DListName = parameterListDir + "/DownSampledPointCloudParameterList.txt";
    DList.open(DListName);
    if (!DList) {
        std::cout << "Failed to open the down sampled point cloud parameter list!" << std::endl;
        return;
    }

    boost::filesystem::create_directories(runTimeDir);
    std::string singleRunTimeFileName = runTimeDir + "/DownSampledPointCloudGeneratorSingleRunTime.txt";
    singleRunTimeFile.open(singleRunTimeFileName);

    ParameterRectification();
    size_t DNum = IOManager::GetLineNumber(DList);
    totalNum = DNum;

    GenerateDownSampledPointCloud(DList, DNum);

    DList.close();
    singleRunTimeFile.close();
}

void DownSampledPointCloudGeneratorVoxel::GenerateDownSampledPointCloud(std::ifstream &DList, const size_t &DNum) {
    const unsigned long maxThreads = std::thread::hardware_concurrency();
    ThreadPool pool(maxThreads);
    std::cout << "Generating down sampled point clouds: " << std::endl;
    IOManager::DisplayProgressBar(processedNum, totalNum);
    for (size_t i = 1; i <= DNum; ++i) {
        std::string DLine;
        DLock.lock();
        getline(DList, DLine);
        DLock.unlock();
        std::string DNumber(DLine, 0, 6);
        std::string DParameter(DLine, 6, DLine.size() - 6);
        pool.enqueue(&DownSampledPointCloudGeneratorVoxel::ThreadGenerateDownSampledPointCloud, this, DNumber,
                     DParameter);
    }
}

void DownSampledPointCloudGeneratorVoxel::ThreadGenerateDownSampledPointCloud(const std::string &DNumber,
                                                                              const std::string &DParameter) {
    struct timespec singleRunStartTime{}, singleRunEndTime{};
    clock_gettime(CLOCK_MONOTONIC, &singleRunStartTime);

    std::stringstream sDParameter(DParameter);
    int mD;
    int nD;
    int bS;
    int DP1;
    int DP2;
    int dMD;
    int pFC;
    int uR;
    int sWS;
    int sR;
    bool mode;
    double zL;

    sDParameter >> mD;
    sDParameter >> nD;
    sDParameter >> bS;
    sDParameter >> DP1;
    sDParameter >> DP2;
    sDParameter >> dMD;
    sDParameter >> pFC;
    sDParameter >> uR;
    sDParameter >> sWS;
    sDParameter >> sR;
    sDParameter >> mode;
    sDParameter >> zL;

    std::string downSampledPointCloudFolderDir;
    downSampledPointCloudFolderDir.append(downSampledPointCloudDir).append("/D").append(DNumber);
    boost::filesystem::create_directories(downSampledPointCloudFolderDir);
    std::ifstream tFile, pFile;
    tFile.open(timeStampFileName);
    size_t tNum = IOManager::GetLineNumber(tFile);
    pFile.open(poseFileName);

    for (size_t i = 1; i <= tNum; ++i) {
        std::string imageTimeStamp;
        getline(tFile, imageTimeStamp);
        std::string leftImageFileName = imageDir + "/l_" + imageTimeStamp + ".png";
        std::string rightImageFileName = imageDir + "/r_" + imageTimeStamp + ".png";
        cv::Mat leftColorImage = cv::imread(leftImageFileName, cv::IMREAD_COLOR);
        cv::Mat rightColorImage = cv::imread(rightImageFileName, cv::IMREAD_COLOR);
        cv::Mat disparity = GetDisparitySGBM(leftColorImage, rightColorImage, mD, nD, bS, DP1, DP2, dMD, pFC, uR, sWS,
                                             sR, mode);

        auto keyFramePointCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        double dL = GetDisparityLimit(zL);
        GenerateDensePointCloud(disparity, dL, keyFramePointCloud);

        std::string poseTimeStamp;
        pFile >> poseTimeStamp;

        double pose[7];
        for (double &d:pose) {
            pFile >> d;
        }

        Eigen::Affine3d transform = Eigen::Affine3d::Identity();
        transform.translation() << pose[0], pose[1], pose[2];
        Eigen::Quaterniond q;
        q.x() = pose[3];
        q.y() = pose[4];
        q.z() = pose[5];
        q.w() = pose[6];
        Eigen::Matrix3d Rt(q);
        transform.rotate(Rt);
        auto transformedCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        pcl::transformPointCloud(*keyFramePointCloud, *transformedCloud, transform);
        transformedCloud->swap(*keyFramePointCloud);

        pcl::VoxelGrid<pcl::PointXYZ> voxelFilter;
        voxelFilter.setInputCloud(keyFramePointCloud);
        voxelFilter.setLeafSize(0.1f, 0.1f, 0.1f);
        auto newKeyFramePointCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        voxelFilter.filter(*newKeyFramePointCloud);

//        pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> keyFramePointCloudOctree(0.1);
//        pcl::PointXYZ originPoint(0, 0, 0);
//        keyFramePointCloud->insert(keyFramePointCloud->begin(), originPoint);
//        keyFramePointCloudOctree.setInputCloud(keyFramePointCloud);
//        keyFramePointCloudOctree.addPointsFromInputCloud();
//        keyFramePointCloudOctree.deleteVoxelAtPoint(originPoint);
//        pcl::octree::OctreePointCloud<pcl::PointXYZ>::AlignedPointTVector keyFrameVoxelCenters;
//        keyFramePointCloudOctree.getOccupiedVoxelCenters(keyFrameVoxelCenters);
//        auto newKeyFramePointCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
//        for (auto center:keyFrameVoxelCenters) {
//            newKeyFramePointCloud->push_back(center);
//        }
//        std::cout << *newKeyFramePointCloud->begin() << std::endl;

        IOManager::WriteKeyFramePointCloudFile(*newKeyFramePointCloud, downSampledPointCloudFolderDir, poseTimeStamp);
    }
    tFile.close();
    pFile.close();

    clock_gettime(CLOCK_MONOTONIC, &singleRunEndTime);
    long double singleRunTime = (singleRunEndTime.tv_sec - singleRunStartTime.tv_sec);
    singleRunTime += (singleRunEndTime.tv_nsec - singleRunStartTime.tv_nsec) / static_cast<long double>(1000000000.0);

    threadLock.lock();
    ++processedNum;
    IOManager::WriteResultFile(singleRunTimeFile, DNumber + DParameter, singleRunTime);
    IOManager::DisplayProgressBar(processedNum, totalNum);
    threadLock.unlock();
}

void DownSampledPointCloudGeneratorVoxel::ParameterRectification() {
    cv::stereoRectify(K1, D1, K2, D2, imageSize, R, T, R1, R2, P1, P2, Q, 0, 0);
}

cv::Mat
DownSampledPointCloudGeneratorVoxel::GetDisparitySGBM(const cv::Mat &leftColorImage, const cv::Mat &rightColorImage,
                                                      const int &mD, const int &nD, const int &bS, const int &DP1,
                                                      const int &DP2, const int &dMD, const int &pFC, const int &uR,
                                                      const int &sWS, const int &sR, const bool &mode) {
    cv::Mat leftGrayImage, rightGrayImage, disparity;
    cv::cvtColor(leftColorImage, leftGrayImage, cv::COLOR_BGR2GRAY);
    cv::cvtColor(rightColorImage, rightGrayImage, cv::COLOR_BGR2GRAY);
    cv::Ptr<cv::StereoSGBM> leftMatcher = cv::StereoSGBM::create(mD, nD, bS, DP1, DP2, dMD, pFC, uR, sWS, sR, mode);
    leftMatcher->compute(leftGrayImage, rightGrayImage, disparity);
    return disparity;
}


double DownSampledPointCloudGeneratorVoxel::GetDisparityLimit(const double &zL) {
    return -T.at<double>(0, 0) * (Q.at<double>(2, 3) / zL - Q.at<double>(3, 3));
}

void DownSampledPointCloudGeneratorVoxel::GenerateDensePointCloud(const cv::Mat &disparity, const double &dL,
                                                                  pcl::PointCloud<pcl::PointXYZ>::Ptr &densePointCloud) {
    for (int i = 0; i < imageSize.height; ++i) {
        for (int j = 0; j < imageSize.width; ++j) {
            double d = static_cast<double>(disparity.at<short>(i, j)) / 16.0;
            if (d < dL) {
                continue;
            }

            cv::Mat v(4, 1, CV_64F);
            v.at<double>(0, 0) = static_cast<double> (j);
            v.at<double>(1, 0) = static_cast<double> (i);
            v.at<double>(2, 0) = static_cast<double> (d);
            v.at<double>(3, 0) = 1.0;

            cv::Mat pos(4, 1, CV_64F);
            pos = Q * v;

            pcl::PointXYZ point;
            point.x = static_cast<float>(pos.at<double>(0, 0) / pos.at<double>(3, 0));
            point.y = static_cast<float>(pos.at<double>(1, 0) / pos.at<double>(3, 0));
            point.z = static_cast<float>(pos.at<double>(2, 0) / pos.at<double>(3, 0));

            densePointCloud->push_back(point);
        }
    }
    densePointCloud->width = densePointCloud->size();
    densePointCloud->height = 1;
}