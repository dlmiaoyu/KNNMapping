#include "KNNMappingGaussian.hpp"

KNNMappingGaussian::KNNMappingGaussian(const double &resMp, const double &cTMax, const double &oT, const double &pM2,
                                       const double &pM, const double &cTMin, const double &pU, const double &pW,
                                       const int &k, const double &resS, const double &fL, const double &mu,
                                       const double &sigma, std::string pointCloudDir, std::string poseFileName)
        : map(resMp), mresMp(resMp), mcTMax(cTMax), moT(oT), mpM2(pM2), mpM(pM), mcTMin(cTMin), mpU(pU), mpW(pW), mk(k),
          mresS(resS), mfL(fL), mmu(mu), msigma(sigma), mpointCloudDir(std::move(pointCloudDir)),
          mposeFileName(std::move(poseFileName)) {
    map.setClampingThresMax(mcTMax);
    map.setOccupancyThres(moT);
    map.setProbMiss(mpM);
    map.setClampingThresMin(mcTMin);
    logFreeProb = static_cast<float>(log(mpM2 / (1 - mpM2)));
}

void KNNMappingGaussian::GenerateInnerPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &keyFramePointCloud,
                                                 const double (&pose)[7], const double &squaredLimit,
                                                 pcl::PointCloud<pcl::PointXYZ>::Ptr &innerPointCloud) {
    std::vector<int> innerIndices;
    for (int i = 0; i < static_cast<int>(keyFramePointCloud->size()); ++i) {
        if (pow(keyFramePointCloud->points[i].x - pose[0], 2) +
            pow(keyFramePointCloud->points[i].y - pose[1], 2) +
            pow(keyFramePointCloud->points[i].z - pose[2], 2) <= squaredLimit) {
            innerIndices.push_back(i);
        }
    }
    pcl::copyPointCloud(*keyFramePointCloud, innerIndices, *innerPointCloud);
}

void KNNMappingGaussian::GetKNNAndDistance(pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> &pointCloudOctree,
                                           const pcl::PointXYZ &searchPoint, const int &k, double &d) {
    std::vector<int> pointKNNIndices;
    std::vector<float> pointKNNSquaredDistance;
    int K = k + 1;
    double sumDist = 0.0;

    if (pointCloudOctree.nearestKSearch(searchPoint, K, pointKNNIndices, pointKNNSquaredDistance) > 0) {
        for (size_t i = 1; i < pointKNNIndices.size(); ++i) {
            sumDist = sumDist + sqrt(pointKNNSquaredDistance[i]);
        }
        d = sumDist / k;
    }
}

void KNNMappingGaussian::UpdateMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr &keyFramePointCloud,
                                   const pcl::PointCloud<pcl::PointXYZ>::Ptr &innerPointCloud,
                                   const double (&pose)[7]) {
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> keyFramePointCloudOctree(mresS);
    keyFramePointCloudOctree.setInputCloud(keyFramePointCloud);
    keyFramePointCloudOctree.addPointsFromInputCloud();

    //update tmpMap occupied nodes
    octomap::ColorOcTree tmpMap(mresMp);
    for (auto point:*innerPointCloud) {
        double d;
        float logProb;
        GetKNNAndDistance(keyFramePointCloudOctree, point, mk, d);
        CalculateLogProbability(d, logProb);
        octomap::point3d pt(point.x, point.y, point.z);
        tmpMap.updateNode(pt, logProb, false);
    }

    //update map occupied nodes
    for (octomap::ColorOcTree::leaf_iterator it = tmpMap.begin_leafs(); it != tmpMap.end_leafs(); ++it) {
        octomap::point3d nodeCenter = it.getCoordinate();
        float logProb = it->getLogOdds();
        map.updateNode(nodeCenter, logProb, false);
    }

    octomap::Pointcloud keyFrameCloud, innerCloud;
    for (auto point:*keyFramePointCloud) {
        octomap::point3d pt(point.x, point.y, point.z);
        keyFrameCloud.push_back(pt);
    }
    for (auto point:*innerPointCloud) {
        octomap::point3d pt(point.x, point.y, point.z);
        innerCloud.push_back(pt);
    }

    octomap::point3d origin(static_cast<float>(pose[0]), static_cast<float>(pose[1]), static_cast<float>(pose[2]));
    octomap::KeySet innerFreeCells, innerOccupiedCells, keyFrameFreeCells, keyFrameOccupiedCells, outerFreeCells;
    map.computeUpdate(innerCloud, origin, innerFreeCells, innerOccupiedCells, -1);
    map.computeUpdate(keyFrameCloud, origin, keyFrameFreeCells, keyFrameOccupiedCells, mfL);
    for (const auto &cell: keyFrameFreeCells) {
        auto it = innerFreeCells.find(cell);
        if (it == innerFreeCells.end()) {
            outerFreeCells.insert(cell);
        }
    }

    //update map free nodes
    for (const auto &cell:innerFreeCells) {
        map.updateNode(cell, false, false);
    }

    for (const auto &cell:outerFreeCells) {
        map.updateNode(cell, logFreeProb, false);
    }
}

void KNNMappingGaussian::CalculateLogProbability(const double &d, float &logProb) const {
    double prob = mpU - 0.5 * (erfc(-(d - mmu) / msigma * M_SQRT1_2) - erfc(mmu / msigma * M_SQRT1_2)) /
                        (1 - 0.5 * erfc(mmu / msigma * M_SQRT1_2)) * (mpU - mpW);
    logProb = static_cast<float>(log(prob / (1 - prob)));
}

void KNNMappingGaussian::Run() {
    std::ifstream poseFile(mposeFileName);
    if (!poseFile) {
        std::cout << "Failed to open the pose file!" << std::endl;
        return;
    }

    double squaredLimit = pow(mfL, 2);

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

        auto innerPointCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        GenerateInnerPointCloud(keyFramePointCloud, pose, squaredLimit, innerPointCloud);
        UpdateMap(keyFramePointCloud, innerPointCloud, pose);

        map.updateInnerOccupancy();
    }
    map.expand();
}