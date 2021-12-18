#include "IOManager.hpp"

void IOManager::DisplayProgressBar(const size_t &processedNum, const size_t &totalNum) {
    double progress = (double) processedNum / (double) totalNum;
    std::cout << "[";
    for (size_t i = 0; i <= 50; ++i) {
        if (i < size_t(progress * 50)) {
            std::cout << '=';
        } else if (i == size_t(progress * 50)) {
            std::cout << '>';
        } else {
            std::cout << ' ';
        }
    }
    if (processedNum != totalNum) {
        std::cout << "] " << size_t(progress * 100) << " %" << '\r' << std::flush;
    } else {
        std::cout << "] " << size_t(progress * 100) << " %" << std::endl;
    }
}

octomap::ColorOcTree *IOManager::ReadMap(const std::string &mapFileName) {
    octomap::AbstractOcTree *mapAbstractOctree = octomap::AbstractOcTree::read(mapFileName);
    if (mapAbstractOctree) {
        auto map = dynamic_cast<octomap::ColorOcTree *>(mapAbstractOctree);
        return map;
    }
    return nullptr;
}

void IOManager::WriteMapFile(const std::string &mapOutputDir, const std::string &mapName, const OctoMapMapping &objOM) {
    std::ostringstream mapFileNameStream;
    mapFileNameStream << mapOutputDir << "/" << mapName << ".ot";
    objOM.map.write(mapFileNameStream.str());
}

void
IOManager::WriteMapFile(const std::string &mapOutputDir, const std::string &mapName, const KNNMappingGaussian &objKMG) {
    std::ostringstream mapFileNameStream;
    mapFileNameStream << mapOutputDir << "/" << mapName << ".ot";
    objKMG.map.write(mapFileNameStream.str());
}

void IOManager::WriteMapFile(const std::string &mapOutputDir, const std::string &mapName, const MapComparer &objMC) {
    std::ostringstream mapFileNameStream;
    mapFileNameStream << mapOutputDir << "/" << mapName << "vsReferenceMap.ot";
    objMC.mapVsRefMap.write(mapFileNameStream.str());
}

std::string IOManager::GetMappingInfo(const std::string &mapInfo, const MapComparer &objMC) {
    std::stringstream smappingInfo;
    smappingInfo << mapInfo << " " << objMC.TP << " " << objMC.FP << " " << objMC.TN << " " << objMC.FN << " "
                 << objMC.unknown;
    return smappingInfo.str();
}

void IOManager::WriteResultFile(std::ofstream &resultFle, const std::string &mappingInfo) {
    resultFle << mappingInfo << std::endl;
}

void IOManager::WriteResultFile(std::ofstream &resultFle, const long double &runTime) {
    resultFle << std::fixed << std::setprecision(3) << runTime << std::endl;
}

void IOManager::WriteResultFile(std::ofstream &resultFle, const std::string &info, const long double &runTime) {
    resultFle << std::defaultfloat << info << " " << std::fixed << std::setprecision(3) << runTime << std::endl;
}

void IOManager::WriteResultFile(std::ofstream &resultFle, const std::string &info, const int &k, const double &resS,
                                const double &fL, const double &mu, const double &sigma, const long double &runTime) {
    resultFle << std::defaultfloat << info << " " << k << " " << resS << " " << fL << " " << std::fixed
              << std::setprecision(6) << mu << " " << sigma << " " << std::setprecision(3) << runTime << std::endl;
}

void IOManager::WriteKeyFramePointCloudFile(const pcl::PointCloud<pcl::PointXYZ> &pointCloud,
                                            const std::string &pointCloudDir, const std::string &timeStamp) {
    if (!pointCloud.empty()) {
        std::ostringstream pointCloudFileName;
        pointCloudFileName << pointCloudDir << "/key_" << timeStamp << ".pcd";
        pcl::io::savePCDFileASCII(pointCloudFileName.str(), pointCloud);
    } else {
        std::cout << "Cannot write the keyframe point cloud file! Frame at " << timeStamp << " contains no points!"
                  << std::endl;
    }
}

void IOManager::WriteDownSampledPointCloudParameterList(std::ofstream &DList, const size_t &DCount, const int &mD,
                                                        const int &nD, const int &bS, const int &P1, const int &P2,
                                                        const int &dMD, const int &pFC, const int &uR, const int &sWS,
                                                        const int &sR, const bool &mode, const double &zL) {
    DList << std::setfill('0') << std::setw(6) << DCount << std::setfill(' ') << " " << mD << " " << nD << " " << bS
          << " " << P1 << " " << P2 << " " << dMD << " " << pFC << " " << uR << " " << sWS << " " << sR << " " << mode
          << " " << zL << std::endl;
}

void IOManager::WriteSparsePointCloudParameterList(std::ofstream &SList, const size_t &SCount, const double &zL) {
    SList << std::setfill('0') << std::setw(6) << SCount << std::setfill(' ') << " " << zL << std::endl;
}

void IOManager::WriteOctoMapParameterList(std::ofstream &OctoMapParameterList, const size_t &OctoMapParameterCount,
                                          const double &resMp, const double &cTMax, const double &pH, const double &oT,
                                          const double &pM, const double &cTMin, const double &fL) {
    OctoMapParameterList << std::setfill('0') << std::setw(6) << OctoMapParameterCount << std::setfill(' ') << " "
                         << resMp << " " << cTMax << " " << pH << " " << oT << " " << pM << " " << cTMin << " " << fL
                         << std::endl;
}

void
IOManager::WriteKNNParameterList(std::ofstream &KNNParameterList, const size_t &KNNParameterCount, const double &resMp,
                                 const double &cTMax, const double &oT, const double &pM2, const double &pM,
                                 const double &cTMin, const double &pU, const double &pW, const int &k,
                                 const double &resS, const double &fL) {
    KNNParameterList << std::setfill('0') << std::setw(6) << KNNParameterCount << std::setfill(' ') << " " << resMp
                     << " " << cTMax << " " << oT << " " << pM2 << " " << pM << " " << cTMin << " " << pU << " " << pW
                     << " " << k << " " << resS << " " << fL << std::endl;
}

void
IOManager::WritePointNumber(std::ofstream &pointNumberFile, const std::string &pointCloudID, const size_t &innerNumber,
                            const size_t &totalNumber) {
    pointNumberFile << pointCloudID << " " << innerNumber << " " << totalNumber << " " << std::endl;
}

void
IOManager::WriteNodeNumber(std::ofstream &nodeNumberFile, const std::string &pointCloudID, const size_t &occupiedNumber,
                           const size_t &freeNumber, const size_t &totalNumber) {
    nodeNumberFile << pointCloudID << " " << occupiedNumber << " " << freeNumber << " " << totalNumber << " "
                   << std::endl;
}

size_t IOManager::GetLineNumber(std::ifstream &file) {
    size_t lineNum = 0;
    std::string line;
    while (getline(file, line)) {
        ++lineNum;
    }
    file.clear();
    file.seekg(0, std::ios::beg);

    return lineNum;
}

void IOManager::SetReferenceMap(const std::string &referenceMapName, octomap::ColorOcTree &referenceMap) {
    octomap::ColorOcTree *refMapPtr = IOManager::ReadMap(referenceMapName);
    if (refMapPtr) {
        refMapPtr->expand();
        referenceMap.setResolution(refMapPtr->getResolution());

        for (auto it = refMapPtr->begin_leafs(); it != refMapPtr->end_leafs(); ++it) {
            octomap::point3d pt = it.getCoordinate();
            octomap::ColorOcTreeNode *node = refMapPtr->search(pt);
            referenceMap.updateNode(pt, refMapPtr->isNodeOccupied(node), false);
        }
        referenceMap.updateInnerOccupancy();
        referenceMap.expand();
    } else {
        std::cout << "Failed to configure reference map from the input file!" << std::endl;
    }
}

int IOManager::F(const int &aMax, const int &aMin, const int &stepA) {
    return (aMax - aMin) / stepA + 1;
}

int IOManager::G(const int &aMin, const int &n, const int &stepA, const int &bMin, const int &nB, const int &stepB) {
    int tmpA = (aMin + (n - 1) * stepA - bMin) / stepB + 1;
    int tmpB = nB;

    if (tmpA <= tmpB) {
        return tmpA;
    } else {
        return tmpB;
    }
}