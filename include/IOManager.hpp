#ifndef IOMANAGER_HPP
#define IOMANAGER_HPP

#include <iostream>
#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>
#include <sstream>
#include <fstream>
#include <sys/stat.h>
#include <cstring>

#include "KNNMappingGaussian.hpp"
#include "OctoMapMapping.hpp"
#include "MapComparer.hpp"

class IOManager {
public:
    static void DisplayProgressBar(const size_t &processedNum, const size_t &totalNum);

    static octomap::ColorOcTree *ReadMap(const std::string &mapFileName);

    static void WriteMapFile(const std::string &mapOutputDir, const std::string &mapName, const OctoMapMapping &objOM);

    static void
    WriteMapFile(const std::string &mapOutputDir, const std::string &mapName, const KNNMappingGaussian &objKMG);

    static void WriteMapFile(const std::string &mapOutputDir, const std::string &mapName, const MapComparer &objMC);

    static std::string GetMappingInfo(const std::string &mapInfo, const MapComparer &objMC);

    static void WriteResultFile(std::ofstream &resultFle, const std::string &mappingInfo);

    static void WriteResultFile(std::ofstream &resultFle, const long double &runTime);

    static void WriteResultFile(std::ofstream &resultFle, const std::string &info, const long double &runTime);

    static void WriteResultFile(std::ofstream &resultFle, const std::string &info, const int &k, const double &resS,
                                const double &fL, const double &mu, const double &sigma, const long double &runTime);

    static void
    WriteKeyFramePointCloudFile(const pcl::PointCloud<pcl::PointXYZ> &pointCloud, const std::string &pointCloudDir,
                                const std::string &timeStamp);

    static void WriteDownSampledPointCloudParameterList(std::ofstream &DList, const size_t &DCount, const int &mD,
                                                        const int &nD, const int &bS, const int &P1, const int &P2,
                                                        const int &dMD, const int &pFC, const int &uR, const int &sWS,
                                                        const int &sR, const bool &mode, const double &zL);

    static void WriteSparsePointCloudParameterList(std::ofstream &SList, const size_t &SCount, const double &zL);

    static void WriteOctoMapParameterList(std::ofstream &OctoMapParameterList, const size_t &OctoMapParameterCount,
                                          const double &resMp, const double &cTMax, const double &pH, const double &oT,
                                          const double &pM, const double &cTMin, const double &fL);

    static void
    WriteKNNParameterList(std::ofstream &KNNParameterList, const size_t &KNNParameterCount, const double &resMp,
                          const double &cTMax, const double &oT, const double &pM2, const double &pM,
                          const double &cTMin, const double &pU, const double &pW, const int &k, const double &resS,
                          const double &fL);

    static void
    WritePointNumber(std::ofstream &pointNumberFile, const std::string &pointCloudID, const size_t &innerNumber,
                     const size_t &totalNumber);

    static void
    WriteNodeNumber(std::ofstream &nodeNumberFile, const std::string &pointCloudID, const size_t &occupiedNumber,
                    const size_t &freeNumber, const size_t &totalNumber);

    static size_t GetLineNumber(std::ifstream &file);

    static void SetReferenceMap(const std::string &referenceMapName, octomap::ColorOcTree &referenceMap);

    static int F(const int &aMax, const int &aMin, const int &stepA);

    static int G(const int &aMin, const int &n, const int &stepA, const int &bMin, const int &nB, const int &stepB);
};

#endif //IOManager_HPP
