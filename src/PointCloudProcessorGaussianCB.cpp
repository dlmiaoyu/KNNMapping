#include "PointCloudProcessorGaussianCB.hpp"

PointCloudProcessorGaussianCB::PointCloudProcessorGaussianCB(const cv::FileStorage &fSettings)
        : poseFileName(fSettings["IO"]["poseFileName"]),
          downSampledPointCloudDir(fSettings["IO"]["downSampledPointCloudDir"]),
          sparsePointCloudDir(fSettings["IO"]["sparsePointCloudDir"]),
          distributionDir(fSettings["IO"]["distributionDir"]),
          referenceMapDir(fSettings["IO"]["referenceMapDir"]),
          layoutFlag(fSettings["ReferenceMap"]["layoutFlag"]),
          mapDir(fSettings["IO"]["mapDir"]),
          mapComparisonDir(fSettings["IO"]["mapComparisonDir"]),
          parameterListDir(fSettings["IO"]["parameterListDir"]),
          testParameterListDir(fSettings["IO"]["testParameterListDir"]),
          isDownSampledPointCloud(static_cast<int>(fSettings["IO"]["isDownSampledPointCloud"]) != 0),
          isOctoMap(static_cast<int>(fSettings["IO"]["isOctoMap"]) != 0),
          writeMapFile(static_cast<int>(fSettings["IO"]["writeMapFile"]) != 0),
          resReferenceMap(fSettings["ReferenceMap"]["resMap"]),
          referenceMap(resReferenceMap) {}

void PointCloudProcessorGaussianCB::Run() {
    std::string CBListName;
    if (isDownSampledPointCloud) {
        if (isOctoMap) {
            CBListName = testParameterListDir + "/CBDOctoMapParameterList.txt";
        } else {
            CBListName = testParameterListDir + "/CBDKNNParameterList.txt";
        }
        distributionFileName = distributionDir + "/DownSampledPointCloudDistribution.txt";
    } else {
        if (isOctoMap) {
            CBListName = testParameterListDir + "/CBSOctoMapParameterList.txt";
        } else {
            CBListName = testParameterListDir + "/CBSKNNParameterList.txt";
        }
        distributionFileName = distributionDir + "/SparsePointCloudDistribution.txt";
    }

    std::ifstream CBList;
    CBList.open(CBListName);
    if (!CBList) {
        std::cout << "Failed to open " << CBListName << " !" << std::endl;
        return;
    }

    totalNum = IOManager::GetLineNumber(CBList);

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

    boost::filesystem::create_directories(mapComparisonDir);
    std::string resultFileName;
    if (isDownSampledPointCloud) {
        if (isOctoMap) {
            resultFileName = mapComparisonDir + "/DownSampledPointCloudOctoMapComparisonResult.txt";
        } else {
            resultFileName = mapComparisonDir + "/DownSampledPointCloudKNNComparisonResult.txt";
        }
    } else {
        if (isOctoMap) {
            resultFileName = mapComparisonDir + "/SparsePointCloudOctoMapComparisonResult.txt";
        } else {
            resultFileName = mapComparisonDir + "/SparsePointCloudKNNComparisonResult.txt";
        }
    }
    resultFile.open(resultFileName);

    if (writeMapFile) {
        boost::filesystem::create_directories(mapDir);
    }

    CompareMap(CBList, totalNum);

    CBList.close();
    resultFile.close();
}

void PointCloudProcessorGaussianCB::CompareMap(std::ifstream &CBList, size_t &CBNum) {
    const unsigned long maxThreads = std::thread::hardware_concurrency();
    ThreadPool pool(maxThreads);
    std::cout << "Generating and comparing maps: " << std::endl;
    IOManager::DisplayProgressBar(processedNum, totalNum);
    for (size_t i = 1; i <= CBNum; ++i) {
        std::string CBLine;
        CBLock.lock();
        getline(CBList, CBLine);
        CBLock.unlock();
        std::string PNumber(CBLine, 1, 6);
        std::string MNumber(CBLine, 8, 6);
        int nS;
        if (isDownSampledPointCloud) {
            nS = FindNthOccur(CBLine, ' ', 13);
        } else {
            nS = FindNthOccur(CBLine, ' ', 2);
        }
        std::string PParameter(CBLine, 14, nS - 14);
        std::string MParameter(CBLine, nS);
        pool.enqueue(&PointCloudProcessorGaussianCB::ThreadCompareMap, this, PNumber, PParameter, MNumber,
                     MParameter);
    }
}

void PointCloudProcessorGaussianCB::ThreadCompareMap(const std::string &PNumber, const std::string &PParameter,
                                                     const std::string &MNumber, const std::string &MParameter) {
    struct timespec singleRunStartTime{}, singleRunEndTime{};
    clock_gettime(CLOCK_MONOTONIC, &singleRunStartTime);
    std::string mappingInfo;
    std::string pointCloudDir;
    if (isDownSampledPointCloud) {
        pointCloudDir.append(downSampledPointCloudDir).append("/D").append(PNumber);
    } else {
        pointCloudDir.append(sparsePointCloudDir).append("/S").append(PNumber);
    }

    std::stringstream sMParameter(MParameter);
    if (isOctoMap) {
        double resMp;
        double cTMax;
        double pH;
        double oT;
        double pM;
        double cTMin;
        double fL;
        sMParameter >> resMp;
        sMParameter >> cTMax;
        sMParameter >> pH;
        sMParameter >> oT;
        sMParameter >> pM;
        sMParameter >> cTMin;
        sMParameter >> fL;

        std::string mapInfo;
        if (isDownSampledPointCloud) {
            mapInfo.append("D").append(PNumber).append("O").append(MNumber).append(PParameter).append(MParameter);
        } else {
            mapInfo.append("S").append(PNumber).append("O").append(MNumber).append(PParameter).append(MParameter);
        }

        OctoMapMapping objOM(resMp, cTMax, pH, oT, pM, cTMin, fL, pointCloudDir, poseFileName);
        objOM.Run();

        MapComparer objMC(objOM.map, referenceMap, writeMapFile);
        objMC.Run();

        if (writeMapFile) {
            std::string mapName(mapInfo, 0, 14);
            IOManager::WriteMapFile(mapDir, mapName, objOM);
            IOManager::WriteMapFile(mapComparisonDir, mapName, objMC);
        }

        mappingInfo = IOManager::GetMappingInfo(mapInfo, objMC);
    } else {
        double resMp;
        double cTMax;
        double oT;
        double pM;
        double pM2;
        double cTMin;
        double pU;
        double pW;
        int k;
        double resS;
        double fL;
        sMParameter >> resMp;
        sMParameter >> cTMax;
        sMParameter >> oT;
        sMParameter >> pM2;
        sMParameter >> pM;
        sMParameter >> cTMin;
        sMParameter >> pU;
        sMParameter >> pW;
        sMParameter >> k;
        sMParameter >> resS;
        sMParameter >> fL;

        std::string mapInfo;
        if (isDownSampledPointCloud) {
            mapInfo.append("D").append(PNumber).append("K").append(MNumber).append(PParameter).append(MParameter);
        } else {
            mapInfo.append("S").append(PNumber).append("K").append(MNumber).append(PParameter).append(MParameter);
        }

        double mu = 0;
        double sigma = 0;

        GetDistribution(PNumber, MParameter, mu, sigma);

        KNNMappingGaussian objKMG(resMp, cTMax, oT, pM2, pM, cTMin, pU, pW, k, resS, fL, mu, sigma, pointCloudDir,
                                  poseFileName);
        objKMG.Run();

        MapComparer objMC(objKMG.map, referenceMap, writeMapFile);
        objMC.Run();

        if (writeMapFile) {
            std::string mapName(mapInfo, 0, 14);
            IOManager::WriteMapFile(mapDir, mapName, objKMG);
            IOManager::WriteMapFile(mapComparisonDir, mapName, objMC);
        }

        mappingInfo = IOManager::GetMappingInfo(mapInfo, objMC);
    }

    clock_gettime(CLOCK_MONOTONIC, &singleRunEndTime);
    long double singleRunTime = (singleRunEndTime.tv_sec - singleRunStartTime.tv_sec);
    singleRunTime +=
            (singleRunEndTime.tv_nsec - singleRunStartTime.tv_nsec) / static_cast<long double>(1000000000.0);

    threadLock.lock();
    ++processedNum;
    IOManager::WriteResultFile(resultFile, mappingInfo, singleRunTime);
    IOManager::DisplayProgressBar(processedNum, totalNum);
    threadLock.unlock();
}

void
PointCloudProcessorGaussianCB::GetDistribution(const std::string &PNumber, const std::string &MParameter, double &mu,
                                               double &sigma) {
    std::ifstream distributionFile;
    distributionFile.open(distributionFileName);
    if (!distributionFile) {
        if (isDownSampledPointCloud) {
            std::cout << "Failed to open the down sampled point cloud distribution file!" << std::endl;
        } else {
            std::cout << "Failed to open the sparse point cloud distribution file!" << std::endl;
        }
        return;
    }

    size_t distributionNum = IOManager::GetLineNumber(distributionFile);

    std::stringstream sMParameter(MParameter);
    std::string resMp;
    std::string cTMax;
    std::string oT;
    std::string pM;
    std::string pM2;
    std::string cTMin;
    std::string pU;
    std::string pW;
    std::string k;
    std::string resS;
    std::string fL;
    sMParameter >> resMp;
    sMParameter >> cTMax;
    sMParameter >> oT;
    sMParameter >> pM2;
    sMParameter >> pM;
    sMParameter >> cTMin;
    sMParameter >> pU;
    sMParameter >> pW;
    sMParameter >> k;
    sMParameter >> resS;
    sMParameter >> fL;

    for (size_t i = 0; i < distributionNum; ++i) {
        std::string distributionLine;
        getline(distributionFile, distributionLine);
        std::string distributionNumber(distributionLine, 0, 6);
        std::string distributionParameter(distributionLine, 6, distributionLine.size() - 6);
        std::stringstream sdistributionParameter(distributionParameter);

        std::string zL;
        std::string k0;
        std::string resS0;
        std::string fL0;

        if (isDownSampledPointCloud) {
            std::string mD;
            std::string nD;
            std::string bS;
            std::string DP1;
            std::string DP2;
            std::string dMD;
            std::string pFC;
            std::string uR;
            std::string sWS;
            std::string sR;
            std::string mode;

            sdistributionParameter >> mD;
            sdistributionParameter >> nD;
            sdistributionParameter >> bS;
            sdistributionParameter >> DP1;
            sdistributionParameter >> DP2;
            sdistributionParameter >> dMD;
            sdistributionParameter >> pFC;
            sdistributionParameter >> uR;
            sdistributionParameter >> sWS;
            sdistributionParameter >> sR;
            sdistributionParameter >> mode;
            sdistributionParameter >> zL;
            sdistributionParameter >> k0;
            sdistributionParameter >> resS0;
            sdistributionParameter >> fL0;
        } else {
            sdistributionParameter >> zL;
            sdistributionParameter >> k0;
            sdistributionParameter >> resS0;
            sdistributionParameter >> fL0;
        }
        if (PNumber == distributionNumber && k == k0 && resS == resS0 && fL == fL0) {
            sdistributionParameter >> mu;
            sdistributionParameter >> sigma;
            return;
        }
    }
    std::string pointCloudInfo;
    if (isDownSampledPointCloud) {
        pointCloudInfo.append("D").append(PNumber);
    } else {
        pointCloudInfo.append("S").append(PNumber);
    }
    std::cout << pointCloudInfo << ": could not find distribution!" << std::endl;
}

int PointCloudProcessorGaussianCB::FindNthOccur(std::string str, char ch, int N) {
    int occur = 0;

    for (int i = 0; i < str.length(); i++) {
        if (str[i] == ch) {
            occur += 1;
        }
        if (occur == N)
            return i;
    }
    return -1;
}