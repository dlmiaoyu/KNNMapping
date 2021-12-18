#include "ParameterListGenerator.hpp"

ParameterListGenerator::ParameterListGenerator(const cv::FileStorage &fSettings)
        : parameterListDir(fSettings["IO"]["parameterListDir"]),
          imageChannelNum(fSettings["ImageChannelNumber"]),
          mDStep(fSettings["StereoSGBM"]["mDStep"]),
          mDMin(fSettings["StereoSGBM"]["mDMin"]),
          mDMax(fSettings["StereoSGBM"]["mDMax"]),
          nDStep(fSettings["StereoSGBM"]["nDStep"]),
          nDMin(fSettings["StereoSGBM"]["nDMin"]),
          nDMax(fSettings["StereoSGBM"]["nDMax"]),
          bSStep(fSettings["StereoSGBM"]["bSStep"]),
          bSMin(fSettings["StereoSGBM"]["bSMin"]),
          bSMax(fSettings["StereoSGBM"]["bSMax"]),
          dMDStep(fSettings["StereoSGBM"]["dMDStep"]),
          dMDMin(fSettings["StereoSGBM"]["dMDMin"]),
          dMDMax(fSettings["StereoSGBM"]["dMDMax"]),
          pFCStep(fSettings["StereoSGBM"]["pFCStep"]),
          pFCMin(fSettings["StereoSGBM"]["pFCMin"]),
          pFCMax(fSettings["StereoSGBM"]["pFCMax"]),
          uRStep(fSettings["StereoSGBM"]["uRStep"]),
          uRMin(fSettings["StereoSGBM"]["uRMin"]),
          uRMax(fSettings["StereoSGBM"]["uRMax"]),
          sWSStep(fSettings["StereoSGBM"]["sWSStep"]),
          sWSMin(fSettings["StereoSGBM"]["sWSMin"]),
          sWSMax(fSettings["StereoSGBM"]["sWSMax"]),
          sRStep(fSettings["StereoSGBM"]["sRStep"]),
          sRMin(fSettings["StereoSGBM"]["sRMin"]),
          sRMax(fSettings["StereoSGBM"]["sRMax"]),
          mode(static_cast<int>(fSettings["StereoSGBM"]["mode"]) != 0),
          factor(fSettings["Factor"]),
          zLStep(fSettings["Filter"]["zLStep"]),
          zLMin(fSettings["Filter"]["zLMin"]),
          zLMax(fSettings["Filter"]["zLMax"]),
          fLStep(fSettings["Filter"]["fLStep"]),
          fLMin(fSettings["Filter"]["fLMin"]),
          fLMax(fSettings["Filter"]["fLMax"]),
          resMpStep(fSettings["OctoMap"]["resMpStep"]),
          resMpMin(fSettings["OctoMap"]["resMpMin"]),
          resMpMax(fSettings["OctoMap"]["resMpMax"]),
          cTMaxStep(fSettings["OctoMap"]["cTMaxStep"]),
          cTMaxMin(fSettings["OctoMap"]["cTMaxMin"]),
          cTMaxMax(fSettings["OctoMap"]["cTMaxMax"]),
          pHStep(fSettings["OctoMap"]["pHStep"]),
          pHMin(fSettings["OctoMap"]["pHMin"]),
          pHMax(fSettings["OctoMap"]["pHMax"]),
          oTStep(fSettings["OctoMap"]["oTStep"]),
          oTMin(fSettings["OctoMap"]["oTMin"]),
          oTMax(fSettings["OctoMap"]["oTMax"]),
          pM2Step(fSettings["OctoMap"]["pM2Step"]),
          pM2Min(fSettings["OctoMap"]["pM2Min"]),
          pM2Max(fSettings["OctoMap"]["pM2Max"]),
          pMStep(fSettings["OctoMap"]["pMStep"]),
          pMMin(fSettings["OctoMap"]["pMMin"]),
          pMMax(fSettings["OctoMap"]["pMMax"]),
          cTMinStep(fSettings["OctoMap"]["cTMinStep"]),
          cTMinMin(fSettings["OctoMap"]["cTMinMin"]),
          cTMinMax(fSettings["OctoMap"]["cTMinMax"]),
          pUStep(fSettings["KNNModel"]["pUStep"]),
          pUMin(fSettings["KNNModel"]["pUMin"]),
          pUMax(fSettings["KNNModel"]["pUMax"]),
          pWStep(fSettings["KNNModel"]["pWStep"]),
          pWMin(fSettings["KNNModel"]["pWMin"]),
          pWMax(fSettings["KNNModel"]["pWMax"]),
          kStep(fSettings["PointSearch"]["kStep"]),
          kMin(fSettings["PointSearch"]["kMin"]),
          kMax(fSettings["PointSearch"]["kMax"]),
          resSStep(fSettings["PointSearch"]["resSStep"]),
          resSMin(fSettings["PointSearch"]["resSMin"]),
          resSMax(fSettings["PointSearch"]["resSMax"]) {}

void ParameterListGenerator::Run() {
    std::cout << "Generating parameter lists: " << std::endl;
    boost::filesystem::create_directories(parameterListDir);
    IOManager::DisplayProgressBar(0, 1);
    GenerateDownSampledPointCloudParameterList();
    GenerateSparsePointCloudParameterList();
    GenerateOctoMapParameterList();
    GenerateKNNParameterList();
    IOManager::DisplayProgressBar(1, 1);
    GetDownSampledPointCloudParameterCombinationNumber();
    GetSparsePointCloudParameterCombinationNumber();
    GetOctoMapParameterCombinationNumber();
    GetKNNParameterCombinationNumber();
}

void ParameterListGenerator::GenerateDownSampledPointCloudParameterList() {
    std::ofstream DList;
    std::string DListName = parameterListDir + "/DownSampledPointCloudParameterList.txt";
    DList.open(DListName);

    for (int mD = mDMin; mD <= mDMax; mD += mDStep) {
        for (int nD = nDMin; nD <= nDMax; nD += nDStep) {
            for (int bS = bSMin; bS <= bSMax; bS += bSStep) {
                int P1 = 8 * imageChannelNum * bS * bS;
                int P2 = 32 * imageChannelNum * bS * bS;
                for (int dMD = dMDMin; dMD <= dMDMax; dMD += dMDStep) {
                    for (int pFC = pFCMin; pFC <= pFCMax; pFC += pFCStep) {
                        for (int uR = uRMin; uR <= uRMax; uR += uRStep) {
                            for (int sWS = sWSMin; sWS <= sWSMax; sWS += sWSStep) {
                                for (int sR = sRMin; sR <= sRMax; sR += sRStep) {
                                    for (int zL = zLMin; zL <= zLMax; zL += zLStep) {
                                        double dzL = static_cast<double>(zL) / factor;
                                        ++DCount;
                                        IOManager::WriteDownSampledPointCloudParameterList(DList, DCount, mD, nD, bS,
                                                                                           P1, P2, dMD, pFC, uR, sWS,
                                                                                           sR, mode, dzL);
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    DList.close();
}

void ParameterListGenerator::GenerateSparsePointCloudParameterList() {
    std::ofstream SList;
    std::string SListName = parameterListDir + "/SparsePointCloudParameterList.txt";
    SList.open(SListName);

    for (int zL = zLMin; zL <= zLMax; zL += zLStep) {
        ++SCount;
        double dzL = static_cast<double>(zL) / factor;
        IOManager::WriteSparsePointCloudParameterList(SList, SCount, dzL);
    }

    SList.close();
}

void ParameterListGenerator::GenerateOctoMapParameterList() {
    std::ofstream OList;
    std::string OListName = parameterListDir + "/OctoMapParameterList.txt";
    OList.open(OListName);

    for (int resMp = resMpMin; resMp <= resMpMax; resMp += resMpStep) {
        double dresMp = static_cast<double>(resMp) / factor;
        for (int cTMax = cTMaxMin; cTMax <= cTMaxMax; cTMax += cTMaxStep) {
            double dcTMax = static_cast<double>(cTMax) / factor;
            for (int pH = pHMin; pH <= pHMax; pH += pHStep) {
                if (cTMax >= pH) {
                    double dpH = static_cast<double>(pH) / factor;
                    for (int oT = oTMin; oT <= oTMax; oT += oTStep) {
                        double doT = static_cast<double>(oT) / factor;
                        for (int pM = pMMin; pM <= pMMax; pM += pMStep) {
                            double dpM = static_cast<double>(pM) / factor;
                            for (int cTMin = cTMinMin; cTMin <= cTMinMax; cTMin += cTMinStep) {
                                if (pM >= cTMin) {
                                    double dcTMin = static_cast<double>(cTMin) / factor;
                                    for (int fL = fLMin; fL <= fLMax; fL += fLStep) {
                                        double dfL = static_cast<double>(fL) / factor;
                                        if (oT <= cTMax && oT >= cTMin) {
                                            ++OCount;
                                            IOManager::WriteOctoMapParameterList(OList, OCount, dresMp, dcTMax, dpH,
                                                                                 doT, dpM, dcTMin, dfL);
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    OList.close();
}

void ParameterListGenerator::GenerateKNNParameterList() {
    std::ofstream KList;
    std::string KListName = parameterListDir + "/KNNParameterList.txt";
    KList.open(KListName);

    for (int resMp = resMpMin; resMp <= resMpMax; resMp += resMpStep) {
        double dresMp = static_cast<double>(resMp) / factor;
        for (int cTMax = cTMaxMin; cTMax <= cTMaxMax; cTMax += cTMaxStep) {
            double dcTMax = static_cast<double>(cTMax) / factor;
            for (int oT = oTMin; oT <= oTMax; oT += oTStep) {
                double doT = static_cast<double>(oT) / factor;
                for (int pM2 = pM2Min; pM2 <= pM2Max; pM2 += pM2Step) {
                    double dpM2 = static_cast<double>(pM2) / factor;
                    for (int pM = pMMin; pM <= pMMax; pM += pMStep) {
                        if (pM2 >= pM) {
                            double dpM = static_cast<double>(pM) / factor;
                            for (int cTMin = cTMinMin; cTMin <= cTMinMax; cTMin += cTMinStep) {
                                if (pM >= cTMin) {
                                    double dcTMin = static_cast<double>(cTMin) / factor;
                                    for (int pU = pUMin; pU <= pUMax; pU += pUStep) {
                                        double dpU = static_cast<double>(pU) / factor;
                                        for (int pW = pWMin; pW <= pWMax; pW += pWStep) {
                                            double dpW = static_cast<double>(pW) / factor;
                                            for (int k = kMin; k <= kMax; k += kStep) {
                                                for (int resS = resSMin; resS <= resSMax; resS += resSStep) {
                                                    double dresS = static_cast<double>(resS) / factor;
                                                    for (int fL = fLMin; fL <= fLMax; fL += fLStep) {
                                                        double dfL = static_cast<double>(fL) / factor;
                                                        if (oT <= cTMax && oT >= cTMin && pU>=pW) {
                                                            ++KCount;
                                                            IOManager::WriteKNNParameterList(KList, KCount, dresMp,
                                                                                             dcTMax,doT, dpM2, dpM, dcTMin,
                                                                                             dpU, dpW, k, dresS, dfL);
                                                        }
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    KList.close();
}

void ParameterListGenerator::GetSparsePointCloudParameterCombinationNumber() {
    size_t SNum = IOManager::F(zLMax, zLMin, zLStep);
    if (SNum == SCount) {
        std::cout << "Total number of combinations for sparse point cloud parameters: " << SNum << "." << std::endl;
    } else {
        std::cout << "Total number of combinations for sparse point cloud parameters check failed!" << std::endl;
    }
}

void ParameterListGenerator::GetDownSampledPointCloudParameterCombinationNumber() {
    int nmD = IOManager::F(mDMax, mDMin, mDStep);
    int nnD = IOManager::F(nDMax, nDMin, nDStep);
    int nbS = IOManager::F(bSMax, bSMin, bSStep);
    int ndMD = IOManager::F(dMDMax, dMDMin, dMDStep);
    int npFC = IOManager::F(pFCMax, pFCMin, pFCStep);
    int nuR = IOManager::F(uRMax, uRMin, uRStep);
    int nsWS = IOManager::F(sWSMax, sWSMin, sWSStep);
    int nsR = IOManager::F(sRMax, sRMin, sRStep);
    int nzL = IOManager::F(zLMax, zLMin, zLStep);

    size_t DNum = nmD * nnD * nbS * ndMD * npFC * nuR * nsWS * nsR * nzL;

    if (DNum == DCount) {
        std::cout << "Total number of combinations for down sampled point cloud parameters: " << DNum << "."
                  << std::endl;
    } else {
        std::cout << "Total number of combinations for down sampled point cloud parameters check failed!" << std::endl;
    }
}

void ParameterListGenerator::GetOctoMapParameterCombinationNumber() {
    int nresMp = IOManager::F(resMpMax, resMpMin, resMpStep);
    int nfL = IOManager::F(fLMax, fLMin, fLStep);

    int ncTMax = IOManager::F(cTMaxMax, cTMaxMin, cTMaxStep);
    int npH = IOManager::F(pHMax, pHMin, pHStep);
    int noT = IOManager::F(oTMax, oTMin, oTStep);
    int npM = IOManager::F(pMMax, pMMin, pMStep);
    int ncTMin = IOManager::F(cTMinMax, cTMinMin, cTMinStep);

    int sumA = 0;
    for (int i = 1; i <= ncTMax; ++i) {
        sumA += IOManager::G(cTMaxMin, i, cTMaxStep, pHMin, npH, pHStep);
    }

    int sumB = 0;
    for (int i = 1; i <= npM; ++i) {
        sumB += IOManager::G(pMMin, i, pMStep, cTMinMin, ncTMin, cTMinStep);
    }


    size_t ONum = nresMp * nfL * noT * sumA * sumB;

    if (ONum == OCount) {
        std::cout << "Total number of combinations for OctoMap mapping parameters: " << ONum << "." << std::endl;
    } else {
        std::cout << "Total number of combinations for OctoMap mapping parameters check failed!" << std::endl;
    }
}

void ParameterListGenerator::GetKNNParameterCombinationNumber() {

    int nresMp = IOManager::F(resMpMax, resMpMin, resMpStep);
    int nk = IOManager::F(kMax, kMin, kStep);
    int nresS = IOManager::F(resSMax, resSMin, resSStep);
    int nfL = IOManager::F(fLMax, fLMin, fLStep);

    int ncTMax = IOManager::F(cTMaxMax, cTMaxMin, cTMaxStep);
    int noT = IOManager::F(oTMax, oTMin, oTStep);
    int npM2 = IOManager::F(pM2Max, pM2Min, pM2Step);
    int npM = IOManager::F(pMMax, pMMin, pMStep);
    int ncTMin = IOManager::F(cTMinMax, cTMinMin, cTMinStep);
    int npU = IOManager::F(pUMax, pUMin, pUStep);
    int npW = IOManager::F(pWMax, pWMin, pWStep);

    int sumA = 0;
    for (int i = 1; i <= npM2; ++i) {
        int n = IOManager::G(pM2Min, i, pM2Step, pMMin, npM, pMStep);
        for (int j = 1; j <= n; ++j) {
            sumA += IOManager::G(pMMin, j, pMStep, cTMinMin, ncTMin, cTMinStep);
        }
    }

    size_t KNum = nresMp * nk * nresS * nfL * ncTMax * noT * npU * npW * sumA;

    if (KNum == KCount) {
        std::cout << "Total number of combinations for KNN mapping parameters: " << KNum << "." << std::endl;
    } else {
        std::cout << "Total number of combinations for KNN mapping parameters check failed!" << std::endl;
    }
}
