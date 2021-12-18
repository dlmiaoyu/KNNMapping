#ifndef PARAMETERLISTGENERATOR_HPP
#define PARAMETERLISTGENERATOR_HPP

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <fstream>
#include <boost/filesystem.hpp>
#include "IOManager.hpp"

class ParameterListGenerator {
public:
    explicit ParameterListGenerator(const cv::FileStorage &fSettings);

    void Run();

private:
    const std::string parameterListDir;

    const int imageChannelNum;

    const int mDStep;
    const int mDMin;
    const int mDMax;
    const int nDStep;
    const int nDMin;
    const int nDMax;
    const int bSStep;
    const int bSMin;
    const int bSMax;
    const int dMDStep;
    const int dMDMin;
    const int dMDMax;
    const int pFCStep;
    const int pFCMin;
    const int pFCMax;
    const int uRStep;
    const int uRMin;
    const int uRMax;
    const int sWSStep;
    const int sWSMin;
    const int sWSMax;
    const int sRStep;
    const int sRMin;
    const int sRMax;
    const bool mode;

    const int factor;
    const int zLStep;
    const int zLMin;
    const int zLMax;
    const int fLStep;
    const int fLMin;
    const int fLMax;

    const int resMpStep;
    const int resMpMin;
    const int resMpMax;
    const int cTMaxStep;
    const int cTMaxMin;
    const int cTMaxMax;
    const int pHStep;
    const int pHMin;
    const int pHMax;
    const int oTStep;
    const int oTMin;
    const int oTMax;
    const int pM2Step;
    const int pM2Min;
    const int pM2Max;
    const int pMStep;
    const int pMMin;
    const int pMMax;
    const int cTMinStep;
    const int cTMinMin;
    const int cTMinMax;

    const int pUStep;
    const int pUMin;
    const int pUMax;
    const int pWStep;
    const int pWMin;
    const int pWMax;

    const int kStep;
    const int kMin;
    const int kMax;
    const int resSStep;
    const int resSMin;
    const int resSMax;

    size_t DCount = 0;
    size_t SCount = 0;
    size_t OCount = 0;
    size_t KCount = 0;

    void GenerateDownSampledPointCloudParameterList();

    void GenerateSparsePointCloudParameterList();

    void GenerateOctoMapParameterList();

    void GenerateKNNParameterList();

    void GetDownSampledPointCloudParameterCombinationNumber();

    void GetSparsePointCloudParameterCombinationNumber();

    void GetOctoMapParameterCombinationNumber();

    void GetKNNParameterCombinationNumber();
};

#endif //PARAMETERLISTGENERATOR_HPP
