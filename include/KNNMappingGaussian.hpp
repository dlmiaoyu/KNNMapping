#ifndef KNNMAPPINGGAUSSIAN_HPP
#define KNNMAPPINGGAUSSIAN_HPP

#include <iostream>
#include <vector>

//pcl
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/octree/octree_search.h>

//octomap
#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>

class KNNMappingGaussian {
public:
    KNNMappingGaussian(const double &resMp, const double &cTMax, const double &oT, const double &pM2, const double &pM,
               const double &cTMin, const double &pU, const double &pW, const int &k, const double &resS,
               const double &fL, const double &mu, const double &sigma, std::string pointCloudDir,
               std::string poseFileName);

    octomap::ColorOcTree map;

    void Run();

    static void
    GenerateInnerPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &keyFramePointCloud, const double (&pose)[7],
                            const double &squaredLimit, pcl::PointCloud<pcl::PointXYZ>::Ptr &innerPointCloud);

    static void GetKNNAndDistance(pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> &pointCloudOctree,
                                  const pcl::PointXYZ &searchPoint, const int &k, double &d);

private:
    //OctoMap parameters
    const double mresMp;
    const double mcTMax;
    const double moT;
    const double mpM2;
    const double mpM;
    const double mcTMin;

    //kNN model parameters
    const double mpU;
    const double mpW;

    //pcl octree
    const int mk;      //real number of neighbours
    const double mresS;

    const double mfL;

    const double mmu;
    const double msigma;

    const std::string mpointCloudDir;
    const std::string mposeFileName;

    float logFreeProb;

    void UpdateMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr &keyFramePointCloud,
                   const pcl::PointCloud<pcl::PointXYZ>::Ptr &innerPointCloud, const double (&pose)[7]);

    void CalculateLogProbability(const double &d, float &logProb) const;
};

#endif //KNNMAPPINGGAUSSIAN_HPP