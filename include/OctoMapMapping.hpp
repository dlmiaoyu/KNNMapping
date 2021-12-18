#ifndef OCTOMAPMAPPING_HPP
#define OCTOMAPMAPPING_HPP

#include <iostream>
#include <ctime>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>

class OctoMapMapping {
public:
    OctoMapMapping(const double &resMp, const double &cTMax, const double &pH, const double &oT, const double &pM,
                   const double &cTMin, const double &fL, std::string pointCloudDir, std::string poseFileName);

    octomap::ColorOcTree map;

    void Run();

private:
    //OctoMap parameters
    const double mresMp;
    const double mcTMax;
    const double mpH;
    const double moT;
    const double mpM;
    const double mcTMin;
    const double mfL;

    const std::string mpointCloudDir;
    const std::string mposeFileName;
};

#endif //OCTOMAPMAPPING_HPP
