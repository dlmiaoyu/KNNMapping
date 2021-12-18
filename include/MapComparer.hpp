#ifndef MAPCOMPARER_HPP
#define MAPCOMPARER_HPP

#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>

class MapComparer {
public:
    MapComparer(const octomap::ColorOcTree &map, const octomap::ColorOcTree &referenceMap, const bool &writeMapFile);

    octomap::ColorOcTree mapVsRefMap;
    double res;

    size_t TP = 0;
    size_t FP = 0;
    size_t TN = 0;
    size_t FN = 0;
    size_t unknown = 0;

    void Run();

    static bool IsPointConnected(const octomap::point3d &point, const std::vector<octomap::point3d> &pointVec,
                          octomap::ColorOcTree &referenceMap);

    void CopyReferenceMap(octomap::ColorOcTree &referenceMap);

private:
    const octomap::ColorOcTree *mmap;
    const octomap::ColorOcTree *mreferenceMap;

    const bool mwriteMapFile;
};

#endif //MAPCOMPARER_HPP
