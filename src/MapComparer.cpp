#include "MapComparer.hpp"

MapComparer::MapComparer(const octomap::ColorOcTree &map, const octomap::ColorOcTree &referenceMap,
                         const bool &writeMapFile)
        : mmap(&map), mreferenceMap(&referenceMap), mwriteMapFile(writeMapFile),
          mapVsRefMap(map.getResolution()), res(map.getResolution()) {
}

void MapComparer::Run() {
    octomap::ColorOcTree referenceMap(mreferenceMap->getResolution());
    CopyReferenceMap(referenceMap);

    double xMax, yMax, zMax;
    mmap->getMetricMax(xMax, yMax, zMax);

    double xMin, yMin, zMin;
    mmap->getMetricMin(xMin, yMin, zMin);

    int xNum = static_cast<int>(lround((xMax - xMin) / res));
    int yNum = static_cast<int>(lround((yMax - yMin) / res));
    int zNum = static_cast<int>(lround((zMax - zMin) / res));

    std::vector<octomap::point3d> pointVec = {
            {static_cast<float>(res),  static_cast<float>(res),  static_cast<float>(res)},
            {static_cast<float>(res),  0.0,                      static_cast<float>(res)},
            {static_cast<float>(res),  -static_cast<float>(res), static_cast<float>(res)},
            {0.0,                      static_cast<float>(res),  static_cast<float>(res)},
            {0.0,                      0.0,                      static_cast<float>(res)},
            {0.0,                      -static_cast<float>(res), static_cast<float>(res)},
            {-static_cast<float>(res), static_cast<float>(res),  static_cast<float>(res)},
            {-static_cast<float>(res), 0.0,                      static_cast<float>(res)},
            {-static_cast<float>(res), -static_cast<float>(res), static_cast<float>(res)},
            {static_cast<float>(res),  static_cast<float>(res),  0.0},
            {static_cast<float>(res),  0.0,                      0.0},
            {static_cast<float>(res),  -static_cast<float>(res), 0.0},
            {0.0,                      static_cast<float>(res),  0.0},
            {0.0,                      0.0,                      0.0},
            {0.0,                      -static_cast<float>(res), 0.0},
            {-static_cast<float>(res), static_cast<float>(res),  0.0},
            {-static_cast<float>(res), 0.0,                      0.0},
            {-static_cast<float>(res), -static_cast<float>(res), 0.0},
            {static_cast<float>(res),  static_cast<float>(res),  -static_cast<float>(res)},
            {static_cast<float>(res),  0.0,                      -static_cast<float>(res)},
            {static_cast<float>(res),  -static_cast<float>(res), -static_cast<float>(res)},
            {0.0,                      static_cast<float>(res),  -static_cast<float>(res)},
            {0.0,                      0.0,                      -static_cast<float>(res)},
            {0.0,                      -static_cast<float>(res), -static_cast<float>(res)},
            {-static_cast<float>(res), static_cast<float>(res),  -static_cast<float>(res)},
            {-static_cast<float>(res), 0.0,                      -static_cast<float>(res)},
            {-static_cast<float>(res), -static_cast<float>(res), -static_cast<float>(res)}};

    if (!mwriteMapFile) {
        for (int k = 0; k < zNum; ++k) {
            double z = zMax - k * res - res / 2;
            for (int i = 0; i < xNum; ++i) {
                double x = xMax - i * res - res / 2;
                for (int j = 0; j < yNum; ++j) {
                    double y = yMax - j * res - res / 2;
                    octomap::point3d point(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z));
                    octomap::ColorOcTreeNode *nodeInMap = mmap->search(point, 0);
                    octomap::ColorOcTreeNode *nodeInReferenceMap = mreferenceMap->search(point, 0);
                    if (nodeInMap != nullptr) {
                        if (nodeInReferenceMap != nullptr) {
                            if (mmap->isNodeOccupied(nodeInMap)) {
                                if (IsPointConnected(point, pointVec, referenceMap)) {
//                                if(mreferenceMap->isNodeOccupied(nodeInReferenceMap)){
                                    ++TP;
                                } else {
                                    ++FP;
                                }
                            } else {
                                if (!mreferenceMap->isNodeOccupied(nodeInReferenceMap)) {
                                    ++TN;
                                } else {
                                    ++FN;
                                }
                            }
                        } else {
                            ++unknown;
                        }
                    }
                }
            }
        }
    } else {
        for (int k = 0; k < zNum; ++k) {
            double z = zMax - k * res - res / 2;
            for (int i = 0; i < xNum; ++i) {
                double x = xMax - i * res - res / 2;
                for (int j = 0; j < yNum; ++j) {
                    double y = yMax - j * res - res / 2;
                    octomap::point3d point(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z));
                    octomap::ColorOcTreeNode *nodeInMap = mmap->search(point, 0);
                    octomap::ColorOcTreeNode *nodeInReferenceMap = mreferenceMap->search(point, 0);
                    if (nodeInMap != nullptr) {
                        if (nodeInReferenceMap != nullptr) {
                            if (mmap->isNodeOccupied(nodeInMap)) {
                                if (IsPointConnected(point, pointVec, referenceMap)) {
//                                if (mreferenceMap->isNodeOccupied(nodeInReferenceMap)) {
                                    ++TP;
                                    mapVsRefMap.updateNode(point, true, false);
                                } else {
                                    ++FP;
                                    mapVsRefMap.updateNode(point, true, false);
                                }
                            } else {
                                if (!mreferenceMap->isNodeOccupied(nodeInReferenceMap)) {
                                    ++TN;
                                    mapVsRefMap.updateNode(point, false, false);
                                } else {
                                    ++FN;
                                    mapVsRefMap.updateNode(point, true, false);
                                }
                            }
                        } else {
                            ++unknown;
                            mapVsRefMap.updateNode(point, true, false);
                        }
                    }
                }
            }
        }
        mapVsRefMap.updateInnerOccupancy();
        mapVsRefMap.expand();

        octomap::ColorOcTree referenceMap2(mreferenceMap->getResolution());
        CopyReferenceMap(referenceMap2);
        for (int k = 0; k < zNum; ++k) {
            double z = zMax - k * res - res / 2;
            for (int i = 0; i < xNum; ++i) {
                double x = xMax - i * res - res / 2;
                for (int j = 0; j < yNum; ++j) {
                    double y = yMax - j * res - res / 2;
                    octomap::point3d point(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z));
                    octomap::ColorOcTreeNode *nodeInMap = mmap->search(point, 0);
                    octomap::ColorOcTreeNode *nodeInReferenceMap = mreferenceMap->search(point, 0);
                    if (nodeInMap != nullptr) {
                        if (nodeInReferenceMap != nullptr) {
                            if (mmap->isNodeOccupied(nodeInMap)) {
                                if (IsPointConnected(point, pointVec, referenceMap2)) {
//                                if (mreferenceMap->isNodeOccupied(nodeInReferenceMap)) {
                                    mapVsRefMap.integrateNodeColor(point.x(), point.y(), point.z(), 0, 0, 255);
                                } else {
                                    mapVsRefMap.integrateNodeColor(point.x(), point.y(), point.z(), 255, 0, 0);
                                }
                            } else {
                                if (mreferenceMap->isNodeOccupied(nodeInReferenceMap)) {
                                    mapVsRefMap.integrateNodeColor(point.x(), point.y(), point.z(), 255, 255, 0);
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

bool MapComparer::IsPointConnected(const octomap::point3d &point, const std::vector<octomap::point3d> &pointVec,
                                   octomap::ColorOcTree &referenceMap) {
    for (const auto &delta:pointVec) {
        octomap::ColorOcTreeNode *neighbourNode = referenceMap.search(point + delta);
        if (neighbourNode != nullptr) {
            if (referenceMap.isNodeOccupied(neighbourNode)) {
                referenceMap.deleteNode(point + delta);
                return true;
            }
        }
    }

    return false;
}

void MapComparer::CopyReferenceMap(octomap::ColorOcTree &referenceMap) {
    for (auto it = mreferenceMap->begin_leafs(); it != mreferenceMap->end_leafs(); ++it) {
        octomap::point3d pt = it.getCoordinate();
        octomap::ColorOcTreeNode *node = mreferenceMap->search(pt);
        referenceMap.updateNode(pt, mreferenceMap->isNodeOccupied(node), false);
    }
    referenceMap.updateInnerOccupancy();
    referenceMap.expand();
}