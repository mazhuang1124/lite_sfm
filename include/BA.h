
#ifndef SFM_BA_H
#define SFM_BA_H

#include "common_include.h"
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include "Map.h"
#include "Frame.h"
#include <unordered_map>

using namespace std;

class BA {

    class ReprojectCost {
        cv::Point2d observation;
    public:
        ReprojectCost(const cv::Point2d &observation) : observation(observation) {}

        template<typename T>
        bool operator()(const T *const intrinsic, const T *const extrinsic, const T *const pos3d, T *residuals) const {
            const T *r = extrinsic;
            const T *t = &extrinsic[3];

            T pos_proj[3];
            ceres::AngleAxisRotatePoint(r, pos3d, pos_proj);    //旋转向量, 待旋转的3D点, 旋转后的3D点

            // Apply the camera translation
            pos_proj[0] += t[0];
            pos_proj[1] += t[1];
            pos_proj[2] += t[2];

            const T x = pos_proj[0] / pos_proj[2];
            const T y = pos_proj[1] / pos_proj[2];

            const T fx = intrinsic[0];
            const T fy = intrinsic[1];
            const T cx = intrinsic[2];
            const T cy = intrinsic[3];

            // Apply intrinsic
            const T u = fx * x + cx;
            const T v = fy * y + cy;

            residuals[0] = u - T(observation.x);
            residuals[1] = v - T(observation.y);

            return true;
        }
    };

    Map::Ptr map;
    ceres::Solver::Options ceres_config_options;

    unordered_map<Camera::Ptr, cv::Matx14d> cameraIntrinsics;
    unordered_map<Frame::Ptr, cv::Matx23d> frameExtrinsics;
    unordered_map<MapPoint::Ptr, cv::Matx13d> mapPointsPos;

    void loadMap();

    void bundleAdjustment();

    void updateMap();

    void clear();

public:

    BA();

    void operator()(Map::Ptr &map);

};


#endif 
