
#ifndef SFM_COMMON_INCLUDE_H
#define SFM_COMMON_INCLUDE_H

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
using Eigen::Vector2d;
using Eigen::Vector3d;

// Sophus
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>
using Sophus::SO3d;
using Sophus::SE3d;

// opencv
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
using cv::Mat;

// std
#include <vector>
#include <list>
#include <memory>
#include <string>
#include <iostream>
#include <set>
#include <unordered_map>
#include <map>

using namespace std;

#endif
