
#ifndef SFM_CAMERA_H
#define SFM_CAMERA_H

#include "common_include.h"


class Camera {
public:
    typedef shared_ptr<Camera> Ptr;
    float fx, fy, cx, cy;

    Camera(float fx, float fy, float cx, float cy) :
            fx(fx), fy(fy), cx(cx), cy(cy) {}

    template<typename T>
    void setIntrinsic(cv::Matx<T, 1, 4> intrinsic) {
        fx=intrinsic(0);
        fy=intrinsic(1);
        cx=intrinsic(2);
        cy=intrinsic(3);
    }

    //坐标转换

    Vector3d world2camera(const Vector3d &p_w, const SE3d &T_c_w);

    Vector3d camera2world(const Vector3d &p_c, const SE3d &T_c_w);

    Vector2d camera2pixel(const Vector3d &p_c);

    Vector3d pixel2camera(const Vector2d &p_p, double depth = 1);

    Vector3d pixel2world(const Vector2d &p_p, const SE3d &T_c_w, double depth = 1);

    Vector2d world2pixel(const Vector3d &p_w, const SE3d &T_c_w);

    cv::Point2f pixel2normal(const cv::Point2d &p) const;

    //获取参数
    float getFocalLength() {
        return (fx + fy) / 2;
    }

    cv::Point2d getPrincipalPoint() {
        return cv::Point2d(cx, cy);
    }

    cv::Matx<float, 3, 3> getKMatxCV() {
        return cv::Matx<float, 3, 3>(fx, 0, cx, 0, fy, cy, 0, 0, 1);
    }

    cv::Mat getKMatCV() {
        Mat K = (cv::Mat_<float>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
        return K;
    }
};


#endif
