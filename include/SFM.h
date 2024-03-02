
#ifndef SFM_SFM_H
#define SFM_SFM_H

#include "common_include.h"
#include <unordered_map>
#include "Map.h"
#include "Frame.h"
#include "BA.h"


class SFM {

    Map::Ptr globalMap;
    cv::Ptr<cv::Feature2D> feature2D;
    cv::Ptr<cv::DescriptorMatcher> matcher;

    //相邻关键帧变量
    struct KeyFrame {
    public:
        typedef shared_ptr<KeyFrame> Ptr;
        Frame::Ptr frame;
        Mat image;
        vector<cv::KeyPoint> keyPoints;
        Mat descriptors;
        unordered_map<int, MapPoint::Ptr> inlierPoints;//在descriptors或keyPoints中的序号和对应的地图点

        KeyFrame(const Frame::Ptr &frame, const Mat &image) :
                frame(frame), image(image) {}
    };

    KeyFrame::Ptr lastKeyFrame, currKeyFrame;

public:
    SFM(const cv::Ptr<cv::Feature2D> &feature2D,
        const cv::Ptr<cv::DescriptorMatcher> &matcher
    ) :
            feature2D(feature2D),
            matcher(matcher),
            globalMap(new Map) {}

    void pipeline(const vector<string> &imagesDir, Camera::Ptr camera);

    ///2D-2D初始化

    //对两张图片提取、匹配、筛选特征点，求解对极约束，三角化
    void init(Mat &image1, Mat &image2, Camera::Ptr camera);

    ///3D-2D求解

    //添加新帧，提取、匹配、筛选和Map的特征点，PnP求解，提取、匹配、筛选和前一帧的特征点，三角化
    void step(Mat &image, const Camera::Ptr &camera);

    ///基本处理

    //加载新帧
    void addImage(Mat &image, const Camera::Ptr &camera);

    //存储新帧
    void moveFrame();

    //检测特征点，提取描述子
    void detectAndCompute();

    //匹配、筛选特征点
    void matchAndFilter(vector<cv::DMatch> &matches);

    //转换齐次坐标点，保存到Map
    void convAndAddMappoints(
            const Mat &inlierMask, const Mat &points4D, const vector<cv::DMatch> &matches);

    //筛选匹配点
    void filtMatches(vector<cv::DMatch> &matches);
};


#endif //SFM_SFM_H
