
#include "SFM.h"


void SFM::pipeline(const vector<string> &imagesDir, Camera::Ptr camera)
{

    auto imageDirIt = imagesDir.begin();
    Mat image1 = cv::imread(*imageDirIt++);
    Mat image2 = cv::imread(*imageDirIt++);

    // 2D-2D
    init(image1, image2, camera);

    // 可视化初始地图点
    // #ifdef PCL_VIEWER_DEBUG
    //         map->visInCloudViewer();
    // #endif

    // 3D-2D

    for (; imageDirIt != imagesDir.end(); ++imageDirIt)
    {
        Mat image = cv::imread(*imageDirIt);
        cout << endl << "============== Adding image: " + *imageDirIt << "==============" << endl;

        step(image, camera);
    }

    // 输出点云信息
    // #ifdef DEBUG
    //         for (auto &mapPoints:map->mapPoints) {
    //             cout << "MapPoint " << mapPoints->getPosPoint3_CV<float>() << endl;
    //             cout << " has " << mapPoints->observedFrames.size() << " oberved frames" << endl;
    //         }
    // #endif

    // 可视化点云
// #ifdef PCL_VIEWER_DEBUG
//     globalMap->visInCloudViewer();
// #endif

    // BA
    BA ba;
    ba(globalMap);

    // 可视化点云
#ifdef PCL_VIEWER_DEBUG
    globalMap->visInCloudViewer();
#endif
    globalMap->savePoints();
}

void SFM::init(Mat &image1, Mat &image2, Camera::Ptr camera)
{
    // 2D-2D
#ifdef DEBUG
    cout << endl << "==============2D-2D initializing==============" << endl;
#endif
    // 检测特征点并匹配
    addImage(image1, camera);
    detectAndCompute();
    moveFrame();    // curr move to last
    addImage(image2, camera);
    detectAndCompute();
    // 筛选匹配点
    vector<cv::DMatch> matches;
    matchAndFilter(matches);

    // 解对极约束并三角化
    vector<cv::Point2f> matchPoints1, matchPoints2;
    for (auto match : matches)
    {
        matchPoints1.push_back(lastKeyFrame->keyPoints[match.queryIdx].pt);
        matchPoints2.push_back(currKeyFrame->keyPoints[match.trainIdx].pt);
    }

    Mat essentialMatrix, inlierMask;
    ;
    essentialMatrix = findEssentialMat(matchPoints1, matchPoints2,
                                       currKeyFrame->frame->camera->getFocalLength(),
                                       currKeyFrame->frame->camera->getPrincipalPoint(),
                                       cv::RANSAC, 0.999, 1.0, inlierMask);
#ifdef DEBUG
    int nPointsFindEssentialMat = countNonZero(inlierMask);
    cout << "findEssentialMat: \n\t" << nPointsFindEssentialMat << " valid points, " << (float)nPointsFindEssentialMat * 100 / matchPoints1.size()
         << "% of " << matchPoints1.size() << " points are used" << endl;
#endif

    // CV可视化用于三角化的点
#ifdef CV_IMSHOW_DEBUG
    vector<cv::DMatch> inlierMatches;
    vector<cv::KeyPoint> inlierKeyPoints1, inlierKeyPoints2;
    for (int i = 0; i < matches.size(); ++i)
    {
        if (!inlierMask.at<uint8_t>(i, 0))
            continue;
        inlierMatches.push_back(matches[i]);
        inlierMatches.back().trainIdx = inlierKeyPoints1.size();
        inlierMatches.back().queryIdx = inlierKeyPoints2.size();
        inlierKeyPoints1.push_back(lastKeyFrame->keyPoints[matches[i].queryIdx]);
        inlierKeyPoints2.push_back(currKeyFrame->keyPoints[matches[i].trainIdx]);
    }

    cv::Mat imageMatchs;
    cv::drawMatches(lastKeyFrame->image, inlierKeyPoints1, currKeyFrame->image, inlierKeyPoints2, inlierMatches, imageMatchs);
    cv::resize(imageMatchs, imageMatchs, cv::Size(), 0.25, 0.25);
    cv::imshow("Matches", imageMatchs);
    cv::waitKey(0);
#endif

    // 解currKeyFrame的R、t并计算se3, 三角化
    Mat R, t, points4D;
    recoverPose(essentialMatrix, matchPoints1, matchPoints2,
                currKeyFrame->frame->camera->getKMatxCV(), R, t, 100, inlierMask,
                points4D);
    Eigen::Matrix3d eigenR;
    cv2eigen(R, eigenR);
    currKeyFrame->frame->Tcw = SE3d(
        eigenR,
        Vector3d(t.at<double>(0, 0), t.at<double>(1, 0), t.at<double>(2, 0)));

#ifdef DEBUG
    int nPointsRecoverPose = countNonZero(inlierMask);
    cout << "recoverPose: \n\t" << nPointsRecoverPose << " valid points, " << (float)nPointsRecoverPose * 100 / matchPoints1.size()
         << "% of " << matchPoints1.size() << " points are used" << endl;
        // cout << "2D-2D currFrame R: " << R.size << endl << R << endl;
        // cout << "2D-2D currFrame t: " << t.size << endl << t << endl;
        cout << "2D-2D currFrame SE3: " << endl << currKeyFrame->frame->Tcw.matrix() << endl;
#endif

    // 保存三角化后的点到地图
    convAndAddMappoints(inlierMask, points4D, matches);

    moveFrame();
}

void SFM::step(Mat &image, const Camera::Ptr &camera)
{

    addImage(image, camera);

    // 检测特征点
    detectAndCompute();
    // 提取地图的特征点
    Mat descriptorsMap;
    vector<cv::Point3f> points3D;
    for (MapPoint::Ptr &point : globalMap->mapPoints)
    {
        if (lastKeyFrame->frame->isInFrame(point->pos))
        {
            points3D.push_back(point->getPosPoint3_CV<float>());
            descriptorsMap.push_back(point->descriptor);
        }
    }
#ifdef DEBUG
    cout << "found " << points3D.size() << " 3D points in the last frame" << endl;
#endif
    // 匹配地图特征点
    vector<cv::DMatch> matches;
    matcher->match(descriptorsMap, currKeyFrame->descriptors, matches, cv::noArray());
#ifdef DEBUG
    cout << "found " << matches.size() << " keypoints matched with 3D points" << endl;
#endif

    // 筛选匹配点
    filtMatches(matches);

    // 解PnP得相机位姿
    vector<cv::Point2f> points2DPnP;
    vector<cv::Point3f> points3DPnP;
    for (auto match : matches)
    {
        points2DPnP.push_back(currKeyFrame->keyPoints[match.trainIdx].pt);
        points3DPnP.push_back(points3D[match.queryIdx]);
    }
    Mat r, t, indexInliers;
    solvePnPRansac(points3DPnP, points2DPnP, camera->getKMatxCV(),
                   cv::noArray(), r, t, false, 100, 8.0, 0.99,
                   indexInliers);
    Mat R;
    cv::Rodrigues(r, R);

    Eigen::Vector3d angleAxis(r.at<double>(0, 0),
                              r.at<double>(1, 0),
                              r.at<double>(2, 0)); // 提取角轴部分

    Sophus::SO3d rotationMatrix = Sophus::SO3d::exp(angleAxis); // 使用角轴创建旋转矩阵

    currKeyFrame->frame->Tcw = SE3d(
        rotationMatrix,
        Vector3d(t.at<double>(0, 0), t.at<double>(1, 0), t.at<double>(2, 0)));

#ifdef DEBUG
    cout << "solvePnPRansac: \n\t" << indexInliers.rows << " valid points, " << (float)indexInliers.rows * 100 / points2DPnP.size()
         << "% of " << points2DPnP.size() << " points are used" << endl;
        // cout << "3D-2D currFrame R: " << R.size << endl << R << endl;
        // cout << "3D-2D currFrame t: " << t.size << endl << t << endl;
        cout << "3D-2D currFrame SE3: " << endl << currKeyFrame->frame->Tcw.matrix() << endl;

#endif

    if ((float)indexInliers.rows < 30)
    {
        cout << "current frame has bad matched points!" << endl;
        return;
    }

    // 匹配帧间特征点
    matchAndFilter(matches);

    // 通过findEssentialMat筛选异常点
    vector<cv::Point2f> matchPoints1, matchPoints2;
    for (auto match : matches)
    {
        matchPoints1.push_back(lastKeyFrame->keyPoints[match.queryIdx].pt);
        matchPoints2.push_back(currKeyFrame->keyPoints[match.trainIdx].pt);
    }
    Mat inlierMask;
    findEssentialMat(matchPoints1, matchPoints2,
                     currKeyFrame->frame->camera->getFocalLength(),
                     currKeyFrame->frame->camera->getPrincipalPoint(),
                     cv::RANSAC, 0.999, 1.0, inlierMask);
#ifdef DEBUG
    int nPointsFindEssentialMat = countNonZero(inlierMask);
    cout << "After EssentialMat Filter: \n\t" << nPointsFindEssentialMat << " valid points, " << (float)nPointsFindEssentialMat * 100 / matchPoints1.size()
         << "% of " << matchPoints1.size() << " points are used" << endl;
#endif

    // 三角化
    vector<cv::Point2f> matchPointsNorm1, matchPointsNorm2;
    matchPointsNorm1.reserve(matches.size());
    matchPointsNorm2.reserve(matches.size());
    for (auto &match : matches)
    {
        matchPointsNorm1.push_back(lastKeyFrame->frame->camera->pixel2normal(lastKeyFrame->keyPoints[match.queryIdx].pt));
        matchPointsNorm2.push_back(currKeyFrame->frame->camera->pixel2normal(currKeyFrame->keyPoints[match.trainIdx].pt));
        /*#ifdef DEBUG
                    if (i < 5) {
                        cout << lastKeyFrame->matchPoints[i] << endl;
                        cout << matchPointsNorm1.back() << endl << endl;
                    }
        #endif*/
    }
    Mat points4D;
    triangulatePoints(lastKeyFrame->frame->getTcw34MatCV(CV_32F), currKeyFrame->frame->getTcw34MatCV(CV_32F),
                      matchPointsNorm1, matchPointsNorm2, points4D);
    /*        triangulatePoints(lastKeyFrame->frame->getProjMatCV(), currKeyFrame->frame->getProjMatCV(),
                              lastKeyFrame->matchPoints, lastKeyFrame->matchPoints, points4D);*/

    // 转换齐次坐标点，保存到Map，并做局部BA
    convAndAddMappoints(inlierMask, points4D, matches);

    moveFrame();
}

void SFM::convAndAddMappoints(const Mat &inlierMask, const Mat &points4D,
                              const vector<cv::DMatch> &matches)
{ // 归一化齐次坐标点,转换Mat
#ifdef DEBUG
    cout << "convAndAddMappoints: " << endl;
#endif
    Map::Ptr localMap(new Map);
    // 建立小地图
    if (lastKeyFrame && currKeyFrame)
    {
        localMap->addFrame(lastKeyFrame->frame);
        localMap->addFrame(currKeyFrame->frame);
    }
#ifdef DEBUG
    int numOldMappoints = localMap->mapPoints.size();
    // cout << "showing 5 samples of 3D points:" << endl;
#endif
    for (int i = 0; i < points4D.cols; ++i)
    {
        MapPoint::Ptr mapPoint;

        // 如果是outlier，跳过
        if (!inlierMask.empty() && !inlierMask.at<uint8_t>(i, 0))
            continue;

        // 获取描述子
        Mat descriptor = currKeyFrame->descriptors.row(matches[i].trainIdx);

        // 如果是上一帧加到地图中的点，更新描述子、加入观测帧后跳过
        if (lastKeyFrame->inlierPoints.find(matches[i].queryIdx) != lastKeyFrame->inlierPoints.end())
        {
            mapPoint = lastKeyFrame->inlierPoints[matches[i].queryIdx];
            // 更新描述子
            mapPoint->descriptor = descriptor;
            // 加入观测帧
            mapPoint->addObervedFrame(
                currKeyFrame->frame, currKeyFrame->keyPoints[matches[i].trainIdx].pt);
            // 记录当前帧加入地图的mapPoint和特征点下标
            currKeyFrame->inlierPoints[matches[i].trainIdx] = mapPoint;
        }
        // 向地图中加入当前帧的地图点
        else
        {
            // 转换齐次localMap坐标
            Mat x = points4D.col(i);

            // 向地图增加点
            // 获取颜色
            cv::Vec3b rgb;
            if (currKeyFrame->image.type() == CV_8UC3)
            {
                rgb = currKeyFrame->image.at<cv::Vec3b>(currKeyFrame->keyPoints[matches[i].trainIdx].pt);
                swap(rgb[0], rgb[2]);
            }
            else if (currKeyFrame->image.type() == CV_8UC1)
            {
                cvtColor(currKeyFrame->image.at<uint8_t>(currKeyFrame->keyPoints[matches[i].trainIdx].pt),
                         rgb,
                         cv::COLOR_GRAY2RGB);
            }

            if (x.type() == CV_32FC1)
            {
                x /= x.at<float>(3, 0); // 归一化
                mapPoint = MapPoint::Ptr(new MapPoint(Vector3d(x.at<float>(0, 0),
                                                               x.at<float>(1, 0),
                                                               x.at<float>(2, 0)),
                                                      descriptor, rgb));
            }
            else if (x.type() == CV_64FC1)
            {
                x /= x.at<double>(3, 0);
                mapPoint = MapPoint::Ptr(new MapPoint(Vector3d(x.at<double>(0, 0),
                                                               x.at<double>(1, 0),
                                                               x.at<double>(2, 0)),
                                                      descriptor, rgb));
            }

            // 记录当前帧加入地图的mapPoint和特征点下标
            currKeyFrame->inlierPoints[matches[i].trainIdx] = mapPoint;

            /*#ifdef DEBUG
                        if (i < 5)
                            cout << mapPoint->pos << endl << endl;
            #endif*/
            mapPoint->addObervedFrame(lastKeyFrame->frame, lastKeyFrame->keyPoints[matches[i].queryIdx].pt);
            mapPoint->addObervedFrame(currKeyFrame->frame, currKeyFrame->keyPoints[matches[i].trainIdx].pt);
            globalMap->addMapPoint(mapPoint);
            // 加入到小地图
            localMap->addMapPoint(mapPoint);
        }
    }
#ifdef DEBUG
    cout << "\t" << localMap->mapPoints.size() - numOldMappoints << " new 3D points added to the map" << endl
         << "\t" << localMap->mapPoints.size() << " in total" << endl;
#endif
    BA ba;
    ba(localMap);
}

void SFM::matchAndFilter(vector<cv::DMatch> &matches)
{
#ifdef DEBUG
    cout << "matchAndFilter keypoints: " << endl;
#endif
    matcher->match(lastKeyFrame->descriptors, currKeyFrame->descriptors, matches, cv::noArray());
#ifdef DEBUG
    cout << "\tfound " << matches.size() << " keypoints matched with last frame" << endl;
#endif
    // 筛选匹配点
    filtMatches(matches);

#ifdef DEBUG
    cout << "\tfound " << matches.size() << " good matches" << endl;
#endif

// #ifdef CV_IMSHOW_DEBUG
//     cv::Mat imageMatchs;
//     cv::drawMatches(lastKeyFrame->image, lastKeyFrame->keyPoints, currKeyFrame->image, currKeyFrame->keyPoints, matches, imageMatchs);
//     cv::resize(imageMatchs, imageMatchs, cv::Size(), 0.25, 0.25);
//     cv::imshow("Matches", imageMatchs);
//     cv::waitKey(0);
// #endif
}

void SFM::filtMatches(vector<cv::DMatch> &matches)
{
    auto minMaxDis = minmax_element(
        matches.begin(), matches.end(),
        [](const cv::DMatch &m1, const cv::DMatch &m2)
        {
            return m1.distance < m2.distance;
        });
    auto minDis = minMaxDis.first->distance;
    auto maxDis = minMaxDis.second->distance;
    vector<cv::DMatch> goodMatches;
    for (auto match : matches)
    {
        if (match.distance <= 4 * minDis)
            goodMatches.push_back(match);
    }
    matches = goodMatches;
#ifdef DEBUG
    cout << "\tfound " << matches.size() << " good matches" << endl;
#endif
}

void SFM::detectAndCompute()
{
#ifdef DEBUG
    cout << "detectAndCompute features: " << endl;
#endif
    feature2D->detect(currKeyFrame->image, currKeyFrame->keyPoints, cv::noArray());
    feature2D->compute(currKeyFrame->image, currKeyFrame->keyPoints, currKeyFrame->descriptors);
}

void SFM::addImage(Mat &image, const Camera::Ptr &camera)
{
    // 加载新帧
    Frame::Ptr frame(new Frame(camera, image));
    currKeyFrame = KeyFrame::Ptr(new KeyFrame(frame, image));   //此处image仅传递行高列宽
}

void SFM::moveFrame()
{
    globalMap->addFrame(currKeyFrame->frame);
    lastKeyFrame = currKeyFrame;
}
