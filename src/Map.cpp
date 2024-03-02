
#include "Map.h"

// #ifdef PCL_VIEWER_DEBUG
#include <pcl/common/common_headers.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
// #endif


void Map::addFrame(Frame::Ptr frame) {
    if (frame)
        frames.push_back(frame);
}

void Map::addMapPoint(MapPoint::Ptr mapPoint) {
    if (mapPoint)
        mapPoints.push_back(mapPoint);
}

void Map::savePoints(){

    // string file_path = "./cloud.pcd";
    string file_path = "./cloud.ply";

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (const auto& point : mapPoints) {
        pcl::PointXYZRGB pointXYZ(point->rgb[0], point->rgb[1], point->rgb[2]);
        pointXYZ.x = point->pos(0);
        pointXYZ.y = point->pos(1);
        pointXYZ.z = point->pos(2);
        cloud->push_back(pointXYZ);
    }
    // 创建PLY写入器
    pcl::io::savePLYFile(file_path, *cloud);

    // // 或者保存PCD文件
    // pcl::io::savePCDFile(file_path, *cloud, true);

}


#ifdef PCL_VIEWER_DEBUG
void Map::visInCloudViewer() {

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (auto point:mapPoints) {
        pcl::PointXYZRGB pointXYZ(point->rgb[0], point->rgb[1], point->rgb[2]);
        pointXYZ.x = point->pos(0);
        pointXYZ.y = point->pos(1);
        pointXYZ.z = point->pos(2);
        cloud->push_back(pointXYZ);
    }

    pcl::visualization::PCLVisualizer viewer("Viewer");
    viewer.setBackgroundColor(50, 50, 50);
    viewer.addPointCloud(cloud, "Triangulated Point Cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                            3,
                                            "Triangulated Point Cloud");
    viewer.addCoordinateSystem(1.0);


    int indexFrame = 0;
    for (auto &frame:frames) {
        Eigen::Matrix4f camPose;
        auto T_c_w = frame->Tcw.inverse().matrix();
        for (int i = 0; i < camPose.rows(); ++i)
            for (int j = 0; j < camPose.cols(); ++j)
                camPose(i, j) = T_c_w(i, j);
        viewer.addCoordinateSystem(1.0, Eigen::Affine3f(camPose), "cam" + to_string(indexFrame++));
    }
    viewer.initCameraParameters ();
    viewer.setCameraPosition(-12.0, -8.0, -6.0, 0.2, -1.0, 0.3, 0, -3.0, 0);
    while (!viewer.wasStopped ()) {

        viewer.spin();

    }

    // 获取pcl相机视角参数
    pcl::visualization::Camera pclCamera;
    viewer.getCameraParameters(pclCamera);
    printf("last pcl viewer camera pos:\n");
    printf("%lf, %lf, %lf,", pclCamera.pos[0], pclCamera.pos[1], pclCamera.pos[2]);
    printf("%lf, %lf, %lf,", pclCamera.view[0], pclCamera.view[1], pclCamera.view[2]);
    printf("%lf, %lf, %lf\n", pclCamera.focal[0], pclCamera.focal[1], pclCamera.focal[2]);  

}
#endif



