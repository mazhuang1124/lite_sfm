#include <iostream>
#include "common_include.h"
#include "SFM.h"

#include <boost/algorithm/string.hpp>
#include <boost/range/adaptor/transformed.hpp>
#include <boost/filesystem.hpp>

#ifdef PCL_VIEWER_DEBUG
#include <X11/Xlib.h>   // 规避PCL可视化偶尔失败的情况
#endif

namespace fs = boost::filesystem;

// 遍历路径下图像
void getImageFiles(const string& imageDir, vector<string>& imageFiles) {
    if (!imageDir.empty()) {
        fs::path dirPath(imageDir);
        
        if (!fs::exists(dirPath) || !fs::is_directory(dirPath)) {
            cerr << "Invalid directory path: " << imageDir << endl;
            return;
        }
        
        for (const auto& itr : fs::directory_iterator(dirPath)) {
            string extension = itr.path().extension().string();
            boost::algorithm::to_lower(extension);
            if (fs::is_regular_file(itr.status())) {
                if (extension == ".jpg" || extension == ".png") {
                    imageFiles.push_back(itr.path().string());
                }
            }
        }
        
        if (imageFiles.size() == 0) {
            cerr << imageDir << " images nums is 0!" << endl;
            return;
        }
        
        sort(imageFiles.begin(), imageFiles.end());

    }
}


void getK(const string& txtK, Eigen::Matrix3d &K)
{
    std::ifstream file(txtK);
    if (!file.is_open())
    {
        std::cerr << "Failed to open file." << std::endl;
        return;
    }

    // 读取参数
    file >> K(0, 0) >> K(0, 1) >> K(0, 2)
         >> K(1, 0) >> K(1, 1) >> K(1, 2)
         >> K(2, 0) >> K(2, 1) >> K(2, 2);
}


int main() {

#ifdef PCL_VIEWER_DEBUG
    XInitThreads(); // 仍存在窗口启动失败的情况
#endif

    SFM sfm(cv::SIFT::create(0, 3, 0.04, 10), cv::DescriptorMatcher::create("BruteForce"));

    string imageDir = "../datasets/fountain_dense_images";
    // string txtK = imageDir + "/K.txt";

    cout << "Images Path: " + imageDir << endl;


    vector<string> imageFiles;
    getImageFiles(imageDir, imageFiles);
    cout << "images nums: "<< imageFiles.size() << endl;

    // cout << "K.txt Path: " + txtK << endl;
    // Eigen::Matrix3d K;
    // getK(txtK, K);
    // cout << "K = \n" << K << endl;
    // Camera::Ptr camera(new Camera(K(0,0), K(1,1), K(0,2), K(1,2)));

    Camera::Ptr camera(new Camera(2759.48, 2764.16, 1520.69, 1006.81));
    // Camera::Ptr camera = std::make_shared<Camera>(2759.48, 2764.16, 1520.69, 1006.81);

    sfm.pipeline(imageFiles, camera);

    return 0;
}