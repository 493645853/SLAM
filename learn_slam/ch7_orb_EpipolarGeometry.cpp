#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include <Eigen/Cholesky>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include "opencv2/imgcodecs/legacy/constants_c.h"
#include <pangolin/pangolin.h>
#include <chrono>
#include <cmath>

#include "monoCalibration.hpp"
#include "visualOdometry.hpp"
#include "slamPlot.hpp"

int main()
{
    // optimize the speed of the iostream
    std::ios::sync_with_stdio(false);
    std::cin.tie(nullptr);
    std::cout.tie(nullptr);
    std::setvbuf(stdout, nullptr, _IOFBF, BUFSIZ);

    // read the matches
    std::string IMG_DIR("/home/lx6/MyWork/SLAM/learn_slam/FilesFromGithub/");
    // instrinc matrix
    cv::Mat K = (cv::Mat_<double>(3, 3) << 6.871898190000000e+02, 0, 3.750426640000000e+02,
                 0, 6.413762210000000e+02, 3.087127080000000e+02,
                 0, 0, 1);

    std::vector<cv::Point2f> key_matched_1, key_matched_2;
    mono::readMatches(IMG_DIR + "I0.txt", key_matched_1);
    mono::readMatches(IMG_DIR + "I2.txt", key_matched_2);

    //Epipolar geometry
    std::vector<cv::Point3f> pts_3D;
    Eigen::Matrix3d R = Eigen::Matrix3d::Zero();
    Eigen::Vector3d t = Eigen::Vector3d(0, 0, 0);
    Eigen::Matrix3d K_intrinsic;
    cv::cv2eigen(K, K_intrinsic);
    auto t1 = std::chrono::steady_clock::now();
    // from 2d-2d
    myVO::findPos2d2d(cv::Size2d(800.0, 600.0), key_matched_1, key_matched_2, K_intrinsic, R, t);
    // find the 3d points (normalized scale)
    myVO::triangulate(key_matched_1, key_matched_2, pts_3D, K_intrinsic, R, t);
    auto t2 = std::chrono::steady_clock::now();
    auto t_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    std::cout << "time cost: " << t_used.count() << "\n";

    std::cout << "R is " << std::endl
              << R << std::endl;
    std::cout << "t is " << std::endl
              << t << std::endl;

    // store the history
    std::vector<Eigen::Isometry3d,Eigen::aligned_allocator<Eigen::Isometry3d>> T_history;
    Eigen::Isometry3d SE3 = Eigen::Isometry3d::Identity();
    T_history.push_back(SE3);
    SE3.prerotate(R.transpose());
    SE3.pretranslate(-t);
    T_history.push_back(SE3);


    // visualize the results
    pangolin::CreateWindowAndBind("3d points", 800, 600);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
        pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0));

    pangolin::View &d_cam = pangolin::CreateDisplay()
                                .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f)
                                .SetHandler(new pangolin::Handler3D(s_cam));
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    while (pangolin::ShouldQuit() == false)
    {
        myPlot::refresh3DCVPoints(d_cam,s_cam,pts_3D,T_history);
    }
    return 0;
}
