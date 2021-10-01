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
    const int NUM_OF_IMG = 4;
    std::string IMG_DIR("/home/lx6/MyWork/SLAM/learn_slam/FilesFromGithub/");
    // instrinc matrix
    cv::Mat K = (cv::Mat_<double>(3, 3) << 6.871898190000000e+02, 0, 3.750426640000000e+02,
                 0, 6.413762210000000e+02, 3.087127080000000e+02,
                 0, 0, 1);

    std::vector<std::vector<cv::Point2f>> key_matched(NUM_OF_IMG);
    for(int i=0;i<NUM_OF_IMG;i++)
    {
        mono::readMatches(IMG_DIR + "I"+std::to_string(i)+".txt", key_matched[i]);
    }
    
    // store the history
    std::vector<Eigen::Isometry3d,Eigen::aligned_allocator<Eigen::Isometry3d>> T_history;
    Eigen::Isometry3d SE3 = Eigen::Isometry3d::Identity(); // init by identity
    T_history.push_back(SE3);

    /**
     * @brief 
     * init using the epipolar 5-point
     * 
     */
    
    std::vector<cv::Point3f> pts_3D_init, pts_3D_current;
    Eigen::Matrix3d R = Eigen::Matrix3d::Zero(); // temporal
    Eigen::Vector3d t = Eigen::Vector3d(0, 0, 0); // temporal
    Eigen::Matrix3d K_intrinsic;
    cv::cv2eigen(K, K_intrinsic);
    // from 2d-2d
    myVO::findPos2d2d(cv::Size2d(800.0, 600.0), key_matched[0], key_matched[1], K_intrinsic, R, t);
    // find the 3d points (normalized scale)
    myVO::triangulate(key_matched[0], key_matched[1], pts_3D_init, K_intrinsic, R, t);
    SE3.prerotate(R.transpose());
    SE3.pretranslate(-R.transpose()*t);
    T_history.push_back(SE3);

    // update the world points to current camera's coordinate
    pts_3D_current = myVO::point3dLocal2local(pts_3D_init,R,t);
    /**
     * @brief 
     * Pnp
     * 
     */
    for(int i=2; i<NUM_OF_IMG;i++)
    {
        // current real 3d points with respect to the last frame
        myVO::findPos3d2d_DLT(pts_3D_current,key_matched[i],K_intrinsic,R,t);
        pts_3D_current=myVO::point3dLocal2local(pts_3D_current,R,t);

        // update the SE3 matrix
        SE3.prerotate(R.transpose());
        SE3.pretranslate(-R.transpose()*t);
        T_history.push_back(SE3);   
    }

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
        myPlot::refresh3DCVPoints(d_cam,s_cam,pts_3D_init,T_history);
    }
    return 0;
}
