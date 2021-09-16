#include<iostream>
#include<Eigen/Dense>
#include<Eigen/Core>
#include<Eigen/Geometry>
#include<opencv2/opencv.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/core/eigen.hpp>
#include<string>
#include<vector>
#include"stereoCalibration.hpp"

int main(void)
{
    std::string cali_DIR="D:/SelfLearning_vscode/MyProject/learn_slam/Calibration/";
    cv::Mat camearaMat,distCoeffs,newCameraMat;
    stereo::load_caliParameters(cali_DIR,camearaMat,distCoeffs,newCameraMat);

    // convert to eigen matrix
    Eigen::MatrixXd K_origin,dist,K_new;
    cv::cv2eigen(camearaMat,K_origin);
    cv::cv2eigen(distCoeffs,dist);
    cv::cv2eigen(newCameraMat,K_new);

    std::cout << "\n cameraMatrix : \n" << K_origin.format(Eigen::IOFormat(Eigen::FullPrecision)) << std::endl;
    std::cout << "\n New cameraMatrix : \n" << K_new.format(Eigen::IOFormat(Eigen::FullPrecision)) << std::endl;
    std::cout << "\n distCoeffs : \n" << dist.format(Eigen::IOFormat(Eigen::FullPrecision)) << std::endl;


    // test the model [u,v,1]^T = 1/Z*K*[R|t]*[X_w,Y_w,Z_w,1]^T
    Eigen::Vector4d P_w(0.1,0.1,500,1);
    Eigen::Isometry3d T(Eigen::Quaterniond(0.35,0.2,0.3,0.1));
    T.pretranslate(Eigen::Vector3d(-0.05,0.01,0.1));


    Eigen::Vector3d P_pix =  K_origin * (( P_w).head<3>());
    P_pix = P_pix/P_pix(2);
    std::cout<<P_pix<<std::endl;

    return 0;
}