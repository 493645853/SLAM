#include<iostream>
#include<Eigen/Eigen>
#include<Eigen/Core>
#include<Eigen/Geometry>
#include<cmath>
/**
 * @brief SLAM 14讲 第三章
 * 
 * 复习eigen库有关知识: 向量变换
 * 
 * @return int 
 */

int main()
{
    // create a matrix
    Eigen::MatrixXd mat = Eigen::MatrixXd::Identity(5,5);
    Eigen::MatrixXd covMat = mat*mat.transpose();

    // Eigendecomposition
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen_solver(covMat);
    
    //std::cout.precision(3);
    std::cout<<"Covariance Matrix = \n"<<covMat<<std::endl;
    std::cout<<"Eigenvalues = \n"<<eigen_solver.eigenvalues()<<std::endl;
    std::cout<<"Eigenvectors = \n"<<eigen_solver.eigenvectors()<<std::endl;

    /**
     * @brief 
     * The code below is about the Eigen/Geometryw
     * 
     */
    // For the rotation matrix, we directly apply the matrix3d (or matrixXd)
    Eigen::Matrix3d rotation_mat = Eigen::Matrix3d::Identity(3,3);
    // Angle axis rotation
    Eigen::AngleAxisd rot_vec(EIGEN_PI/4, Eigen::Vector3d(0,0,1));
    // Euler angles (from rotation matrix)
    Eigen::Vector3d eulerAngles = rotation_mat.eulerAngles(2,1,0); // ZYX order


    // To rotation matrix
    std::cout<<"From axis angle to the rotation matrix\n"<<rot_vec.toRotationMatrix()<<std::endl;
    std::cout<<"From the rotation matrix to the Euler angles\n"<<eulerAngles.transpose()<<std::endl;

    // Euclidean transform matrix
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity(); // 4x4 matrix
    T.rotate(rot_vec);
    T.pretranslate(Eigen::Vector3d(1,3,4));// pretranslate = 提前位移, 坐标系依然是世界坐标系
    std::cout<<"Euclidean tranformation matrix\n"<<T.matrix()<<std::endl;

    //Quaternions
    Eigen::Quaterniond q1(1,0,0,0); // 直接赋值
    std::cout<<"Quaternion from the direct values\n"<<q1.coeffs()<<std::endl;
    Eigen::Quaterniond q2(rot_vec); // 从angleAxis赋值
    std::cout<<"Quaternion from the angleAxis\n"<<q2.coeffs()<<std::endl;

    //rotate other vector
    std::cout<<"(1,0,0) after rotation = "<<(q2*Eigen::Vector3d(1,0,0)).transpose()<<std::endl;

    return 0;
}