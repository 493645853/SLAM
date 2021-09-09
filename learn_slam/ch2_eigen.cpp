#include<iostream>
#include<Eigen/Eigen>
#include<Eigen/Core>
#include<Eigen/Geometry>
#include<cmath>
/**
 * @brief SLAM 14讲 第二章
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
    
    return 0;
}