#include<iostream>
#include<opencv2/opencv.hpp>
#include<Eigen/Core>
#include<Eigen/Eigen>
#include<Eigen/LU>
#include<Eigen/Dense>
#include<chrono>

/**
 * @brief 
 * 手写高斯牛顿法
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char **argv)
{
    // optimize the speed of the iostream
    std::ios::sync_with_stdio(false);
    std::cin.tie(nullptr);
    std::cout.tie(nullptr);
    std::setvbuf(stdout, nullptr, _IOFBF, BUFSIZ);

    double a = 1.0, b = 2.0, c = 1.0; // real parameters
    int N = 100; // number of simulation points

    double w_sigma = 1.0; //noise std
    double inv_sigma = 1.0/w_sigma;
    cv::RNG randgen; //random generator

    /**
     * @brief 
     * generate data
     */
    Eigen::MatrixXd x_data=Eigen::MatrixXd::Zero(1,N);
    Eigen::MatrixXd y_data=Eigen::MatrixXd::Zero(1,N);
    for(int i=0;i<N;i++)
    {
        double x = i/100.0;
        x_data(0,i) = x;
        y_data(0,i) = exp(a*x*x + b*x + c)+randgen.gaussian(w_sigma*w_sigma);
    } 

    /**
     * @brief 
     * Gaussian-Newton method
     */
    int iteration = 100;
    double cost = 0.0, lastCost = 0.0;
    Eigen::Vector3d est(2.0,-1.0,5.0);  // init estimates;
    Eigen::Vector3d dest;
    Eigen::Matrix3d H;
    Eigen::Vector3d g;
    Eigen::MatrixXd fx=Eigen::MatrixXd::Zero(1,N), residue=Eigen::MatrixXd::Zero(1,N);
    Eigen::MatrixXd Jacob = Eigen::MatrixXd::Zero(3,N);
    Eigen::MatrixXd x_data_abs2 = x_data.array().square().matrix();

    auto t1 = std::chrono::steady_clock::now();
    for(int iter=0;iter<iteration;iter++)
    {
        fx = Eigen::exp((est(0)*x_data_abs2 + est(1)*x_data + est(2)*Eigen::MatrixXd::Ones(1,N)).array());
        residue = y_data - fx;
        cost = residue.array().square().sum();
        Jacob.row(0) = -x_data_abs2.cwiseProduct(fx);
        Jacob.row(1) = -x_data.cwiseProduct(fx);
        Jacob.row(2) = -fx;

        // solve the Gaussian-Newton equation
        H = Jacob*Jacob.transpose();
        g = Jacob.cwiseProduct(-Eigen::MatrixXd::Ones(3,1)*residue).rowwise().sum(); // equivalent to sum(x,2) in matlab
        dest = H.ldlt().solve(g);

        if(std::isnan(dest[0]))
        {
            std::cout<<"result is nan"<<"\n";
            break;
        }
        if(iter>0 && cost>= lastCost)
        {
            std::cout<<"iter"<<iter<<" cost: "<<cost<<">= last cost: "<<lastCost<<", break"<<"\n";
            break;
        }

        // update
        est += dest;
        lastCost = cost;
        std::cout <<"Iteration: "<<iter<< " est: " << est.transpose()<<"\n";
    }
    auto t2 = std::chrono::steady_clock::now();
    auto time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    std::cout << "solve time cost = " << time_used.count() << " seconds. " << std::endl;
    std::cout << "estimated abc = " << est.transpose()<< std::endl;
    return 0;
}