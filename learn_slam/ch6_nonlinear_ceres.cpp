#include <iostream>
#include <opencv2/core/core.hpp>
#include <ceres/ceres.h>
#include <chrono>

struct CURVE_FITTING_COST
{
    // constructor
    CURVE_FITTING_COST(const double& x, const double& y):x_data(x),y_data(y){}

    // function (const T* means to create a pointer pointing to a const object, T* const means the pointer
    // itself is const)
    template<typename T>
    bool operator()(const T *const abc, T* residue)const
    {
        *residue = T(y_data) - exp(abc[0]*T(x_data)*T(x_data) + abc[1]*T(x_data) + abc[2]);
        return true;
    }
    // data
    const double x_data,y_data;
};

int main ( int argc, char** argv )
{
    /**
     * @brief 
     * Generate data
     * 
     */
    double a=1.0, b=2.0, c=1.0;         // 真实参数值
    int N=100;                          // 数据点
    double w_sigma=1.0;                 // 噪声Sigma值
    cv::RNG rng;                        // OpenCV随机数产生器
    double abc[3] = {0,0,0};            // abc参数的估计值

    std::vector<double> x_data, y_data;      // 数据

    std::cout<<"generating data: "<<std::endl;
    for ( int i=0; i<N; i++ )
    {
        double x = i/100.0;
        x_data.push_back ( x );
        y_data.push_back (
            exp ( a*x*x + b*x + c ) + rng.gaussian ( w_sigma )
        );
        std::cout<<x_data[i]<<" "<<y_data[i]<<std::endl;
    }

    /**
     * @brief 
     * create ceres problem
     */
    ceres::Problem LS_prob;
    for (int i = 0; i < N; i++)
    {
        LS_prob.AddResidualBlock(new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 3>(
                                     new CURVE_FITTING_COST(x_data[i], y_data[i])),
                                 nullptr,
                                 abc);
    }

    /**
     * @brief 
     * run the solver
     * 
     */
    ceres::Solver::Options options; // 配置参数
    options.linear_solver_type = ceres::DENSE_QR; // QR分解求矩阵逆
    options.minimizer_progress_to_stdout = true; // 结果输出到cout

    ceres::Solver::Summary summary; // 输出结果总结类

    // 计时并运行
    auto t1 = std::chrono::steady_clock::now();
    ceres::Solve(options,&LS_prob,&summary); // 计算
    auto t2 = std::chrono::steady_clock::now();
    auto time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2-t1);

    // 输出结果
    std::cout<<"Time Cost: "<< time_used.count()<<"\n";
    std::cout<<summary.BriefReport()<<"\n";
    std::cout<<"Final Optimization Results: "<< abc[0] << " "<<abc[1]<<" "<<abc[2]<<std::endl;


    return 0;
}
