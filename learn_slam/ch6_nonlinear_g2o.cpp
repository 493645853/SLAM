#include <iostream>
#include <g2o/core/g2o_core_api.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <cmath>
#include <chrono>

/**
 * @brief 
 * 图优化中的顶点 (abc), dim = 3
 */
class CurveFittingVertex: public g2o::BaseVertex<3,Eigen::Vector3d>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  /**
   * @brief Set the To Origin Impl object
   * 重写虚函数: 重置变量方式
   */
  virtual void setToOriginImpl() override
  {
    this->_estimate << 0,0,0;
  }

  /**
   * @brief 
   * 重写虚函数: 变量相加的方式
   * @param update 
   */
  virtual void oplusImpl(const double *update) override
  {
    this->_estimate += Eigen::Vector3d(update);
  }

  virtual bool read(std::istream &in){return true;};
  virtual bool write(std::ostream &out)const {return true;};
  
};

/**
 * @brief 
 * 误差边缘函数: 观测值维度=1, 类型为double, 顶点类型为如上定义的类
 */
class CurveFittingEdge: public g2o::BaseUnaryEdge<1,double,CurveFittingVertex>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  CurveFittingEdge(const double& x):BaseUnaryEdge(),x_data(x){}

  // 重写误差计算
  virtual void computeError() override
  {
    const CurveFittingVertex *v = static_cast<const CurveFittingVertex*>(this->_vertices[0]);
    const Eigen::Vector3d abc = v->estimate();
    this->_error(0,0) = this->_measurement - exp(abc[0]*this->x_data*this->x_data + abc[1]*this->x_data + abc[2]);
  }

  // 重写Jacob
  virtual void linearizeOplus() override
  {
    const CurveFittingVertex *v = static_cast<const CurveFittingVertex*>(this->_vertices[0]);
    const Eigen::Vector3d abc = v->estimate();
    double fx = exp(abc[0]*this->x_data*this->x_data + abc[1]*this->x_data + abc[2]);
    this->_jacobianOplusXi[0] = -this->x_data*this->x_data*fx;
    this->_jacobianOplusXi[1] = -this->x_data*fx;
    this->_jacobianOplusXi[2] = -fx;
  }

  virtual bool read(std::istream &in){return true;};
  virtual bool write(std::ostream &out)const {return true;};

public:
  double x_data;
};

int main(int argc, char **argv)
{
  double ar = 1.0, br = 2.0, cr = 1.0;  // 真实参数值
  double ae = 2.0, be = -1.0, ce = 5.0; // 估计参数值
  int N = 100;                          // 数据点
  double w_sigma = 1.0;                 // 噪声Sigma值
  double inv_sigma = 1.0 / w_sigma;
  cv::RNG rng; // OpenCV随机数产生器

  std::vector<double> x_data, y_data; // 数据
  for (int i = 0; i < N; i++)
  {
    double x = i / 100.0;
    x_data.push_back(x);
    y_data.push_back(exp(ar * x * x + br * x + cr) + rng.gaussian(w_sigma * w_sigma));
  }

  // 求解器别名: 3待解量, 1输出量
  using g2oBlockerSolver3i1o =  g2o::BlockSolver<g2o::BlockSolverTraits<3,1>>;
  // 线性方程求解设置: chol
  using g2oLineaerSolverType = g2o::LinearSolverDense<Eigen::Matrix3d>;

  /**
   * @brief 
   * create solver
   */
  // unique ptr
  std::unique_ptr<g2oBlockerSolver3i1o> blockSolver = std::make_unique<g2oBlockerSolver3i1o>(std::make_unique<g2oLineaerSolverType>());
  // unique ptr不能被复制或者传参, 只能被移动std::move (保证唯一性)
  auto *LS_algorithm = new g2o::OptimizationAlgorithmGaussNewton(std::move(blockSolver));

  g2o::SparseOptimizer optimizer; // 图模型?
  optimizer.setAlgorithm(LS_algorithm);
  optimizer.setVerbose(true); // 打开调试输出

  // 添加顶点和边
  CurveFittingVertex *v = new CurveFittingVertex();
  v->setEstimate(Eigen::Vector3d(ae,be,ce));
  v->setId(0); // vertex id
  optimizer.addVertex(v);

  // 边
  for(int i=0;i<N;i++)
  {
    CurveFittingEdge *edge = new CurveFittingEdge(x_data[i]);
    edge->setId(i);
    edge->setVertex(0,v); // 设置与之连接的顶点
    edge->setMeasurement(y_data[i]); // 观测值
    edge->setInformation(Eigen::Matrix<double,1,1>::Identity()*1/(w_sigma*w_sigma)); // 信息矩阵: 协方差矩阵的逆
    optimizer.addEdge(edge);
  }

  /**
   * @brief 
   * 开始优化
   * 
   */
  std::cout<<"Begin optimization"<<std::endl;
  auto t1 = std::chrono::steady_clock::now();
  optimizer.initializeOptimization();
  optimizer.optimize(10);
  auto t2 = std::chrono::steady_clock::now();
  auto t_used = std::chrono::duration_cast<std::chrono::duration<double>> (t2-t1);
  std::cout<<"Total time cost: "<<t_used.count()<<"\n";

  // 输出优化值
  std::cout<<"Optimization results: "<<v->estimate().transpose()<<std::endl;

  return 0;
}