#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include <Eigen/Cholesky>
#include <Eigen/Geometry>
#include <sophus/se3.hpp>
#include <g2o/core/g2o_core_api.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/dense/linear_solver_dense.h>

#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include "opencv2/imgcodecs/legacy/constants_c.h"

namespace myVO
{
    typedef Eigen::Ref<Eigen::MatrixXd> EigMatfunType;

    void feature_match(cv::Ptr<cv::ORB> &orb, cv::BFMatcher &matcher, cv::Mat &frame_1, cv::Mat &frame_2, cv::Mat &frame_match,
                       std::vector<cv::KeyPoint> &key_pt_1, std::vector<cv::KeyPoint> &key_pt_2, std::vector<cv::DMatch> &matches)
    {
        cv::Mat descptr_1, descptr_2;
        orb->detect(frame_1, key_pt_1);
        orb->detect(frame_2, key_pt_2);

        // compute descripter
        orb->compute(frame_1, key_pt_1, descptr_1);
        orb->compute(frame_2, key_pt_2, descptr_2);

        // match descripters
        if (descptr_1.cols == descptr_2.cols)
        {
            matcher.match(descptr_1, descptr_2, matches);

            // find good matches
            double min_dist = 1e4, max_dist = 0;
            // find min/max dist
            for (int i = 0; i < descptr_2.rows; i++)
            {
                double dist = matches[i].distance;
                if (dist < min_dist)
                    min_dist = dist;
                if (dist > max_dist)
                    max_dist = dist;
            }

            std::vector<cv::DMatch> good_matches;

            for (int i = 0; i < descptr_2.rows; i++)
            {
                if (matches[i].distance <= std::max(2 * min_dist, 50.0))
                {
                    good_matches.push_back(matches[i]);
                }
            }

            matches.assign(good_matches.begin(), good_matches.end());
            cv::drawMatches(frame_1, key_pt_1, frame_2, key_pt_2, good_matches, frame_match);
        }
    }
    
    /**
     * @brief Find the unique solution from all 4 possible solutions of the epipolar geometry
     * Z2 * K^(-1)p2 = R * Z1 * K^(-1) * p1 + t
     * and thus we have
     * [-R*x1 x2][Z1 Z2] = t,
     * solve above equation and then we have the depth
     * 
     * @param p1 matching points for the first graph
     * @param p2 matching points for the second graph
     * @param K camera's matrix input
     * @param E Essential matrix input
     * @param R rotation matrix output
     * @param t translation matrix output 
     */
    void findEipolarSolution(const cv::Point2f &p1, const cv::Point2f &p2,
                             EigMatfunType K, EigMatfunType E, EigMatfunType R, EigMatfunType t)
    {
        Eigen::Matrix3d Rz_90_deg, K_inv = K.inverse();
        Rz_90_deg << 0, -1, 0,
            1, 0, 0,
            0, 0, 1;

        // 4 possible solutions
        // find the only solution (based on the positive depth)
        Eigen::JacobiSVD<Eigen::MatrixXd> svd_solver(E, Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::MatrixXd U = svd_solver.matrixU(),
                        V = svd_solver.matrixV(),
                        S = svd_solver.singularValues();

        // projection matrix
        Eigen::Matrix<double, 3, 4> Proj_1;
        Proj_1 << Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero();
        Proj_1 = K * Proj_1;

        std::vector<Eigen::Matrix<double, 3, 4>, Eigen::aligned_allocator<Eigen::Matrix<double, 3, 4>>> Proj_2_all;
        Eigen::Matrix<double, 3, 4> Proj_2;
        Proj_2 << U * Rz_90_deg * V.transpose(), U.col(2);
        Proj_2_all.push_back(Proj_2);
        Proj_2 << U * Rz_90_deg * V.transpose(), -U.col(2);
        Proj_2_all.push_back(Proj_2);
        Proj_2 << U * Rz_90_deg.transpose() * V.transpose(), U.col(2);
        Proj_2_all.push_back(Proj_2);
        Proj_2 << U * Rz_90_deg.transpose() * V.transpose(), -U.col(2);
        Proj_2_all.push_back(Proj_2);

        double u1 = p1.x, v1 = p1.y,
               u2 = p2.x, v2 = p2.y;

        Eigen::Vector3d x1 = K_inv*Eigen::Vector3d(u1,v1,1),
                        x2 = K_inv*Eigen::Vector3d(u2,v2,1);
        for (int i = 0; i < 4; i++)
        {
            R = Proj_2_all[i].leftCols(3);
            t = Proj_2_all[i].col(3);
            Eigen::Matrix<double,3,2> A;
            A << -R*x1, x2;
            Eigen::Vector2d depth = A.colPivHouseholderQr().solve(t);
            if(depth(0)>0 && depth(1)>0)
            {
                break;
            }
        }
    }

    /**
     * @brief Triangulation to find the 3d point coordinates (normalized scale)
     * By solving these two equations:
     * p1^ * K*[I|0]*P_real = p1^ * p1 = 0
     * p2^ * K*[R|t]*P_real = p2^ * p2 = 0
     * Simplify above equation and find the LS solution using SVD
     * 
     * @param p1 matching points for the first graph
     * @param p2 matching points for the second graph
     * @param p_3d 3d point output
     * @param K camera's matrix input
     * @param R rotation matrix input
     * @param t translation matrix input
     */
    void triangulate(std::vector<cv::Point2f> &p1, std::vector<cv::Point2f> &p2, std::vector<cv::Point3f> &p_3d,
                     EigMatfunType K, EigMatfunType R, EigMatfunType t)
    {
        const int NUM_OF_MATCHES = p1.size();
        // projection matrix
        Eigen::Matrix<double, 3, 4> Proj_1;
        Proj_1 << Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero();
        Proj_1 = K * Proj_1;
        Eigen::Matrix<double, 3, 4> Proj_2;
        Proj_2 << R, t;
        Proj_2 = K * Proj_2;

        for (int i = 0; i < NUM_OF_MATCHES; i++)
        {
            double u1 = p1[i].x, v1 = p1[i].y,
                   u2 = p2[i].x, v2 = p2[i].y;
            Eigen::Matrix4d A;
            A.row(0) = u1 * Proj_1.row(2) - Proj_1.row(0);
            A.row(1) = v1 * Proj_1.row(2) - Proj_1.row(1);
            A.row(2) = u2 * Proj_2.row(2) - Proj_2.row(0);
            A.row(3) = v2 * Proj_2.row(2) - Proj_2.row(1);
            Eigen::JacobiSVD<Eigen::MatrixXd> svd_solver(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
            Eigen::Vector4d P_real = svd_solver.matrixV().col(3);
            P_real = P_real / P_real(3); // normalize
            p_3d.push_back(cv::Point3f(P_real(0), P_real(1), P_real(2)));
        }
    }

    /**
     * @brief Perform the least square method to find the epipolar geometry solution
     * s1*p1 = K*[I|0]*P_real
     * s2*p2 = K*[R|t]*P_real
     * where P_real is homogeneuous coordinate, if we use P_3d to represent the non-homogeneous point,
     * Then the equation becomes
     * s1*p1 = K*P_3d
     * s2*p2 = K*(R*P_3d+t)
     * Denote x1 = K^(-1)p1, x2 = K^(-1)p2, then
     * s1*x1 = P_3d
     * s2*x2 = R*P_3d + t
     * Thus
     * s2*x2 = R*s1*x1 + t
     * we left multiply the outer product matrix t^ on both sides
     * s2*t^*x2 = t^*R*s1*x1 + 0
     * and then left multiply the outer product matrix x2^T on both sides
     * s2*x2^T *t^*x2 = x2^T *t^*R*s1*x1 + 0 = 0
     * and thus we have the epipolar constraint
     * 
     * x2^T *t^*R*x1 = 0
     * or p2^T * K^(-T) * t^ * R * k^(-1) * p1 = 0
     * 
     * we define the "Essential matrix" E=t^ * R and the "fundamental matrix" F=K^(-T) * t^ * R * k^(-1)
     * For numeracal reasons, we solve for the F first.
     * p2^T * F * p1 = 0, can be simplified to A*F_vec = 0, which can be solved using SVD
     * Here, the pixel should be normalized firstly for the numerical stability, and after solving F, we can apply
     * F = N^T * F * N to denomalize F.
     * 
     * E = K^T * F * K
     * 
     * There exist 4 possible combinations of the rotation matrix & translation vector,
     * we can apply the triangulation to any pixel pairs to examine their depth, if both possitive,
     * we can find the final answer
     * 
     * @param Img_size size of the img
     * @param key_matched_1 matching points for the first graph
     * @param key_matched_2 matching points for the second graph
     * @param K camera's matrix input
     * @param R rotation matrix output
     * @param t translation matrix output
     */
    void findPos2d2d(const cv::Size2d &Img_size,
                     const std::vector<cv::Point2f> &key_matched_1,
                     const std::vector<cv::Point2f> &key_matched_2,
                     EigMatfunType K, EigMatfunType R, EigMatfunType t)
    {
        // build the epipolar matrix
        const int NUM_OF_MATCHES = key_matched_1.size();
        const double WIDTH = Img_size.width, HEIGHT = Img_size.height;

        R = Eigen::MatrixXd::Zero(3, 3);
        t = Eigen::Vector3d(0, 0, 0);
        Eigen::MatrixXd UV = Eigen::MatrixXd::Zero(NUM_OF_MATCHES, 9);

        Eigen::Matrix3d N;
        N << 2. / WIDTH, 0, -1,
            0, 2. / HEIGHT, -1,
            0, 0, 1;

        Eigen::Vector3d x1, x2;
        for (int i = 0; i < NUM_OF_MATCHES; i++)
        {
            // data centroid
            x1 = N * Eigen::Vector3d(key_matched_1[i].x, key_matched_1[i].y, 1);
            x2 = N * Eigen::Vector3d(key_matched_2[i].x, key_matched_2[i].y, 1);

            double u1 = x1(0),
                   v1 = x1(1),
                   u2 = x2(0),
                   v2 = x2(1);
            UV.row(i) << u1 * u2, u2 * v1, u2, v2 * u1, v2 * v1, v2, u1, v1, 1;
        }

        // SVD to find the null space
        Eigen::JacobiSVD<Eigen::MatrixXd> UV_svd_solver(UV, Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::MatrixXd UV_V = UV_svd_solver.matrixV();

        // Fundamental matrix (make rank 2)
        Eigen::Matrix3d F = Eigen::Matrix3d(UV_V.col(8).data()).transpose();
        Eigen::JacobiSVD<Eigen::MatrixXd> F_svd_solver(F, Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::MatrixXd F_U = F_svd_solver.matrixU(),
                        F_V = F_svd_solver.matrixV(),
                        F_S = F_svd_solver.singularValues();
        F_S << (F_S(0) + F_S(1)) / 2, (F_S(0) + F_S(1)) / 2, 0;
        F = N.transpose() * (F_U * F_S.asDiagonal() * F_V.transpose()) * N;

        // Eseential matrix (make rank 2)
        Eigen::MatrixXd E = K.transpose() * F * K;
        Eigen::JacobiSVD<Eigen::MatrixXd> svd_solver(E, Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::MatrixXd U = svd_solver.matrixU(),
                        V = svd_solver.matrixV(),
                        S = svd_solver.singularValues();
        S << 1, 1, 0;
        E = U * S.asDiagonal() * V.transpose();

        // Solve the translation & rotation
        findEipolarSolution(key_matched_1[0], key_matched_2[0], K, E, R, t);
    }

    /**
     * @brief Direct linear transform method to solve PnP
     * 
     * Since
     * s2*x2 = [R|t]*P_world_4d
     * we know x2 and P_world_4d by the initialization using epipolar 5 points method
     * we can solve for [R|t], following the methods from
     * https://zhuanlan.zhihu.com/p/58648937?edition=yidianzixun&yidian_docid=0LT3DZsH
     * 
     * @param Pos_3d_frame_1 
     * @param Pts_2d_frame_2 
     * @param K 
     * @param R 
     * @param t 
     */
    void findPos3d2d_DLT(const std::vector<cv::Point3f> &Pos_3d_frame_1,
                         const std::vector<cv::Point2f> &Pts_2d_frame_2,
                         EigMatfunType K, EigMatfunType R, EigMatfunType t)
    {
        const int NUM_OF_MATCHES = Pos_3d_frame_1.size();
        Eigen::MatrixXd A = Eigen::MatrixXd::Zero(2*NUM_OF_MATCHES,12);
        for(int i=0;i<NUM_OF_MATCHES;i++)
        {
            Eigen::Vector4d P_n(Pos_3d_frame_1[i].x,Pos_3d_frame_1[i].y,Pos_3d_frame_1[i].z,1);
            
            double u_n = (Pts_2d_frame_2[i].x - K(0,2))/K(0,0),
                   v_n = (Pts_2d_frame_2[i].y - K(1,2))/K(1,1);

            A.row(2*i) << P_n.transpose(), Eigen::Vector4d::Zero().transpose(), -u_n * P_n.transpose();
            A.row(2*i+1) << Eigen::Vector4d::Zero().transpose(), P_n.transpose(), -v_n * P_n.transpose();
        }

        Eigen::JacobiSVD<Eigen::MatrixXd> svd_solver(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::Matrix<double,3,4> T = (Eigen::Matrix<double,4,3>(svd_solver.matrixV().col(11).data())).transpose();
        R = T.leftCols(3);
        t = T.col(3);
        

        // find the best R
        Eigen::JacobiSVD<Eigen::MatrixXd> R_svd_solver(R,Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::Vector3d R_S = R_svd_solver.singularValues();
        R = R_svd_solver.matrixU()*R_svd_solver.matrixV().transpose();

        // find the scaled t
        double scale = 1.0/(R_S.sum()/3);
        t = scale * t;

        // determine + -
        int num_of_positive = 0,
            num_of_negative = 0;
        for(int i=0;i<NUM_OF_MATCHES;i++)
        {
            Eigen::Vector3d P_n(Pos_3d_frame_1[i].x,Pos_3d_frame_1[i].y,Pos_3d_frame_1[i].z);
            ((R.row(2)*P_n + t(2,0)) >= 0)?(num_of_positive++):(num_of_negative++);
        }
        if(num_of_positive<num_of_negative)
        {
            R = -R;
            t = -t;
        }
    }

    /**
     * @brief Gaussian-Newton method to solve PnP problem (recover R, t from 3D to 2D)
     * 
     * @param Pos_3d_frame_1 
     * @param Pts_2d_frame_2 
     * @param K 
     * @param R 
     * @param t 
     * @param maxIterations
     */
    void findPos3d2d_GN(const std::vector<cv::Point3f> &Pos_3d_frame_1,
                        const std::vector<cv::Point2f> &Pts_2d_frame_2,
                        EigMatfunType K, EigMatfunType R, EigMatfunType t,
                        int maxIterations = 100,
                        double errorThreshold = 1e-6)
    {
        typedef Eigen::Matrix<double,6,1> Vector6d; 
        double cost = 0, lastCost = 0;
        double fx = K(0,0), fy = K(1,1), cx = K(0,2), cy = K(1,2);

        Eigen::Matrix<double,6,6> H = Eigen::Matrix<double,6,6>::Zero();
        Eigen::Matrix<double,2,6> J = Eigen::Matrix<double,2,6>::Zero();
        Vector6d b = Vector6d::Zero();
        Sophus::SE3d pos_se3; // estimation

        //main iteration loop
        for(int i=0;i<maxIterations;i++)
        {
            H.fill(0);
            b.fill(0);
            cost = 0;

            // find cost
            for(int j=0; j<Pos_3d_frame_1.size();j++)
            {
                Eigen::Vector3d pos_rotTranslate = pos_se3 * Eigen::Vector3d(Pos_3d_frame_1[j].x,
                                                                             Pos_3d_frame_1[j].y,
                                                                             Pos_3d_frame_1[j].z);
                double inv_z = 1./pos_rotTranslate(2);
                pos_rotTranslate = pos_rotTranslate*inv_z; // norm the z axis

                Eigen::Vector2d proj(fx*pos_rotTranslate(0)+cx, 
                                     fy*pos_rotTranslate(1)+cy);

                Eigen::Vector2d error = Eigen::Vector2d(Pts_2d_frame_2[j].x, Pts_2d_frame_2[j].y) - proj;
                cost += error.squaredNorm();

                // Jacobian matrix
                J << -fx*inv_z, 0, fx*pos_rotTranslate(0)*inv_z, fx*pos_rotTranslate(0)*pos_rotTranslate(1), -fx-fx*pos_rotTranslate(0)*pos_rotTranslate(0), fx*pos_rotTranslate(1),
                     0, -fy*inv_z, fy*pos_rotTranslate(1)*inv_z, fy+fy*pos_rotTranslate(1)*pos_rotTranslate(1), -fy*pos_rotTranslate(0)*pos_rotTranslate(1), -fy*pos_rotTranslate(0);
                H += J.transpose()*J;
                b += -J.transpose()*error;
            }

            // update incremental dx
            Vector6d dx = H.ldlt().solve(b);

            // check if nan
            if(std::isnan(dx(0)))
            {
                std::cout<<"result is nan"<< "\n";
                break;
            }

            if(i > 0 && cost>=lastCost)
            {
                //std::cout << "cost: " << cost << ", last cost: " << lastCost << "\n";
                break;
            }

            // update estimation (exp mapping) left disturbance model
            pos_se3 = Sophus::SE3d::exp(dx) * pos_se3;
            lastCost = cost;

            // if the algorithm converges, break
            if(dx.norm() < errorThreshold)
            {
                //std::cout<<"converge"<<"\n";
                break;
            }
        }

        R = pos_se3.rotationMatrix();
        t = pos_se3.translation();
        //std::cout << "Final Cost: " << lastCost <<"\t pose by g−n: \n" << pos_se3.matrix() << std::endl;

    }


    /**
     * @brief Define the vertex and the edge for g2o optimization
     * 
     */
    namespace g2oBA
    {   
        /**
         * @brief SE3 Vertex for the g2o optimization
         * 
         */
        class VertexSE3:public g2o::BaseVertex<6, Sophus::SE3d>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

            // how the SE3 initialize
            virtual void setToOriginImpl() override
            {
                _estimate = Sophus::SE3d();
            }

            // how the SE3 update (left distub model)
            virtual void oplusImpl(const double *update) override
            {
                Eigen::Matrix<double,6,1> tangSpaceVec;
                tangSpaceVec << update[0], update[1], update[2], update[3], update[4], update[5];
                _estimate = Sophus::SE3d::exp(tangSpaceVec) * _estimate;
            }

            virtual bool read(std::istream &in) override {return true;}
            virtual bool write(std::ostream &out) const override {return true;}
        };

        /**
         * @brief error edge for the g2o optimization
         * 
         */
        class EdgeProjError:public g2o::BaseUnaryEdge<2, Eigen::Vector2d, VertexSE3>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

            // constructor
            EdgeProjError(const EigMatfunType& pos3d, const EigMatfunType& K):_pos3d(pos3d),_K(K){}

            // how to compute the error
            virtual void computeError() override
            {
                const VertexSE3 *v = static_cast<const VertexSE3*>(this->_vertices[0]);
                Sophus::SE3d T = v->estimate();
                Eigen::Vector3d pts2d_estimate = _K * (T * _pos3d);
                pts2d_estimate = pts2d_estimate / pts2d_estimate(2);
                _error = _measurement - pts2d_estimate.head<2>();
            }

            // how to find the jacobian
            virtual void linearizeOplus() override
            {
                const VertexSE3 *v = static_cast<const VertexSE3*>(this->_vertices[0]);
                Sophus::SE3d T = v->estimate();
                Eigen::Vector3d pos_rotTranslate = T * _pos3d;
                double inv_z = 1./pos_rotTranslate(2);
                double fx = _K(0, 0), fy = _K(1, 1), cx = _K(0, 2), cy = _K(1, 2);
                _jacobianOplusXi << -fx * inv_z, 0, fx * pos_rotTranslate(0) * inv_z, fx * pos_rotTranslate(0) * pos_rotTranslate(1), -fx - fx * pos_rotTranslate(0) * pos_rotTranslate(0), fx * pos_rotTranslate(1),
                                    0, -fy * inv_z, fy * pos_rotTranslate(1) * inv_z, fy + fy * pos_rotTranslate(1) * pos_rotTranslate(1), -fy * pos_rotTranslate(0) * pos_rotTranslate(1), -fy * pos_rotTranslate(0);
            }

            virtual bool read(std::istream &in) override {return true;}
            virtual bool write(std::ostream &out) const override {return true;}

        private:
            Eigen::Vector3d _pos3d;
            Eigen::Matrix3d _K; // camera matrix  
        };
    }

    /**
     * @brief G2O Gaussian-Newton method to solve PnP problem (recover R, t from 3D to 2D)
     * 
     * @param Pos_3d_frame_1 
     * @param Pts_2d_frame_2 
     * @param K 
     * @param R 
     * @param t 
     * @param maxIterations
     */
    class PnP_g2o
    {
    public:
        
        // pos estimate is 6 (lie algebra) 
        typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>> BlockSolverType;
        typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType; //线性求解器类型

        // constructor
        PnP_g2o(const EigMatfunType& K):_K(K),blockSolver(std::make_unique<BlockSolverType>(std::make_unique<LinearSolverType>()))
        {
            LS_algorithm = new g2o::OptimizationAlgorithmGaussNewton(std::move(blockSolver));
        }

        void run(const std::vector<cv::Point3f> &Pos_3d_frame_1,
                 const std::vector<cv::Point2f> &Pts_2d_frame_2,
                 EigMatfunType R, EigMatfunType t,
                 int maxIterations = 50)
        {
            // 添加顶点和边
            g2oBA::VertexSE3 *v = new g2oBA::VertexSE3();
            v->setEstimate(Sophus::SE3d());
            v->setId(1); // vertex id

            // init graph optimizer
            g2o::SparseOptimizer *optimizer; // 图模型
            optimizer = new g2o::SparseOptimizer();
            optimizer->setAlgorithm(LS_algorithm);
            optimizer->setVerbose(false);
            optimizer->addVertex(v);

            // edge
            for(int i=0;i<Pos_3d_frame_1.size();i++)
            {
                Eigen::Vector3d pos3d = Eigen::Vector3d(Pos_3d_frame_1[i].x,Pos_3d_frame_1[i].y,Pos_3d_frame_1[i].z);
                g2oBA::EdgeProjError *edge = new g2oBA::EdgeProjError(pos3d,_K);
                edge->setId(i);
                edge->setVertex(0, v);
                edge->setMeasurement(Eigen::Vector2d(Pts_2d_frame_2[i].x, Pts_2d_frame_2[i].y));
                edge->setInformation(Eigen::Matrix2d::Identity());

                // add edge
                optimizer->addEdge(edge);
            }

            // optimize
            optimizer->initializeOptimization();
            optimizer->optimize(maxIterations);

            // update
            Sophus::SE3d pos_se3 = v->estimate();
            R = pos_se3.rotationMatrix();
            t = pos_se3.translation();

            // delete pointer
            delete v;
        }

    private:
        std::unique_ptr<BlockSolverType> blockSolver;
        g2o::OptimizationAlgorithmGaussNewton *LS_algorithm;
        Eigen::Matrix3d _K;
    };

    /**
     * @brief Transform the points for coordinate 1 to the points for coordinate 2
     * 
     * @param Pos_3d_local_1 
     * @param R 
     * @param t 
     * @return std::vector<cv::Point3f> 
     */
    std::vector<cv::Point3f> point3dLocal2local(const std::vector<cv::Point3f> &Pos_3d_local_1,
                              EigMatfunType R, EigMatfunType t)
    {
        std::vector<cv::Point3f> Pos_3d_local_2;
        for(auto& pts:Pos_3d_local_1)
        {
            Eigen::Vector3d P_local_2_eigen = R*Eigen::Vector3d(pts.x,pts.y,pts.z)+t;
            Pos_3d_local_2.push_back(cv::Point3f(P_local_2_eigen(0),P_local_2_eigen(1),P_local_2_eigen(2)));
        }
        return Pos_3d_local_2;
    }


}