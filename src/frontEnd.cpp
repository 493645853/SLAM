#include "mySLAM/frontEnd.h"
#include "mySLAM/feature2d.h"
#include "mySLAM/videoCapture.h"

namespace mySLAM
{
    FrontEnd::FrontEnd()
    {
        _orb = cv::ORB::create(num_maxFeatures);
        _matcher = cv::BFMatcher(cv::NORM_HAMMING);

        // init matrix
        N << 2. / VideoCapture::width, 0, -1,
            0, 2. / VideoCapture::height, -1,
            0, 0, 1;
    }

    void FrontEnd::step(Frame::Ptr frame)
    {
        _current_frame = frame;
        init();
    }

    bool FrontEnd::init()
    {
        if(detectFeature()>num_minFeatures)
        {
            //findPos2d2d();
            //std::cout << _relative_pos.matrix() << "\n";
        }
            
        return true;
    }

    int FrontEnd::detectFeature()
    {
        // detect key points & key descipter
        std::vector<cv::KeyPoint> prev_keypoints, curr_keypoints;
        cv::Mat prev_descptr, curr_descptr;
        std::vector<cv::DMatch> matches;
        _matches.clear();
        _current_frame->prev_feature.clear();
        _current_frame->curr_feature.clear();

        _orb->detect(_current_frame->prev_img, prev_keypoints);
        _orb->detect(_current_frame->curr_img, curr_keypoints);

        // compute descripter
        _orb->compute(_current_frame->prev_img, prev_keypoints, prev_descptr);
        _orb->compute(_current_frame->curr_img, curr_keypoints, curr_descptr);

        // match descripters
        if (curr_descptr.cols == prev_descptr.cols)
        {
            _matcher.match(prev_descptr,
                           curr_descptr, matches);
            // find good matches
            double min_dist = 1e4, max_dist = 0;
            // find min/max dist
            for (int i = 0; i < prev_descptr.rows; i++)
            {
                double dist = matches[i].distance;
                if (dist < min_dist)
                    min_dist = dist;
                if (dist > max_dist)
                    max_dist = dist;
            }

            for (int i = 0; i < prev_descptr.rows; i++)
            {
                if (matches[i].distance <= std::max(2 * min_dist, 30.0))
                {
                    _matches.push_back(matches[i]);
                }
            }

            // load the matched key points to the current frame object
            // show matches
            cv::Mat key_img;
            _current_frame->curr_img.copyTo(key_img);
            for (auto &m : _matches)
            {
                _current_frame->prev_feature.push_back(
                    Feature2d::Ptr(new Feature2d(prev_keypoints[m.queryIdx], _current_frame)));
                _current_frame->curr_feature.push_back(
                    Feature2d::Ptr(new Feature2d(curr_keypoints[m.trainIdx], _current_frame)));
                
                cv::circle(key_img, curr_keypoints[m.trainIdx].pt, 2, cv::Scalar(0, 255.0, 0), -1);
                cv::rectangle(key_img, curr_keypoints[m.trainIdx].pt - cv::Point2f(5,5), 
                              curr_keypoints[m.trainIdx].pt + cv::Point2f(5,5),
                              cv::Scalar(0,255,0));
  
            }

            _gui->updateKeyPtsImg(key_img);
        }

        return _matches.size();
    }

    bool FrontEnd::findPos2d2d()
    {
        Eigen::MatrixXd UV = Eigen::MatrixXd::Zero(_current_frame->prev_feature.size(), 9);
        Eigen::Vector3d x1, x2;
        for (int i = 0; i < UV.rows(); i++)
        {
            x1 = N * Eigen::Vector3d(_current_frame->prev_feature[i]->pts.pt.x,
                                     _current_frame->prev_feature[i]->pts.pt.y, 1);
            x2 = N * Eigen::Vector3d(_current_frame->curr_feature[i]->pts.pt.x,
                                     _current_frame->curr_feature[i]->pts.pt.y, 1);

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
        Eigen::MatrixXd E = VideoCapture::K.transpose() * F * VideoCapture::K;
        Eigen::JacobiSVD<Eigen::MatrixXd> svd_solver(E, Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::MatrixXd U = svd_solver.matrixU(),
                        V = svd_solver.matrixV(),
                        S = svd_solver.singularValues();
        S << 1, 1, 0;
        E = U * S.asDiagonal() * V.transpose();

        Eigen::Matrix3d Rz_90_deg, K_inv = VideoCapture::K.inverse();
        Rz_90_deg << 0, -1, 0,
            1, 0, 0,
            0, 0, 1;

        // projection matrix
        Eigen::Matrix<double, 3, 4> Proj_1;
        Proj_1 << Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero();
        Proj_1 = VideoCapture::K * Proj_1;

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

        double u1 = _current_frame->prev_feature[0]->pts.pt.x, v1 = _current_frame->prev_feature[0]->pts.pt.y,
               u2 = _current_frame->curr_feature[0]->pts.pt.x, v2 = _current_frame->curr_feature[0]->pts.pt.y;

        x1 = K_inv*Eigen::Vector3d(u1,v1,1);
        x2 = K_inv*Eigen::Vector3d(u2,v2,1);
        for (int i = 0; i < 4; i++)
        {
            _relative_pos.setRotationMatrix(Proj_2_all[i].leftCols(3));
            _relative_pos.translation() = Proj_2_all[i].col(3);
            Eigen::Matrix<double,3,2> A;
            A << -_relative_pos.rotationMatrix()*x1, x2;
            Eigen::Vector2d depth = A.colPivHouseholderQr().solve(_relative_pos.translation());
            if(depth(0)>0 && depth(1)>0)
            {
                break;
            }
        }
    }

}