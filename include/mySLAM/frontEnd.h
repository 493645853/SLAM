#ifndef __FRONTEND_H__
#define __FRONTEND_H__

/**
 * @file frontEnd.h
 * @author xin li
 * @brief front end of the slam system
 * @version 0.1
 * @date 2021-10-03
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <thread>

#include "common_include.h"
#include "map.h"
#include "imu.h"
#include "gui.h"
#include "frame.h"
#include "videoCapture.h"

namespace mySLAM
{
    class GUI;
    class Map;

    enum class FrontEndStatus{INITING,TRACKING,LOST};
    /**
     * @brief real-time tracking of the orb feature points,
     * and estimating camera extrinsics, triangulate the feature points
     * 
     */
    class FrontEnd
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<FrontEnd> Ptr;
        FrontEnd();

        // get status
        FrontEndStatus status() {return _status;}
        
        // run the frontEnd
        void step(Frame::Ptr frame);

        // link to gui for info update
        void linkToGUI(std::shared_ptr<GUI> ptr) { _gui = ptr; }
        void linkToMap(std::shared_ptr<Map> ptr) { _map = ptr; }

    private:
        FrontEndStatus _status = FrontEndStatus::INITING;
        bool init();

        // orb feature detect
        int detectFeature();

        // eipolar estimation of the first frame
        bool findPos2d2d();

        Frame::Ptr _current_frame = nullptr; // current frame
        std::shared_ptr<GUI> _gui = nullptr; // gui in other thread
        std::shared_ptr<Map> _map = nullptr; // global map

        // parameters
        const int num_maxFeatures = 500;
        const int num_minFeatures = 80;
        const int num_minTrackingFeatures = 50;
        const int num_keyFrameFeatures = 100;

        cv::Mat _matchedTwoImg;           // for gui drawing
        std::vector<cv::DMatch> _matches; // matches
        cv::Ptr<cv::ORB> _orb;            // orb detector
        cv::BFMatcher _matcher;           // bf matcher

        // matrix
        Sophus::SE3d _relative_pos; // relatibe pose
        Eigen::Matrix3d N; // normalization matrix
    };
}

#endif // __FRONTEND_H__