/**
 * @file videoCapture.h
 * @author xin li
 * @brief produce video frames from camera or video source
 * @version 0.1
 * @date 2021-10-03
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once
#ifndef __VIDEOCAPTURE_H__
#define __VIDEOCAPTURE_H__

#include "common_include.h"
#include "frame.h"

namespace mySLAM
{
    enum class VideoCaptureStatus{CONNECTED, DISCONNECTED, INITING, WORKING};

    class VideoCapture
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<VideoCapture> Ptr;
        VideoCapture();
        ~VideoCapture();

        // init
        bool init();
        static const int width = 640,
                         height = 480;
        static Eigen::Matrix3d K;
        
        // get status
        VideoCaptureStatus status() {return _status;}

        //capture and return the next frame
        Frame::Ptr NextFrame();

        // camera intrinsic matrix
        const cv::Mat& cameraMat() {return _newCameraMat;}

    private:
        bool load_caliParameters(const std::string &saved_DIR, cv::Mat &cameraMat,
                                 cv::Mat &distCoeffs, cv::Mat &newCameraMat);
        VideoCaptureStatus _status = VideoCaptureStatus::DISCONNECTED;
        cv::VideoCapture *_cap;
        cv::Mat _img;
        cv::Mat _cameraMat, _distCoeffs, _newCameraMat;

    };
}

#endif // __VIDEOCAPTURE_H__