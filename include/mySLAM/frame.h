/**
 * @file frame.h
 * @author xin li
 * @brief two adjacent images from the camera
 * @version 0.1
 * @date 2021-10-03
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once
#ifndef __FRAME_H__
#define __FRAME_H__

#include "common_include.h"

namespace mySLAM
{
    // forward declare
    struct Feature2d;
    struct LandMark;

    struct Frame
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<Frame> Ptr;

        u_long ID = 0;              // id of this frame
        double timeStamp;           // time stamp
        bool isKeyFrame = false;    // is the key frame
        u_long keyID = 0;           // key frame id
        Sophus::SE3d currCamPos;    // current camera poses
        cv::Mat prev_img, curr_img; // two adjacent images
        std::vector<std::shared_ptr<Feature2d>> prev_feature,
            curr_feature;                         // features in two images
        cv::Mat prev_descripter, curr_descripter; // feature descripter
        std::mutex camPos_data_mutex;             // data mutex for the cam pos

    public:
        Frame();
        Frame(u_long _id, double _time_stamp, const Sophus::SE3d &_pos, const cv::Mat &_prev_img,
              const cv::Mat &_curr_img);

        // set & get poses, thread safe
        Sophus::SE3d pose()
        {
            std::unique_lock<std::mutex> lck(camPos_data_mutex);
            return currCamPos;
        }

        // set this frame as the key frame
        void setThisAsKey();

        // factory construction model & assign ID
        static std::shared_ptr<Frame> createFrame();
    };

}

#endif // __FRAME_H__