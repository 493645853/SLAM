/**
 * @file feature2d.h
 * @author xin li
 * @brief matching feature
 * @version 0.1
 * @date 2021-10-04
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once
#ifndef __FEATURE2D_H__
#define __FEATURE2D_H__

#include "common_include.h"

namespace mySLAM
{
    struct Frame;
    struct LandMark;

    struct Feature2d
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<Feature2d> Ptr;

        cv::KeyPoint pts;                     // feature location
        std::weak_ptr<Frame> parentFrame;     // frame which owns this feature
        std::weak_ptr<LandMark> bindLandMark; // associated landmark

        bool isOutlier = false; // is outlier point
        bool isFromPrev = true; // is from the previous frame

    public:
        Feature2d();
        Feature2d(const cv::KeyPoint &_pts, const std::shared_ptr<Frame>& _parentFrame)
            : pts(_pts), parentFrame(_parentFrame) {}
    };
}

#endif // __FEATURE2D_H__