/**
 * @file landMark.h
 * @author xin li
 * @brief 3D landmark from the triangulation
 * @version 0.1
 * @date 2021-10-04
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once
#ifndef __LANDMARK_H__
#define __LANDMARK_H__

#include "common_include.h"

namespace mySLAM
{
    struct LandMark
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<LandMark> Ptr;
    
    public:
        LandMark();
    };
}

#endif // __LANDMARK_H__