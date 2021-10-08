/**
 * @file backEnd.h
 * @author xin li
 * @brief back end to optimise map & loop closure & retrace the track
 * @version 0.1
 * @date 2021-10-05
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once
#ifndef __BACKEND_H__
#define __BACKEND_H__

#include "common_include.h"

namespace mySLAM
{
    class BackEnd
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<BackEnd> Ptr;
        BackEnd();
    };
}

#endif // __BACKEND_H__