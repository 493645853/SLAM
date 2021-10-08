/**
 * @file map.h
 * @author xin li
 * @brief map to store all the points
 * @version 0.1
 * @date 2021-10-03
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once
#ifndef __MAP_H__
#define __MAP_H__

#include "common_include.h"

namespace mySLAM
{
    class Map
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        Map();
    };
}

#endif // __MAP_H__