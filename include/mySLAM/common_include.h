/**
 * @file common_include.h
 * @author xin li
 * @brief include common lib
 * @version 0.1
 * @date 2021-10-03
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#pragma once
#ifndef __COMMON_INCLUDE_H__
#define __COMMON_INCLUDE_H__

#include <iostream>
#include <cmath>
#include <memory>
#include <unistd.h>
#include <vector>
#include <queue>
#include <string>
#include <unordered_map>


// eigen
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

// opencv
#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/features2d/features2d.hpp>

// sophus
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

// glog
#include <glog/logging.h>


// typedef
typedef Eigen::Ref<Eigen::MatrixXd> EigMatfunType;
typedef Eigen::Matrix<double,3,9> EigMatrix3x9d;
typedef Eigen::Matrix<double,3,6> EigMatrix3x6d;
typedef Eigen::Matrix<double,6,1> EigVector6d;
typedef Eigen::Matrix<double,6,6> EigMatrix6d;
typedef Eigen::Matrix<double,7,1> EigVector7d;
typedef Eigen::Matrix<double,8,1> EigVector8d;
typedef Eigen::Matrix<double,8,8> EigMatrix8d;
typedef Eigen::Matrix<double,9,1> EigVector9d;
typedef Eigen::Matrix<double,9,9> EigMatrix9d;
typedef Eigen::Matrix<double,10,1> EigVector10d;
typedef Eigen::Matrix<double,10,10> EigMatrix10d;
#endif // __COMMON_INCLUDE_H__