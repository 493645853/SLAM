#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include <Eigen/Cholesky>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include "opencv2/imgcodecs/legacy/constants_c.h"
#include <pangolin/pangolin.h>
#include <chrono>
#include <cmath>

#include "visualOdometry.hpp"
#include "slamPlot.hpp"