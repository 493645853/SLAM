#pragma once
#ifndef __VSLAMMGR_H__
#define __VSLAMMGR_H__

#include "common_include.h"
#include "gui.h"
#include "imu.h"
#include "videoCapture.h"
#include "frontEnd.h"

namespace mySLAM
{
    class VSlamManager
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<VSlamManager> Ptr;
        VSlamManager();
        ~VSlamManager();

        bool init();   // initialize before running the main task
        bool step();   // run one step for vslam
        void run(); // main loop to run all the tasks

    private:
        GUI::Ptr _gui = nullptr;
        IMU::Ptr _imu = nullptr;

        FrontEnd::Ptr _fontEnd = nullptr;
        VideoCapture::Ptr _videoCap = nullptr;
    };
}

#endif // __VSLAMMGR_H__