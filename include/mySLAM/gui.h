/**
 * @file gui.h
 * @author Xin Li
 * @brief GUI for the slam 
 * @version 0.1
 * @date 2021-10-01
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#pragma once

#ifndef __GUI_H__
#define __GUI_H__

#include <thread>
#include <pangolin/pangolin.h>

#include "common_include.h"
#include "imu.h"

namespace mySLAM
{
    class IMU;

    namespace GuiStatus
    {
        enum class MainWinStatus
        {
            OPENED,
            CLOSED
        };
        enum class DataWinStatus
        {
            OPENED,
            CLOSED
        };
    }
    /**
     * @brief GUI Class for data viewing
     * 
     */
    class GUI
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<GUI> Ptr; // share the GUI data

        GUI(std::string winName, int w = 800, int h = 600);
        ~GUI();

        // main window status
        GuiStatus::MainWinStatus mainWindowStatus() { return _mainWinStatus; }
        // data window status
        GuiStatus::DataWinStatus dataWindowStatus() { return _dataWinStatus; }

        // External interface
        void updateIMUInfo(); // update imu info
        void updateKeyPtsImg(const cv::Mat& img){img.copyTo(_keyPtsImg);} 

        // link
        void linkToIMU(std::shared_ptr<IMU> imuPtr) { _imu = imuPtr; }

    private:
        void _main_loop();
        void _showSensor_loop();

    private:
        void drawCamera(float R, float G, float B);
        void drawCoordinate(float scale);

    private:
        GuiStatus::MainWinStatus _mainWinStatus = GuiStatus::MainWinStatus::CLOSED;
        GuiStatus::DataWinStatus _dataWinStatus = GuiStatus::DataWinStatus::CLOSED;
        std::thread _main_thread, _showSensor_thread;
        std::mutex _gui_data_mux;
        std::string _windowName;
        int _width, _height;
        cv::Mat _keyPtsImg;

    private:
        std::shared_ptr<IMU> _imu = nullptr;
    };
}

#endif // __GUI_H__