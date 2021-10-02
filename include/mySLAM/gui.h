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

#ifndef __GUI_H__
#define __GUI_H__

#include <iostream>
#include <unistd.h>
#include <vector>
#include <string>
#include <thread>
#include <pangolin/pangolin.h>

namespace mySLAM
{
    class GUI
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<GUI> Ptr; // share the GUI data
         
        GUI(std::string winName, int w=800, int h=600);
        ~GUI();

    private:
        void _main_loop();
        void _showSensor_loop();

    private:
        std::thread _main_thread, _showSensor_thread;

        std::string _windowName;
        int _width, _height;
    };
}

#endif // __GUI_H__