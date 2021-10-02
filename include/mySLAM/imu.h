#ifndef __IMU_H__
#define __IMU_H__

/**
 * @file imu.h
 * @author xin Li
 * @brief hands_free_imu driver
 * @version 0.1
 * @date 2021-10-02
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <iostream>
#include <thread>
#include <string>
#include <unistd.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <boost/asio.hpp>

namespace mySLAM
{
    class IMU
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<IMU> Ptr;

        IMU(std::string usb_portName = "/dev/ttyUSB0");
        ~IMU();

    private:
        std::thread _imu_thread;

        enum
        {
            BAUD_RATE = 921600, // baud rate
            PKG_SIZE = 44 // 44 bytes data
        };
        std::string _serialPortName;
        boost::asio::serial_port *_imu_serialPort;
        unsigned char* _buffer;
        int checkDatHead();
        bool initPort();
        void readPort();

    private:
        double _ax,_ay,_az; // acceleration
        double _wx,_wy,_wz; // angle velocity
        double _mx,_my,_mz; // 3d magnetic sensor data

        void byte2accel(const int& H8b, const int& L8b);
        void byte2anglVel(const int& H8b, const int& L8b);
        void byte2mag(const int& H8b, const int& L8b);
    };
}
#endif // __IMU_H__