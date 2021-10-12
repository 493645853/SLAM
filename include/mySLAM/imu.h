#pragma once
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

#include <thread>
#include <boost/asio.hpp>

#include "common_include.h"

namespace mySLAM
{
    enum class ImuStatus
    {
        CONNECTED,
        DISCONNECTED,
        EKF_INITING,
        WORKING
    };
    class IMU
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<IMU> Ptr;

        IMU(std::string usb_portName = "/dev/ttyUSB0");
        ~IMU();

        // terminate thread
        void terminate() { _status = ImuStatus::DISCONNECTED; }

        // status
        ImuStatus status() { return _status; }

        // External interface
        const Eigen::Quaterniond &getQuaternion() { return _q_imu; }
        const Eigen::Vector3d &getAccel() { return accel_vec; }
        const Eigen::Vector3d &getAnglVel() { return anglVel_vec; }
        const Eigen::Vector3d &getMag() { return mag_vec; }
        const double &ax() { return _ax; }
        const double &ay() { return _ay; }
        const double &az() { return _az; }
        const double &wx() { return _wx; }
        const double &wy() { return _wy; }
        const double &wz() { return _wz; }
        const double &mx() { return _mx; }
        const double &my() { return _my; }
        const double &mz() { return _mz; }
        const double &temperature() { return _temperature; }

    private:
        std::thread _imu_thread;
        std::mutex _imu_data_mux;

        enum
        {
            BAUD_RATE = 921600, // baud rate
            DATA_RATE = 200,    // 200 Hz
            RX_RATE = 100,      // down-sampling rate
            PKG_SIZE = 44,      // 44 bytes data
            SLIDING_WINDOW = DATA_RATE / RX_RATE * PKG_SIZE
        };
        std::string _serialPortName;
        boost::asio::serial_port *_imu_serialPort;
        unsigned char *_buffer;
        int checkDatHead();
        bool checkDatSum(const int &);
        bool initPort();
        bool readPort();
        void _imu_loop();
        void _stop();

    private:
        ImuStatus _status = ImuStatus::DISCONNECTED;
        const double g = 9.81; // gravity acceleration
        double _temperature;
        double _ax, _ay, _az; // acceleration
        double _wx, _wy, _wz; // angle velocity
        double _mx, _my, _mz; // 3d magnetic sensor data

        Eigen::Vector3d accel_vec, accel_vec_ref;
        Eigen::Vector3d anglVel_vec;
        Eigen::Vector3d mag_vec;

        inline double byte2temp(const u_char &L8b, const u_char &H8b);
        inline double byte2accel(const u_char &L8b, const u_char &H8b);
        inline double byte2anglVel(const u_char &L8b, const u_char &H8b);
        inline double byte2mag(const u_char &L8b, const u_char &H8b);

        /**
     * @brief for EKF algorithm
     * 
     */
    private:
        const long unsigned int num_maxHistory = 100;
        std::deque<Eigen::Vector3d> accel_history;
        const double _dt = 1.0 / RX_RATE;
        const double _angleThresh = 5;

        // Extended Kalman filter state parameters
        double _sigma2_gyroBias, _sigma2_gyroNoise;
        Eigen::Vector3d _accel_bias, _gyro_bias;
        Eigen::Quaterniond _q_imu; // 4D quaternion, 3d gyro bias
        EigMatrix6d _stateErrCov, _stateErrTran; // error state cov & transform, 3d angle axis, 3d gyro bias
        EigMatrix6d _processNoiseCov;
        Eigen::Matrix3d _accelNoiseCov;
        Eigen::Matrix3d _measurementNoiseCov;

        /* function for kalman filter */
        // compute the equivalent skew-symmetric matrix for input vector
        inline Eigen::Matrix3d toCrossProductMat(const Eigen::Vector3d &angleVel);
        // compute the equivalent omega matrix for the input vector
        inline Eigen::Matrix4d toOmegaMat(const Eigen::Vector3d &angleVel);
        // propagate quaternion using only unbiased gyro measurement
        void quaternionPropagate(const Eigen::Vector3d &angleVel);
        // compute the state covariance
        void stateErrCovPropagate(const Eigen::Vector3d &angleVel);


        bool state_init();
        // EKF predict
        void state_predict(const Eigen::Vector3d &angleVel);
        // EKF update 
        void state_update(const Eigen::Vector3d &accel);
        // perform the whole EKF for 1 iteration  
        void ekf_stepAll();   
    };
}
#endif // __IMU_H__