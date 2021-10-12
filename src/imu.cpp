#include "mySLAM/imu.h"
#include <algorithm>
#include <numeric>

namespace mySLAM
{
    IMU::IMU(std::string usb_portName) : _serialPortName(usb_portName)
    {
        // thread
        _imu_thread = std::thread(&IMU::_imu_loop, this);
    }

    IMU::~IMU()
    {
        _imu_thread.join();
    }

    void IMU::_stop()
    {
        _status = ImuStatus::DISCONNECTED;
        if (_imu_serialPort->is_open())
            _imu_serialPort->close();
        if (_imu_serialPort)
        {
            delete _imu_serialPort;
        }
        // delete buffer
        if (_buffer)
            delete[] _buffer;
    }

    /**
     * @brief initialize the serial port
     * 
     * @return true if init success
     * @return false if error occurs
     */
    bool IMU::initPort()
    {
        boost::system::error_code ec;
        boost::asio::io_service io;
        _imu_serialPort = new boost::asio::serial_port(io);
        _imu_serialPort->open(_serialPortName, ec);
        if (ec)
        {
            LOG(FATAL) << "Can't find the IMU, please check the USB connection ! ";
            return false; // return if can not open serial port
        }

        // setting
        // Base serial settings
        boost::asio::serial_port_base::baud_rate BAUD(BAUD_RATE);
        boost::asio::serial_port_base::flow_control FLOW(boost::asio::serial_port_base::flow_control::none);
        boost::asio::serial_port_base::parity PARITY(boost::asio::serial_port_base::parity::none);
        boost::asio::serial_port_base::stop_bits STOP(boost::asio::serial_port_base::stop_bits::one);
        boost::asio::serial_port_base::character_size CHAR_SIZE(8U);

        _imu_serialPort->set_option(BAUD);
        _imu_serialPort->set_option(FLOW);
        _imu_serialPort->set_option(PARITY);
        _imu_serialPort->set_option(STOP);
        _imu_serialPort->set_option(CHAR_SIZE);

        // buffer
        _buffer = new unsigned char[SLIDING_WINDOW];
        accel_vec = Eigen::Vector3d::Zero();
        accel_vec_ref = Eigen::Vector3d(0.0, 0.0, 1.0);
        anglVel_vec = Eigen::Vector3d::Zero();
        mag_vec = Eigen::Vector3d::Zero();

        LOG(INFO) << "IMU Initializes Sucessfully from USB Port: " + _serialPortName;
        _status = ImuStatus::CONNECTED;
        return true;
    }

    /**
     * @brief check the data frame header (0x55, 0x51)
     * 
     * @return int 
     */
    int IMU::checkDatHead()
    {
        int i = 0;
        for (i = 0; i < SLIDING_WINDOW - 1; i++)
        {
            if (static_cast<int16_t>(_buffer[i]) == 0x55 &&
                static_cast<int16_t>(_buffer[i + 1]) == 0x51)
                return i;
        }

        return -1;
    }

    /**
     * @brief check the sum of total 10 data == 11th data
     * 
     * @param head index offset
     * @return true 
     * @return false 
     */
    inline bool IMU::checkDatSum(const int &head)
    {
        u_char sum = 0;
        for (int i = 0; i < 10; i++)
            sum += (_buffer[head + i]);

        return (sum == _buffer[head + 10]) ? true : false;
    }

    /**
     * @brief read data
     * 
     */
    bool IMU::readPort()
    {
        std::unique_lock<std::mutex> lck(_imu_data_mux); // thread lock, will unlock after this function
        boost::asio::read(*_imu_serialPort, boost::asio::buffer(_buffer, SLIDING_WINDOW));
        int head = checkDatHead();
        if (head != -1)
        {
            // check the sum
            if (!checkDatSum(head))
                return false;

            /**
             * @brief 
             * right-hand coordinate
             * 
             */
            // parse the acceleration data
            _temperature = 0;
            _ax = byte2accel(_buffer[head + 2], _buffer[head + 3]);
            _ay = byte2accel(_buffer[head + 4], _buffer[head + 5]);
            _az = byte2accel(_buffer[head + 6], _buffer[head + 7]);
            accel_vec << _ax, _ay, _az;
            _temperature += byte2temp(_buffer[head + 8], _buffer[head + 9]);

            // check the sum
            if (!checkDatSum(head + 11))
                return false;

            // parse the angular velocity data
            _wx = byte2anglVel(_buffer[head + 13], _buffer[head + 14]);
            _wy = byte2anglVel(_buffer[head + 15], _buffer[head + 16]);
            _wz = byte2anglVel(_buffer[head + 17], _buffer[head + 18]);
            anglVel_vec << _wx, _wy, _wz;
            _temperature += byte2temp(_buffer[head + 19], _buffer[head + 20]);

            // check the sum
            if (!checkDatSum(head + 33))
                return false;

            // parse the magnetic sensor data
            _mx = byte2mag(_buffer[head + 35], _buffer[head + 36]);
            _my = byte2mag(_buffer[head + 37], _buffer[head + 38]);
            _mz = byte2mag(_buffer[head + 39], _buffer[head + 40]);
            mag_vec << _mx, _my, _mz;
            _temperature += byte2temp(_buffer[head + 41], _buffer[head + 42]);
            _temperature *= (1.0 / 3.0); // average
        }
        else
            return false;

        return true;
    }

    /**
     * @brief main thread loop of the imu
     * 
     */
    void IMU::_imu_loop()
    {
        if (initPort())
        {
            _status = ImuStatus::EKF_INITING;
            while (_imu_serialPort->is_open())
            {
                if (!readPort() || _status == ImuStatus::DISCONNECTED)
                    break;
                else
                    ekf_stepAll();

                //usleep(5000);
            }
        }

        _stop();
    }

    inline double IMU::byte2temp(const u_char &L8b, const u_char &H8b)
    {
        return static_cast<int16_t>((H8b << 8) | L8b) / 100.0;
    }

    inline double IMU::byte2accel(const u_char &L8b, const u_char &H8b)
    {
        return static_cast<int16_t>((H8b << 8) | L8b) / 32768.0 * 16 * g;
    }

    inline double IMU::byte2anglVel(const u_char &L8b, const u_char &H8b)
    {
        return static_cast<int16_t>((H8b << 8) | L8b) / 32768.0 * 2e3 * M_PI / 180.0;
    }

    inline double IMU::byte2mag(const u_char &L8b, const u_char &H8b)
    {
        return static_cast<int16_t>((H8b << 8) | L8b);
    }

    inline Eigen::Matrix3d IMU::toCrossProductMat(const Eigen::Vector3d &angleVel)
    {
        // right-handed
        Eigen::Matrix3d cross;
        cross << 0, -angleVel(2, 0), angleVel(1, 0),
            angleVel(2, 0), 0, -angleVel(0, 0),
            -angleVel(1, 0), angleVel(0, 0), 0;
        return cross;
    }

    inline Eigen::Matrix4d IMU::toOmegaMat(const Eigen::Vector3d &angleVel)
    {
        Eigen::Matrix4d Omega;
        // right-handed
        Omega << 0, -angleVel.transpose(),
            angleVel, -toCrossProductMat(angleVel);
        return Omega;
    }

    void IMU::quaternionPropagate(const Eigen::Vector3d &angleVel)
    {
        // right-handed rotation
        // then update the rotation using zero-th order integrator model
        // which assumes the rotation angle is stationary over dt
        double w_norm = angleVel.norm();
        // for small rotation angle
        if (w_norm < (_angleThresh / 32768.0 * 2e3 * M_PI / 180.0))
            _q_imu = _q_imu * Eigen::Quaterniond(1, 0.5 * angleVel.x() * _dt, 0.5 * angleVel.y() * _dt, 0.5 * angleVel.z() * _dt);
        else // for normal case
            _q_imu = _q_imu * Eigen::Quaterniond(Eigen::AngleAxisd(w_norm * _dt, angleVel / w_norm));
    }

    void IMU::stateErrCovPropagate(const Eigen::Vector3d &angleVel)
    {
        double w_norm = angleVel.norm(),
               angle = w_norm * _dt;
        Eigen::Matrix3d w_outer = toCrossProductMat(angleVel);
        Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
        Eigen::Matrix<double, 6, 6> I_66 = Eigen::Matrix<double, 6, 6>::Identity();

        if (w_norm < (_angleThresh / 32768.0 * 2e3 * M_PI / 180.0))
        {
            _stateErrTran.block(0, 0, 3, 3) = I - _dt * w_outer;
            _stateErrTran.block(0, 3, 3, 3) = -I * _dt;
            _processNoiseCov.block(0, 0, 3, 3) = _sigma2_gyroNoise * _dt * I;
            _processNoiseCov.block(0, 3, 3, 3) = _sigma2_gyroBias * pow(_dt, 2) * I;
        }
        else
        {
            _stateErrTran.block(0, 0, 3, 3) = Eigen::AngleAxisd(angle, angleVel / w_norm).toRotationMatrix().transpose();
            _stateErrTran.block(0, 3, 3, 3) = -_dt * I;

            _processNoiseCov.block(0, 0, 3, 3) = _sigma2_gyroNoise * _dt * I;
            _processNoiseCov.block(0, 3, 3, 3) = -_sigma2_gyroBias * pow(_dt, 2) * I;
        }
        _processNoiseCov.block(3, 0, 3, 3) = _processNoiseCov.block(0, 3, 3, 3).transpose();
        _processNoiseCov.block(3, 3, 3, 3) = _sigma2_gyroBias * _dt * I;

        // predict the error covariance
        _stateErrCov = _stateErrTran * _stateErrCov * _stateErrTran.transpose() + I_66 * _processNoiseCov * I_66.transpose();
    }

    bool IMU::state_init()
    {
        // store the history (queue)
        if (accel_history.size() < num_maxHistory)
        {
            accel_history.push_back(accel_vec);
            return false;
        }
        else
        {
            // Init the EKF
            _q_imu = Eigen::Quaterniond(1.0, 0, 0, 0.0);
            _stateErrTran = EigMatrix6d::Identity();
            _stateErrCov = 1e-6*EigMatrix6d::Identity();
            _sigma2_gyroBias = 0;
            _sigma2_gyroNoise = 1e-8;

            Eigen::Vector3d mean = std::accumulate(accel_history.begin(),accel_history.end(), Eigen::Vector3d(0,0,0))/accel_history.size();
            Eigen::Vector3d var = std::accumulate(accel_history.begin(),accel_history.end(), Eigen::Vector3d(0,0,0),
                                                   [&mean](const Eigen::Vector3d& a,const Eigen::Vector3d& b){return a + (b-mean).cwiseAbs2();})/accel_history.size();
            _accel_bias.x() = mean.x();
            _accel_bias.y() = mean.y();
            _accel_bias.z() = 0;
            _accelNoiseCov = Eigen::Matrix3d::Identity();
            _accelNoiseCov(0,0) *= var.x();
            _accelNoiseCov(1,1) *= var.y();
            _accelNoiseCov(2,2) *= var.z();
            return true;
        }
    }

    void IMU::state_predict(const Eigen::Vector3d &angleVel)
    {
        // find the expected angular velocity (delete the bias)
        Eigen::Vector3d w_hat = angleVel - _gyro_bias;

        // propagate the quaternions
        quaternionPropagate(w_hat);

        // predict the covariane
        stateErrCovPropagate(w_hat);
    }

    void IMU::state_update(const Eigen::Vector3d &accel)
    {
        // q is from local to global, we need to find the local coordinate
        Eigen::Vector3d accel_est = _q_imu.toRotationMatrix().transpose() * accel_vec_ref;
        Eigen::Vector3d accel_residue = (accel-_accel_bias).normalized() - accel_est;

        // compute H
        EigMatrix3x6d H = EigMatrix3x6d::Zero();
        H.block(0, 0, 3, 3) = toCrossProductMat(accel_est);

        // // residue cov
        Eigen::Matrix3d S = H * _stateErrCov * H.transpose() + _accelNoiseCov;

        // kalman gain
        Eigen::Matrix<double, 6, 3> K = _stateErrCov * H.transpose() * S.inverse();

        // find the error state
        EigVector6d stateErr = K * accel_residue;
        Eigen::Vector3d delta_theta = stateErr.topRows(3);
        Eigen::Quaterniond quaternionErr(Eigen::AngleAxisd(delta_theta.norm(), delta_theta.normalized()));

        // update the true state
        _q_imu = _q_imu * quaternionErr;
        _gyro_bias += stateErr.block(3, 0, 3, 1);

        // update the error covariance matrix
        EigMatrix6d I_KH = EigMatrix6d::Identity() - K * H;
        _stateErrCov = I_KH * _stateErrCov * I_KH.transpose() + K * _accelNoiseCov * K.transpose();
    }

    void IMU::ekf_stepAll()
    {
        if(_status == ImuStatus::EKF_INITING)
        {
            if(state_init())
            {
                _status = ImuStatus::WORKING;
                LOG(INFO) << "ESKF Init Success";
            }
            else return;
        }
        state_predict(anglVel_vec);
        state_update(accel_vec);
    }

}