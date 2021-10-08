#include "mySLAM/imu.h"

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
        anglVel_vec = Eigen::Vector3d::Zero();
        mag_vec = Eigen::Vector3d::Zero();

        // quaternion (default z axis)
        _q_imu = Eigen::Quaterniond(1.0, 0, 0, 0.0);

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
            if (static_cast<int16_t>(_buffer[i]) == 0x55 && static_cast<int16_t>(_buffer[i + 1]) == 0x51)
            {
                return i;
            }
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
        if (sum != _buffer[head + 10])
            return false;
        else
            return true;
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


            // store the history
            if(angleVel_history.size()<num_maxHistory)
            {
                angleVel_history.push_back(anglVel_vec);
            }
            else
            {
                angleVel_history.erase(angleVel_history.begin());
                angleVel_history.push_back(anglVel_vec);
            }
            // solve the quaternion
            quaternionPropagte(anglVel_vec);
        }
        else
            return false;

        return true;
    }

    /**
     * @brief main loop of the imu
     * 
     */
    void IMU::_imu_loop()
    {
        if (initPort())
        {
            _status = ImuStatus::WORKING;
            while (_imu_serialPort->is_open())
            {
                if (!readPort() || _status == ImuStatus::DISCONNECTED)
                {
                    break;
                }
                usleep(5000);
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

    Eigen::Matrix3d IMU::toCrossProductMat(const Eigen::Vector3d &angleVel)
    {
        Eigen::Matrix3d cross;
        cross << 0,             angleVel(2,0), -angleVel(1,0),
                -angleVel(2,0), 0,              angleVel(0,0),
                angleVel(1,0), -angleVel(0,0),  0;
        return cross;
    }

    Eigen::Matrix4d IMU::toOmegaMat(const Eigen::Vector3d &angleVel)
    {
        Eigen::Matrix4d Omega;
        Omega << toCrossProductMat(angleVel), angleVel,
                 angleVel.transpose(), 0;
        return Omega;
    }

    void IMU::quaternionPropagte(const Eigen::Vector3d &angleVel)
    {
        // transfer the local rotation to the world rotation
        Eigen::Vector3d worldAngleVel = _q_imu.toRotationMatrix() * angleVel;

        // then update the rotation using zero-th order integrator model
        // which assumes the rotation angle is stationary over dt
        double w_norm = worldAngleVel.norm();
        if (w_norm < (5 / 32768.0 * 2e3 * M_PI / 180.0))
        {
            _q_imu.coeffs() = (Eigen::Matrix4d::Identity() + 
                               0.5 * _dt * toOmegaMat(angleVel)) * _q_imu.coeffs();
        }
        else
        {
            Eigen::AngleAxisd w(w_norm * _dt,  worldAngleVel / w_norm);
            Eigen::Quaterniond _q_rot(w);
            _q_imu = _q_rot * _q_imu;
        }

    }

}