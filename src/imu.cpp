#include "mySLAM/imu.h"

namespace mySLAM
{
    IMU::IMU(std::string usb_portName) : _serialPortName(usb_portName)
    {
        // thread
        _imu_thread = std::thread(&IMU::readPort, this);
    }

    IMU::~IMU()
    {
        // delete buffer
        _imu_thread.join();
    }

    int IMU::checkDatHead()
    {
        return 1;
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
            return false; // return if can not open serial port

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
        _buffer = new unsigned char[2 * PKG_SIZE];

        return true;
    }

    /**
     * @brief thread: read data
     * 
     */
    void IMU::readPort()
    {
        if (initPort())
        {
            while (_imu_serialPort->is_open())
            {
                boost::asio::read(*_imu_serialPort, boost::asio::buffer(_buffer, 2 * PKG_SIZE));
                for (int i = 0; i < 2 * PKG_SIZE; i++)
                {
                    std::cout << std::hex << static_cast<int>(_buffer[i]) << " ";
                }
                std::cout << "\n";
            }
        }
        if (_imu_serialPort->is_open())
            _imu_serialPort->close();
        if (_imu_serialPort)
        {
            delete _imu_serialPort;
        }
    }

}