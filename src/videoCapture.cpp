#include "mySLAM/videoCapture.h"
#include <fstream>
#include <iterator>
#include <opencv2/core/eigen.hpp>

namespace mySLAM
{
    Eigen::Matrix3d VideoCapture::K;

    VideoCapture::VideoCapture()
    {
    }

    VideoCapture::~VideoCapture()
    {
        if (_cap)
        {
            if (_cap->isOpened())
            {
                _cap->release();
            }
            delete _cap;
            _status = VideoCaptureStatus::DISCONNECTED;
        }
    }

    bool VideoCapture::init()
    {
        LOG(INFO) << "Ready to open USB cam...";
        _cap = new cv::VideoCapture(0);
        _cap->set(cv::CAP_PROP_FRAME_WIDTH, width);
        _cap->set(cv::CAP_PROP_FRAME_HEIGHT, height);
        if(!_cap->isOpened())
        {
            LOG(FATAL)<<"Can't connect USB cam, please check the conection!!";
            return false;
        }

        // load calibration parameters
        LOG(INFO) << "Ready to read calibration parameters...";
        if(!load_caliParameters("/home/lx6/MyWork/SLAM/Config/", _cameraMat, _distCoeffs, _newCameraMat))
        {
            LOG(FATAL) << "Can't find the calibration file at /Config/";
            return false;
        }
        else LOG(INFO) << "Read calibration parameters success at ./Config/";
        cv::cv2eigen(_newCameraMat, K);

        LOG(INFO) << "Open USB cam Success!";
        _status = VideoCaptureStatus::CONNECTED;
        return true;
    }

    /**
     * @brief generate new frames
     * 
     * @return Frame::Ptr 
     */
    Frame::Ptr VideoCapture::NextFrame()
    {
        if(_img.empty())
        {
            LOG(INFO) << "Ready to init the 1st camera image...";
            cv::Mat _currImg;
            _cap->read(_currImg);
            cv::undistort(_currImg,_img,_cameraMat,_distCoeffs); // adjust distortion
            LOG(INFO) << "Init the 1st camera image sucess!";
            _status = VideoCaptureStatus::INITING;
            return nullptr;
        }
        else
        {
            auto newFrame = Frame::createFrame();
            cv::Mat _dist_img;
            _cap->read(_dist_img);
            _img.copyTo(newFrame->prev_img); // previous frame
            cv::undistort(_dist_img,_img,_cameraMat,_distCoeffs); // adjust distortion
            _img.copyTo(newFrame->curr_img); // current frame
            
            _status = VideoCaptureStatus::WORKING;
            return newFrame;
        }
    }
    
    bool VideoCapture::load_caliParameters(const std::string &saved_DIR, cv::Mat &cameraMat,
                                     cv::Mat &distCoeffs, cv::Mat &newCameraMat) 
    {
        std::ifstream fin(saved_DIR + "CaliParameters.txt", std::ios::in);
        if (fin.is_open())
        {
            std::istream_iterator<double> begin(fin), end; // iterator to read all the double data
            std::vector<double> data(begin, end);          // load all data to the vector
            if (data.size() == 23)
            {
                // assign data (CV_64F = double)
                cameraMat = cv::Mat(3, 3, CV_64F, (std::vector<double>(data.begin(), data.begin() + 9)).data()).clone();
                newCameraMat = cv::Mat(3, 3, CV_64F, (std::vector<double>(data.begin() + 9, data.begin() + 18)).data()).clone();
                distCoeffs = cv::Mat(1, 5, CV_64F, (std::vector<double>(data.begin() + 18, data.end())).data()).clone();
                return true;
            }
            else
                return false;
        }
        else
        {
            return false;
        }
    }

}