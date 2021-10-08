#include "mySLAM/frame.h"

namespace mySLAM
{
    Frame::Frame()
    {

    }
    
    Frame::Frame(u_long _id, double _time_stamp, const Sophus::SE3d &_pos, const cv::Mat &_prev_img,
              const cv::Mat &_curr_img):ID(_id),timeStamp(_time_stamp),currCamPos(_pos),
                prev_img(_prev_img),curr_img(_curr_img){}

    void Frame::setThisAsKey()
    {
        static long keyFrame_facory_ID = 0;
        isKeyFrame = true;
        keyID = keyFrame_facory_ID++;
    }

    /**
     * @brief create new frame, controlled by the "VideoCapture" object
     * 
     * @return Frame::Ptr 
     */
    Frame::Ptr Frame::createFrame()
    {
        static long factory_ID = 0;
        Frame::Ptr newFrame(new Frame); // create new frame
        newFrame->ID = factory_ID++;
        return newFrame;
    }
}