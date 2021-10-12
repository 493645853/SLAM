#include "mySLAM/vslamMgr.h"

namespace mySLAM
{
    VSlamManager::VSlamManager() 
    {
        
    }
    
    VSlamManager::~VSlamManager() 
    {
        
    }
    
    bool VSlamManager::init() 
    {
        //google::InitGoogleLogging("my SLAM");
        // create components (all shared ptr) and links
        _videoCap = VideoCapture::Ptr(new VideoCapture);
        if(!_videoCap->init()) return false;

        _fontEnd = FrontEnd::Ptr(new FrontEnd);
        _gui = GUI::Ptr(new GUI("visual Slam"));
        _imu = IMU::Ptr(new IMU);
        
        // link
        _gui->linkToIMU(_imu);
        _fontEnd->linkToGUI(_gui);

        return true;
    }
    
    bool VSlamManager::step() 
    {
        Frame::Ptr newFrame = _videoCap->NextFrame();
        if(_videoCap->status()==VideoCaptureStatus::WORKING)
        {
            _fontEnd->step(newFrame);
        }
        return true;
    }
    
    void VSlamManager::run() 
    {
        LOG(INFO) << "Slam Manager running...";
        while(1)
        {
            if(!step()) break;
            if(_gui->mainWindowStatus()==GuiStatus::MainWinStatus::CLOSED &&
               _gui->dataWindowStatus()==GuiStatus::DataWinStatus::CLOSED)
            {
                _imu->terminate();
                break;
            }
            //usleep(30000);
        }
        LOG(INFO) << "Slam Manager terminated";
    }
}