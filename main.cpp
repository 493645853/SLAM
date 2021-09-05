#include<iostream>
#include<opencv2/core.hpp>
#include<opencv2/videoio.hpp>
#include<opencv2/opencv.hpp>
#include<opencv2/highgui.hpp>
#include<opencv2/features2d.hpp>

int main( int argc, char** argv )
{
    cv::Mat frame;
    //---INITIALIZE VIDEOCAPTURE
    cv::VideoCapture cap;
    cap.open(0,cv::CAP_ANY);
    if(!cap.isOpened())
    {
        return -1;
    }

    while(1)
    {
        cap.read(frame);
        cv::imshow("Camera",frame);
        if(cv::waitKey(5)>=0)
        {
            break;
        }
    }

    return 0;
}