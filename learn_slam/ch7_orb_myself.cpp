#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core.hpp>
#include <chrono>
#include "orbMatcher.hpp"
using namespace std;


int main(int argc, char **argv)
{
    cv::VideoCapture capture(1);
    cv::Mat frame_now;

    // initialization of the frames
    cv::Mat frame_gray;
    std::vector<cv::KeyPoint> key_pt_now;
    std::vector<std::vector<uint32_t>> desc_now;

    while (1)
    {
        capture >> frame_now;
        cv::cvtColor(frame_now,frame_gray,cv::COLOR_RGB2GRAY);
        auto t1 = std::chrono::steady_clock::now();
        cv::FAST(frame_gray, key_pt_now, 20);
        //myCV::FastCornerDetect(frame_gray, key_pt_now, 20);
        auto t2 = std::chrono::steady_clock::now();
        auto t_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
        std::cout << "time cost: " << t_used.count() << "\t number of key pts: " << key_pt_now.size() << "\n";

        cv::Mat frame_key;
        cv::drawKeypoints(frame_now, key_pt_now, frame_key, cv::Scalar(0,255,0),cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        imshow("orb match", frame_key);

        // pause for 1 ms
        if(cv::waitKey(1)>=0)
        {
            break;
        };

    }

    
    return 0;
}