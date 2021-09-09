#include <iostream>
#include <opencv2/opencv.hpp>
#include<opencv2/features2d/features2d.hpp>
#include <opencv2/core.hpp>
using namespace std;


int main(int argc, char **argv)
{
    cv::VideoCapture capture(1);
    cv::Mat frame_now, frame_last;

    // initialization of the frames
    capture >> frame_now;
    capture >> frame_last;

    // key points initialization
    std::vector<cv::KeyPoint> key_pt_now,key_pt_last;
    // descripter initialization
    cv::Mat descptr_now,descptr_last;
    cv::Ptr<cv::ORB> orb = cv::ORB::create(500,1.2f,8,31,0,2,cv::ORB::HARRIS_SCORE,31,20);

    // matcher initialization
    std::vector<cv::DMatch> matches;
    cv::BFMatcher matcher(cv::NORM_HAMMING);

    // match result images
    cv::Mat frame_match;
    
    while (1)
    {
        // update img frames
        capture >> frame_now;

        // detect key points
        orb->detect(frame_last,key_pt_last);
        orb->detect(frame_now,key_pt_now);

        // compute descripter
        orb->compute(frame_last,key_pt_last,descptr_last);
        orb->compute(frame_now,key_pt_now,descptr_now);

        // match descripters
        matcher.match(descptr_last,descptr_now,matches);

        // find good matches
        double min_dist = 1e4, max_dist = 0;
        // find min/max dist
        for(int i = 0;i<descptr_last.rows;i++)
        {
            double dist = matches[i].distance;
            if(dist<min_dist) min_dist = dist;
            if(dist>max_dist) max_dist = dist;
        }

        std::vector<cv::DMatch> good_matches;

        for (int i = 0; i<descptr_last.rows;i++)
        {
            if(matches[i].distance<=max(2*min_dist,30.0))
            {
                good_matches.push_back(matches[i]);
            }
        }

        // draw matches
        cv::drawMatches(frame_last,key_pt_last,frame_now,key_pt_now,good_matches,frame_match);

        imshow("orb match",frame_match);
        
        // pause for 100 ms
        if(cv::waitKey(100)>=0)
        {
            break;
        };
        // update the last frame (must be cloned, or the pointer will be returned)
        frame_last=frame_now.clone();
    }
    return 0;
}