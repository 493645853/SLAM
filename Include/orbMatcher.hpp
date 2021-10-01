#ifndef __ORBMATCHER_H__
#define __ORBMATCHER_H__

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core.hpp>
#include <chrono>
namespace myCV
{
    enum
    {
        NUM_OF_CIRCLE_PIXELS = 16,
        NUM_OF_VALID_PIXELS = 12
    };

    static const int offset16[][2] =
    {
        {0, 3}, {1, 3}, {2, 2}, {3, 1}, {3, 0}, {3, -1}, {2, -2}, {1, -3}, 
        {0, -3}, {-1, -3}, {-2, -2}, {-3, -1}, {-3, 0}, {-3, 1}, {-2, 2}, {-1, 3}};

    /**
    * @brief 
    *   Fast high-speed test of the corner points, the 1,9,5,13 pixels are compared with pixel p
    * @param Img gray img
    * @param row row of desired pixel
    * @param col col of desired pixel
    * @param threshold threshold for deciding the corner
    * @return true if the pixel is corner
    * @return false if the pixel is not corner
    */
    inline bool is_corner(const cv::Mat &Img, const int &row,
                          const int &col, const int &threshold)
    {
        
        int intensity = Img.ptr<uchar>(row)[col],
            intensity5 = Img.ptr<uchar>(row + offset16[12][0])[col + offset16[12][1]],
            intensity1 = Img.ptr<uchar>(row + offset16[0][0])[col + offset16[0][1]],
            intensity13 = Img.ptr<uchar>(row + offset16[4][0])[col + offset16[4][1]],
            intensity9 = Img.ptr<uchar>(row + offset16[8][0])[col + offset16[8][1]];

        int counter = 0;
        bool isCorner = false;
        if (abs(intensity1 - intensity) > threshold && abs(intensity9 - intensity) > threshold)
        {
            counter += 2;
            if (abs(intensity5 - intensity) > threshold)
                counter += 1;
            if (abs(intensity13 - intensity) > threshold)
                counter += 1;
            
            // then check contingutous 8 points
            if(counter>=3)
            {
                for(int i=0;i<NUM_OF_CIRCLE_PIXELS;i++)
                {
                    int continuous_lighter_count = 0,continuous_darker_count = 0;
                    for(int j=0;j<NUM_OF_VALID_PIXELS;j++)
                    {
                        int search_index = (i+j)%NUM_OF_CIRCLE_PIXELS;
                        int search_intensity = Img.ptr<uchar>(row+offset16[search_index][0])[+offset16[search_index][1]];  
                        // are there 8 pts all darker/lighter than the center pt?
                        if((search_intensity-intensity)>threshold)
                            continuous_lighter_count++;
                        else if((intensity-search_intensity)>threshold)
                            continuous_darker_count++;
                    }
                    if(continuous_lighter_count==NUM_OF_VALID_PIXELS || continuous_darker_count==NUM_OF_VALID_PIXELS)
                    {
                        isCorner = true;
                        break;
                    }
                }
            }
        }

        return isCorner;
    }


    inline double FastScore(const cv::Mat &Img, const int &row,const int &col)
    {
        double score = 0.0;

        for(int i=0;i<NUM_OF_CIRCLE_PIXELS;i++)
        {

        }
        return score;
    }


    void FastCornerDetect(const cv::Mat& frame_gray, std::vector<cv::KeyPoint>& key_pt, const int& threshold, bool isMaxSuppresion = false)
    {
        key_pt.clear();
        // range of detection
        const int IMG_DETECT_RANGE_LEFT = 3,
                  IMG_DETECT_RANGE_RIGHT = frame_gray.rows - 3,
                  IMG_DETECT_RANGE_TOP = 3,
                  IMG_DETECT_RANGE_BOTTOM = frame_gray.cols - 3;

        // key points
        cv::KeyPoint pts(0.0f,0.0f,1.0f);
        int key_pts_count = 0;

        // scan the image
        for (int i = IMG_DETECT_RANGE_LEFT; i < IMG_DETECT_RANGE_RIGHT; i++)
        {
            for (int j = IMG_DETECT_RANGE_TOP; j < IMG_DETECT_RANGE_BOTTOM; j++)
            {
                if (is_corner(frame_gray, i, j, threshold))
                {
                    pts.pt.x = static_cast<float>(j);
                    pts.pt.y = static_cast<float>(i);


                    key_pt.push_back(pts);



                }
            }
        }
    }

    
    using DescType=std::vector<uint32_t>;
    /**
     * @brief TODO. slam14讲中似乎没有实现金字塔结构来维护尺度不变性
     * 
     * @param frame_gray 
     * @param key_Pts 
     * @param descriptors 
     * @param half_boundary 
     */
    void orb_genBriefDesc(const cv::Mat& frame_gray, const std::vector<cv::KeyPoint>& key_Pts,
                     std::vector<DescType>& descriptors, int half_boundary=16)
    {
        const int half_patch_size = static_cast<int>(half_boundary/2);
        int bad_pts_count = 0;

        for(auto &pts:key_Pts)
        {
            // ignore the points outside the boundary
            if(pts.pt.x < half_boundary || pts.pt.y < half_boundary ||
               pts.pt.x >= frame_gray.cols-half_boundary || pts.pt.y >= frame_gray.rows - half_boundary)
            {
                bad_pts_count++;
                descriptors.push_back({});
                continue;
            }

            // find the direction of the key pts
            float m01 = 0.0f, m10 = 0.0f; // moments
            for (int dx = -half_patch_size; dx < half_patch_size; dx++)
            {
                int x_search_index = static_cast<int>(pts.pt.x) + dx;
                for (int dy = -half_patch_size; dy < half_patch_size; dy++)
                {
                    int y_search_index = static_cast<int>(pts.pt.y) + dy;
                    m01 += y_search_index*frame_gray.ptr<uchar>(y_search_index)[x_search_index];
                    m10 += x_search_index*frame_gray.ptr<uchar>(y_search_index)[x_search_index];
                }
            }
            // compute the angle
            float m_sqrt = sqrt(m10*m10+m01*m01) + 1e-18;
            float sin_theta = m01/m_sqrt;
            float cos_theta = m10/m_sqrt;

            //TODO
            
        }
    }
}
#endif // __ORBMATCHER_H__