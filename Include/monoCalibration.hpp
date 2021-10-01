#ifndef __MONOCALIBRATION_H__
#define __MONOCALIBRATION_H__

#include <iostream>
#include <iterator>
#include <fstream>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>
#include <string>
#include <vector>

namespace mono
{
    /**
    * @brief 
    * Save the calibration results to the specified directory
    * @param saved_DIR 
    * Directory to save parameters
    * @param cameraMat 
    * camera instrinc matrix
    * @param distCoeffs 
    * distortion index
    * @param newCameraMat
     * new camera instrinc matrix after the cv::getOptimalNewCameraMatrix function.
    */
    bool save_CaliParameters(const std::string &saved_DIR, const cv::Mat &cameraMat,
                             const cv::Mat &distCoeffs, const cv::Mat &newCameraMat)
    {
        std::ofstream fout;
        fout.open(saved_DIR + "CaliParameters.txt", std::ios::out | std::ios::trunc); //new file
        if (fout.is_open())
        {
            Eigen::MatrixXd cameraMat_Eigen;
            Eigen::MatrixXd newCameraMat_Eigen;
            Eigen::MatrixXd distCoeffs_Eigen;

            // convert to eigen matrix (avoid print [])
            cv::cv2eigen(cameraMat, cameraMat_Eigen);
            cv::cv2eigen(newCameraMat, newCameraMat_Eigen);
            cv::cv2eigen(distCoeffs, distCoeffs_Eigen);

            // print the matrix using stream (full precision)
            fout << cameraMat_Eigen.format(Eigen::IOFormat(Eigen::FullPrecision)) << std::endl;
            fout << std::endl;
            fout << newCameraMat_Eigen.format(Eigen::IOFormat(Eigen::FullPrecision)) << std::endl;
            fout << std::endl;
            fout << distCoeffs_Eigen.format(Eigen::IOFormat(Eigen::FullPrecision)) << std::endl;

            fout.close();
            return true;
        }
        else
        {
            return false;
        }
    }

    /**
    * @brief 
    * Read the camera calibration parameters from the specified directory
    * @param saved_DIR Directory to save parameters
    * @param cameraMat Output camera instrinc matrix
    * @param distCoeffs Output distortion index
    * @param newCameraMat Output new camera instrinc matrix after the cv::getOptimalNewCameraMatrix function.
    * @return true If sucessfully load the parameters
    * @return false If fail to load the parameters
    */
    bool load_caliParameters(const std::string &saved_DIR, cv::Mat &cameraMat,
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

    /**
     * @brief 
     * 从txt中读取匹配点
     * 
     * @param DIR 文档存放地址
     * @param match_pts 匹配点
     */
    void readMatches(const std::string &DIR, std::vector<cv::Point2f> &match_pts)
    {
        match_pts.clear();
        std::ifstream fin(DIR, std::ios::in);
        std::istream_iterator<double> begin(fin), end; // iterator to read all the double data
        std::vector<double> data(begin, end);          // load all data to the vector

        for (int i = 0; i < data.size(); i += 2)
        {
            match_pts.push_back(cv::Point2f(data[i], data[i + 1]));
        }
    }
}
#endif // __MONOCALIBRATION_H__