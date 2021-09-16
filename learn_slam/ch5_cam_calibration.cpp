#include<iostream>
#include<opencv2/opencv.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<string>
#include<vector>
#include"stereoCalibration.hpp"

/**
 * @brief SLAM 14 讲 第5章 相机与图像
 * 相机模型, 标定, 点云拼接
 * 
 * @return int 
 */

/**
 * @brief 
 * Delete the images in the specified directory
 * @param chessBoards_saved_DIR 
 * Path of the directory where the images are stored
 */
void del_allImages(const std::string &chessBoards_saved_DIR)
{
    std::vector<std::string> chessImg_del_DIR;         // Directory to the chess board images
    cv::glob(chessBoards_saved_DIR, chessImg_del_DIR); // obtain the image directories
    for (auto &i : chessImg_del_DIR)
        std::remove(i.data());
    std::cout << "Clear All Images" << std::endl;
}



/**
 * @brief 
 * Main Lopp
 * @return int 
 */
int main()
{
    /**
     * @brief 
     * 首先获得chess board 图片
     * 
     */
    // Video capture object 
    cv::VideoCapture cap(1,cv::CAP_DSHOW);
    cv::Mat img;

    // Counter to record the number of saved images
    int img_saved_cnt = 0;
    constexpr int MAX_SAVED_IMG = 20; // const Maximum images

    // The directionary where the images are saved
    std::string chessBoards_saved_DIR = "D:/SelfLearning_vscode/MyProject/learn_slam/Calibration/Graphs/";

    while(cap.isOpened())
    {
        cap>>img;
        cv::imshow("Camera",img);

        int presskey = cv::waitKey(1);
        if(presskey=='s')
        {
            // save imgs (less than the maximum)
            (img_saved_cnt>MAX_SAVED_IMG)? (img_saved_cnt = 0):(img_saved_cnt +=1);            
            std::string img_name = "chess_" + std::to_string(img_saved_cnt)+".jpg";
            std::cout<<"Save Img "<< img_name <<std::endl;   
            cv::imwrite(chessBoards_saved_DIR+img_name, img);
        }
        else if(presskey=='c')
        {
            img_saved_cnt = 0;
            del_allImages(chessBoards_saved_DIR);
        }
        else if(presskey==27)
        {
            std::cout<<"Finish collecting images"<<std::endl;
            cv::destroyAllWindows();
            cap.release();
            break;
        }
        
    }

    std::cout<<"Start to calibrate the camera"<<std::endl;

    /**
     * @brief 
     * 寻找图片角点
     * 
     */
    constexpr int CHESS_WIDTH = 10; // width of the chessboard
    constexpr int CHESS_HEIGHT = 7; // height of the chessboard
    constexpr float CHESS_SPACING = 0.018; // 1.8 cm spacing

    cv::Size2i CHESS_PATTERN_SIZE(CHESS_WIDTH-1,CHESS_HEIGHT-1); // Size of the chessboard conner
    std::vector<cv::Point3f> conners_world_single; // the 3D conners in the real world for 1 images
    std::vector<std::vector<cv::Point3f>> conners_world_all; // the 3D conners in the real world for all images

    // init the chess board conners in the world coordinate
    for(int i=0;i<CHESS_PATTERN_SIZE.height;i++)
    {
        for(int j=0;j<CHESS_PATTERN_SIZE.width;j++)
        {
            conners_world_single.push_back(cv::Point3f(CHESS_SPACING*j,CHESS_SPACING*i,0));
        }
    }

    
    std::vector<cv::String> chessImg_DIR; // Directory to the chess board images
    cv::glob(chessBoards_saved_DIR,chessImg_DIR); // obtain the image directories
    std::vector<std::vector<cv::Point2f>> conners_pixel_all; // the 2D conners for all images

    for(int i=0;i<chessImg_DIR.size();i++)
    {
        cv::Mat chessImg = cv::imread(chessImg_DIR[i]);
        std::vector<cv::Point2f> conner_buf; // buffer

        if(!cv::findChessboardCorners(chessImg,CHESS_PATTERN_SIZE,conner_buf))
        {
            std::cout<<std::string("Cant't find the conners for the img "+std::to_string(i+1))<<std::endl;
        }
        else
        {
            // sub-pixel conner finder
            cv::Mat chessImg_gray;
            cv::cvtColor(chessImg,chessImg_gray,cv::COLOR_RGB2GRAY);
            cv::find4QuadCornerSubpix(chessImg_gray,conner_buf,CHESS_PATTERN_SIZE);
            conners_pixel_all.push_back(conner_buf); // store the conner result
            conners_world_all.push_back(conners_world_single); // store the world coordinates

            std::cout<<std::string("Sucessfully find the conners for the img "+std::to_string(i+1))<<std::endl;

            // draw the results
            cv::drawChessboardCorners(chessImg,CHESS_PATTERN_SIZE,conner_buf,true);
            imshow(std::string("Chessboard conners for img "+std::to_string(i+1)),chessImg);
            cv::waitKey(1); // pause for 0.5s. 
        }
    }
    cv::destroyAllWindows();


    /**
     * @brief 
     * 开始标定
     * 
     */
    cv::Mat cameraMatrix,newCameraMat,distCoeffs,R,T;
    cv::calibrateCamera(conners_world_all,conners_pixel_all,cv::Size(img.rows,img.cols),cameraMatrix,distCoeffs,R,T);
    newCameraMat = cv::getOptimalNewCameraMatrix(cameraMatrix,distCoeffs,cv::Size(img.rows,img.cols),0,cv::Size(img.rows,img.cols)).clone();

    std::cout << "\n cameraMatrix : \n" << cameraMatrix << std::endl;
    std::cout << "\n New cameraMatrix : \n" << newCameraMat << std::endl;
    std::cout << "\n distCoeffs : \n" << distCoeffs << std::endl;
    // std::cout << "Rotation vector : \n" << R << std::endl;
    // std::cout << "Translation vector : \n" << T << std::endl;

    //save result
    stereo::save_CaliParameters("D:/SelfLearning_vscode/MyProject/learn_slam/Calibration/",cameraMatrix,distCoeffs,newCameraMat);

    /**
     * @brief 
     * 展示去畸变结果
     * 
     */
    cv::VideoCapture cap_undist(1);
    cv::Mat chessImg = cv::imread(chessImg_DIR[1]);
    cv::Mat chessImg_undist;
    cv::undistort(chessImg,chessImg_undist,cameraMatrix,distCoeffs,newCameraMat);

    while(1)
    {
        cap_undist>>img;
        cv::Mat img_undist;
        cv::undistort(img,img_undist,cameraMatrix,distCoeffs);
        cv::imshow("Undistored Camera",img_undist);
        cv::imshow("original img",chessImg);
        cv::imshow("Undistored Img",chessImg_undist);
        if(cv::waitKey(1)>=0)
        {
            break;
        }
    }

    return 0;
}