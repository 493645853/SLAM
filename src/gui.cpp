#include "mySLAM/gui.h"

namespace mySLAM
{
    GUI::GUI(std::string winName, int w, int h) : _windowName(winName), _width(w), _height(h)
    {
        _main_thread = std::thread(&mySLAM::GUI::_main_loop, this);
        _showSensor_thread = std::thread(&mySLAM::GUI::_showSensor_loop, this);
    }

    GUI::~GUI()
    {
        _showSensor_thread.join();
        _main_thread.join();
    }

    void GUI::_main_loop()
    {
        _mainWinStatus = GuiStatus::MainWinStatus::OPENED;
        // create window
        pangolin::CreateWindowAndBind(this->_windowName, this->_width, this->_height);

        // enable 3d mouse handler
        glEnable(GL_DEPTH_TEST);

        // Issue specific OpenGl we might need
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        pangolin::OpenGlRenderState main_handler(
            pangolin::ProjectionMatrix(800, 600, 650, 650, 400, 300, 0.1, 6500), // intrinsic matrix (fake)
            pangolin::ModelViewLookAt(0, 0, -3, 0, 0, 0, pangolin::AxisNegY));

        // add the main (cloud points) view
        pangolin::View &d_main = pangolin::CreateDisplay()
                                     .SetAspect(-800.0f / 600.0f)
                                     .SetHandler(new pangolin::Handler3D(main_handler));

        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        // Default hooks for exiting (Esc) and fullscreen (tab).
        while (!pangolin::ShouldQuit())
        {
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            d_main.Activate(main_handler);
            if(_imu)
            {
                Eigen::Isometry3d T(_imu->getQuaternion());
                glMultMatrixd(T.data());
                drawCoordinate(0.2);
                drawCamera(0.0, 0.0, 1.0f);
                glPopMatrix();
            }


            // Swap frames and Process Events
            pangolin::FinishFrame();
            usleep(5000);
        }

        // destroy window if thread terminates
        pangolin::DestroyWindow(_windowName);
        _mainWinStatus = GuiStatus::MainWinStatus::CLOSED;
    }

    void GUI::_showSensor_loop()
    {
        _dataWinStatus = GuiStatus::DataWinStatus::OPENED;
        // create window
        pangolin::CreateWindowAndBind("Imu Data Viewer", 640, 240);

        // enable 3d mouse handler
        glEnable(GL_DEPTH_TEST);

        // Issue specific OpenGl we might need
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        // add the IMU orientation view
        pangolin::OpenGlRenderState IMU_pos_handler(
            pangolin::ProjectionMatrix(800, 600, 650, 650, 400, 300, 0.1, 6500),
            pangolin::ModelViewLookAt(2, 0, 0, 0, 0, 0, pangolin::AxisZ));
      
        pangolin::View &d_IMU_pos = pangolin::CreateDisplay()
                                        .SetHandler(new pangolin::Handler3D(IMU_pos_handler));

        // data plotter of the IMU
        pangolin::DataLog IMU_dat_log; // data logger
                                       // Optionally add named labels
        std::vector<std::string> labels;
        labels.push_back(std::string("ax"));
        labels.push_back(std::string("ay"));
        labels.push_back(std::string("az"));
        IMU_dat_log.SetLabels(labels);
        pangolin::Plotter IMU_dat_plotter(&IMU_dat_log, 0.0f, 500.0f, -20.0f, 20.0f, 200.0f, 5.0f);
        IMU_dat_plotter.Track("$i");
    

        pangolin::Display("IMU")
                        .SetLayout(pangolin::LayoutEqualHorizontal)
                        .AddDisplay(d_IMU_pos)
                        .AddDisplay(IMU_dat_plotter);

        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        // Default hooks for exiting (Esc) and fullscreen (tab).
        while (!pangolin::ShouldQuit())
        {
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            if (_imu)
            {
                std::unique_lock<std::mutex> lck(_gui_data_mux);
                IMU_dat_log.Log(_imu->ax(), _imu->ay(), _imu->az());
                
                // load quaternion
                d_IMU_pos.Activate(IMU_pos_handler);
                glPushMatrix();
                // right-hand coordinate
                Eigen::Isometry3d T(_imu->getQuaternion());
                glMultMatrixd(T.data());
                pangolin::glDrawColouredCube(-0.2, 0.2);
                drawCoordinate(0.6);
                glPopMatrix();
            }

            if(!_keyPtsImg.empty())
            {
                cv::imshow("Orb KeyPoints", _keyPtsImg);
                cv::waitKey(1);
            }

            // Swap frames and Process Events
            pangolin::FinishFrame();
            usleep(10000);
        }
        // destroy window if thread terminates
        pangolin::DestroyWindow("Imu Data Viewer");
        _dataWinStatus = GuiStatus::DataWinStatus::CLOSED;
    }

    void GUI::drawCamera(float R, float G, float B)
    {
        const float w = 0.1;
        const float h = w * 0.75;
        const float z = w * 0.6;
        glLineWidth(1);
        glColor3f(R, G, B);
        glBegin(GL_LINES);
        glVertex3f(0, 0, 0);
        glVertex3f(w, h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(w, -h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(-w, -h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(-w, h, z);

        glVertex3f(w, h, z);
        glVertex3f(w, -h, z);

        glVertex3f(-w, h, z);
        glVertex3f(-w, -h, z);

        glVertex3f(-w, h, z);
        glVertex3f(w, h, z);

        glVertex3f(-w, -h, z);
        glVertex3f(w, -h, z);
        glEnd();
    }

    void GUI::drawCoordinate(float scale)
    {
        glLineWidth(3);
        glBegin(GL_LINES);
        //x
        glColor3f(0.0, 0.0, 1.0);
        glVertex3d(0, 0, 0);
        glVertex3d(scale, 0, 0);
        //y
        glColor3f(0.0, 1.0, 0.0);
        glVertex3d(0, 0, 0);
        glVertex3d(0, scale, 0);
        //z
        glColor3f(1.0, 0.0, 0.0);
        glVertex3d(0, 0, 0);
        glVertex3d(0, 0, scale);
        glEnd();
    }

}