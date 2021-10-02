#include "mySLAM/gui.h"

mySLAM::GUI::GUI(std::string winName, int w, int h) : _windowName(winName), _width(w), _height(h)
{
    _main_thread = std::thread(&mySLAM::GUI::_main_loop, this);
    _showSensor_thread = std::thread(&mySLAM::GUI::_showSensor_loop, this);
}

mySLAM::GUI::~GUI()
{
    _showSensor_thread.join();
    _main_thread.join(); 
}

void mySLAM::GUI::_main_loop()
{
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

    //glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    // Default hooks for exiting (Esc) and fullscreen (tab).
    while (!pangolin::ShouldQuit())
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_main.Activate(main_handler);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        pangolin::glDrawColouredCube();

        // Swap frames and Process Events
        pangolin::FinishFrame();
        usleep(5000);
    }

    // destroy window if thread terminates
    pangolin::DestroyWindow(_windowName); 

}

void mySLAM::GUI::_showSensor_loop()
{
    // create window
    pangolin::CreateWindowAndBind("Sensor Data Viewer", 640, 480);

    // enable 3d mouse handler
    glEnable(GL_DEPTH_TEST);

    // Issue specific OpenGl we might need
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // Two img view
    pangolin::View &d_Two_Frames = pangolin::CreateDisplay();

    // add the IMU orientation view

    pangolin::OpenGlRenderState IMU_pos_handler(
        pangolin::ProjectionMatrix(800, 600, 650, 650, 400, 300, 0.1, 6500),
        pangolin::ModelViewLookAt(0, 0, -2, 0, 0, 0, pangolin::AxisY));
    pangolin::View &d_IMU_pos = pangolin::CreateDisplay()
                                    .SetHandler(new pangolin::Handler3D(IMU_pos_handler));

    // data plotter of the IMU
    pangolin::DataLog IMU_dat_log; // data logger
    pangolin::Plotter IMU_dat_plotter(&IMU_dat_log);
    IMU_dat_plotter.Track("$i");

    pangolin::Display("multi")
        .SetBounds(0.0, 1.0, 0.0, 1.0)
        .SetLayout(pangolin::LayoutEqualVertical)
        .AddDisplay(d_Two_Frames)
        .AddDisplay(pangolin::Display("IMU")
                        .SetLayout(pangolin::LayoutEqualHorizontal)
                        .AddDisplay(d_IMU_pos)
                        .AddDisplay(IMU_dat_plotter));

    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    // Default hooks for exiting (Esc) and fullscreen (tab).
    while (!pangolin::ShouldQuit())
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_Two_Frames.Activate();
        pangolin::glDrawColouredCube();

        d_IMU_pos.Activate(IMU_pos_handler);
        pangolin::glDrawColouredCube();

        IMU_dat_log.Log(1);

        // Swap frames and Process Events
        pangolin::FinishFrame();
        usleep(5000);
    }
    // destroy window if thread terminates
    pangolin::DestroyWindow("Sensor Data Viewer"); 
}
