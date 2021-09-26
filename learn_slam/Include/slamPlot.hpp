#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Cholesky>
#include <Eigen/Geometry>
#include <pangolin/pangolin.h>

namespace myPlot
{
    void drawCamera()
    {
        const float w = 0.1;
        const float h = w * 0.75;
        const float z = w * 0.6;
        glLineWidth(1.5);
        glColor3f(0.0f, 0.0f, 1.0f);
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

    void drawCoordinate(float scale)
    {
        glLineWidth(3);
        glBegin(GL_LINES);
        //x
        glColor3f(1.0, 0.0, 0.0);
        glVertex3d(0, 0, 0);
        glVertex3d(scale, 0, 0);
        //y
        glColor3f(0.0, 1.0, 0.0);
        glVertex3d(0, 0, 0);
        glVertex3d(0, scale, 0);
        //z
        glColor3f(0.0, 0.0, 1.0);
        glVertex3d(0, 0, 0);
        glVertex3d(0, 0, scale);
        glEnd();
    }

    void refresh3DCVPoints(pangolin::View &d_cam, pangolin::OpenGlRenderState &s_cam,
                           std::vector<cv::Point3f> &p_3d,
                           std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> &camPos_history)
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        d_cam.Activate(s_cam);

        // draw world coordinate
        drawCoordinate(0.2);

        // draw camera
        for (int i = 0; i < camPos_history.size(); i++)
        {
            glPushMatrix();
            pangolin::OpenGlMatrix Twc_(camPos_history[i].matrix());
            glMultMatrixd(Twc_.m);
            drawCamera();
            glPopMatrix();

            // trajectory
            if (i > 0)
            {
                glColor3f(1.0, 0.0, 0.0);
                glBegin(GL_LINES);
                Eigen::Vector3d p1 = camPos_history[i - 1].translation(),
                                p2 = camPos_history[i].translation();
                glVertex3d(p1[0], p1[1], p1[2]);
                glVertex3d(p2[0], p2[1], p2[2]);
                glEnd();
            }
        }

        //绘制点
        glPointSize(5.0);   //设置点大小
        glBegin(GL_POINTS); //点设置的开始
        glColor3f(0.0, 0.0, 0.0);
        for (auto &pts : p_3d)
        {
            glVertex3f(pts.x, pts.y, pts.z);
        }
        glEnd(); //点设置的结束
        pangolin::FinishFrame();
        Sleep(5); // sleep 5 ms
    }

}