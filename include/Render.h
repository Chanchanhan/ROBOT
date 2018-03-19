//
// Created by qqh on 18-3-19.
//

#ifndef ROBOT_RENDER_H_H
#define ROBOT_RENDER_H_H

#include <GL/freeglut.h>

#include "glm.h"
#include <opencv2/core/core.hpp>
using namespace OD;
namespace ORD
{

    struct ShapePoseInfo
    {
        GLMmodel* m_shape;
        GLfloat mv_matrix[16];
        std::vector<cv::Point3d> m_bb;  //bound box 8 points
        void setBoundBox();
    };

    class Render
    {
    public:
        static void init(CameraCalibration& calibration, int width, int height, int argc, char** argv);
        static void reshape(int width, int height);
        static void display(void);
        static void display_wgf(void);
        static void keyboard(unsigned char key, int x, int y);
        /*static void mouse(int button, int state, int x, int y);
        static void mouseMovement(int x, int y);*/
        static void buildProjectionMatrix(const CameraCalibration& calibration, GLfloat* projectionMatrix);
        static void matrixFromCV2GL(cv::Mat& cv_matrix, GLfloat* gl_matrix);
        static void rendering();
        static void rendering_for_vg();
        static cv::Point3f get3DPos(int x, int y);
        static cv::Mat getRenderedImg();
        static cv::Mat getRenderedImg(const cv::Point p1, const cv::Point p2);
        static void getDepthImg();
        static void getDepthImg(const cv::Point& p1, const cv::Point& p2);
    public:
        static int m_width;
        static int m_height;
        static CameraCalibration m_calibration;
        static uchar* m_renderImg;
        static float* m_buffer_depth;
        static std::vector<ShapePoseInfo> m_shapePoseInfo;

        static double eye[3];  //camera location
        static double at[3];   //camera at location
        static int mouse_x;
        static int mouse_y;
        static int mouse_button_type;
        static bool mouse_button_pressed;


        static bool m_is_material;

    };
}

#endif //ROBOT_RENDER_H_H
