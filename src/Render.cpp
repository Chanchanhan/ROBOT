//
// Created by qqh on 18-3-19.
//

#include "Render.h"
#include <iostream>
#include <opencv2/highgui/highgui.hpp>



using namespace ORD;
cv::Mat m_calibration;
cv::Mat t_calibration;
int Render::m_width;
int Render::m_height;
uchar* Render::m_renderImg;
float* Render::m_buffer_depth;
std::vector<ShapePoseInfo> Render::m_shapePoseInfo;

double Render::eye[3];
double Render::at[3];
int Render::mouse_x = -1;
int Render::mouse_y = -1;
int Render::mouse_button_type = -1;
bool Render::mouse_button_pressed = false;


void ShapePoseInfo::setBoundBox()
{
    assert(m_shape);
    float radius = glmMaxRadius(m_shape);
    int d = std::ceil(radius/2);
    m_bb.push_back(cv::Point3d(-d,-d,-d));
    m_bb.push_back(cv::Point3d(d,-d,-d));
    m_bb.push_back(cv::Point3d(d,d,-d));
    m_bb.push_back(cv::Point3d(-d,d,-d));
    m_bb.push_back(cv::Point3d(-d,-d,d));
    m_bb.push_back(cv::Point3d(d,-d,d));
    m_bb.push_back(cv::Point3d(d,d,d));
    m_bb.push_back(cv::Point3d(-d,d,d));
}

void Render::init(cv::Mat& calibration, int width, int height, int argc, char** argv)
{
    m_calibration = calibration;
    t_calibration = calibration;
    m_width = width;
    m_height = height;
    m_renderImg = new uchar[width*height*3];
    m_buffer_depth = new float[width*height];
    glutInit(&argc,argv);
    glutInitDisplayMode(GLUT_RGBA|GLUT_DEPTH);
    glutInitWindowPosition(0,0);
    glutInitWindowSize(width,height);
    glutCreateWindow("OpenGL");
    glutDisplayFunc(display);


    eye[0] = 0.0; eye[1] = 0.0; eye[2] = 0.0;
    at[0] = 0.0; at[1] = 0.0; at[2] = -50.0;


}

void Render::reshape(int width, int height)
{
    GLfloat projMatrix[16];
    m_width = width;
    m_height = height;
    buildProjectionMatrix(m_calibration,projMatrix);
    glMatrixMode(GL_PROJECTION);
    glLoadMatrixf(projMatrix);
}

void Render::display(void)
{
    glEnable(GL_DEPTH_TEST);
    glDepthRange(0,1);
    glClearColor(0.0,0.0,0.0,1.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_COLOR_MATERIAL);
    glEnable(GL_LIGHT0);
    /*glPolygonMode(GL_FRONT,GL_LINE);
    glPolygonMode(GL_BACK,GL_LINE);*/

    glMatrixMode(GL_MODELVIEW);

    for(int i=0; i<(int)m_shapePoseInfo.size(); i++)
    {
        glLoadIdentity();
        glLoadMatrixf(m_shapePoseInfo[i].mv_matrix);
        if(m_shapePoseInfo[i].m_shape)
            //glmDraw(m_shapePoseInfo[i].m_shape,GLM_NONE);
            glmDraw(m_shapePoseInfo[i].m_shape,GLM_MATERIAL|GLM_SMOOTH);

    }
    glFlush();

}


void Render::rendering()
{

    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutPostRedisplay();
    glutMainLoopEvent();

}



void Render::matrixFromCV2GL(const cv::Mat& cv_matrix, GLfloat* gl_matrix)
{
    const float pi = 3.1415926f;
    cv::Mat rx(4,4,CV_32FC1);
    rx.at<float>(0,0)=1; rx.at<float>(0,1)=0; rx.at<float>(0,2)=0; rx.at<float>(0,3)=0;
    rx.at<float>(1,0)=0; rx.at<float>(1,1)=cos(pi); rx.at<float>(1,2)=-sin(pi); rx.at<float>(1,3)=0;
    rx.at<float>(2,0)=0; rx.at<float>(2,1)=sin(pi); rx.at<float>(2,2)=cos(pi); rx.at<float>(2,3)=0;
    rx.at<float>(3,0)=0; rx.at<float>(3,1)=0; rx.at<float>(3,2)=0; rx.at<float>(3,3)=1;

    cv::Mat T = rx*cv_matrix;
    gl_matrix[0] = T.at<float>(0,0); gl_matrix[4] = T.at<float>(0,1); gl_matrix[8] = T.at<float>(0,2); gl_matrix[12] = T.at<float>(0,3);
    gl_matrix[1] = T.at<float>(1,0); gl_matrix[5] = T.at<float>(1,1); gl_matrix[9] = T.at<float>(1,2); gl_matrix[13] = T.at<float>(1,3);
    gl_matrix[2] = T.at<float>(2,0); gl_matrix[6] = T.at<float>(2,1); gl_matrix[10] = T.at<float>(2,2); gl_matrix[14] = T.at<float>(2,3);
    gl_matrix[3] = T.at<float>(3,0); gl_matrix[7] = T.at<float>(3,1); gl_matrix[11] = T.at<float>(3,2); gl_matrix[15] = T.at<float>(3,3);


}

void Render::getDepthImg()
{
    int64 time0 = cv::getTickCount();
    glReadPixels(0,0,m_width,m_height,GL_DEPTH_COMPONENT,GL_FLOAT,m_buffer_depth);

    int64 time1 = cv::getTickCount();
    //printf("read pixel time:%f\n",(time1-time0)/cv::getTickFrequency());
}

void Render::getDepthImg(const cv::Point& p1, const cv::Point& p2)
{
    int width = p2.x - p1.x;
    int height = p2.y - p1.y ;

    float* renderImg = new float[width*height];

    //int64 time0 = cv::getTickCount();
    glReadPixels(p1.x,m_height-p2.y,width,height,GL_DEPTH_COMPONENT,GL_FLOAT,renderImg);  //Õâ¸öÌ«ºÄÊ±ÁË£¬ÔõÃ´°ì£¿Ò»´Î½ü0.1s
    //glReadPixels(1,1,202,202,GL_RGB,GL_UNSIGNED_BYTE,renderImg);
    //int64 time1 = cv::getTickCount();
    //printf("read image buffer:%f\n",(time1-time0)/cv::getTickFrequency());
    for(int i=0; i<m_height; i++)
    {
        for(int j=0; j<m_width; j++)
        {
            m_buffer_depth[i*m_width+j] = 1;
        }
    }


    for(int i=0; i<width-1; i++)
    {
        for(int j=0; j<height-1; j++)
        {
            m_buffer_depth[(m_height-p2.y+j)*m_width+p1.x+i] = renderImg[j*width+i];
        }
    }


    delete renderImg;

}

cv::Point3f Render::get3DPos(int x, int y)
{

    GLint viewport[4];
    GLdouble modelview[16];
    GLdouble projection[16];
    GLfloat winX, winY, winZ;
    GLdouble posX, posY, posZ;
//#pragma omp critical
//	{
    glGetDoublev( GL_MODELVIEW_MATRIX, modelview );
    glGetDoublev( GL_PROJECTION_MATRIX, projection );
    glGetIntegerv( GL_VIEWPORT, viewport );

    winX = (float)x;
    winY = (float)viewport[3] - (float)y - 1;

    //glReadPixels( x, int(winY), 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &winZ );
    winZ = m_buffer_depth[(int)winY*m_width+x];

    //printf("wx=%f      wy=%f         wz=%f\n",winX,winY,winZ);
    gluUnProject( winX, winY, winZ, modelview, projection, viewport, &posX, &posY, &posZ);
    //printf("...x=%d  y=%d...px=%.2lf...py=%.2lf...pz=%.2lf\n",x,y,posX,posY,posZ);
//	}

    return cv::Point3f(posX,posY,posZ);


}

cv::Mat Render::getRenderedImg()
{
    glReadPixels(0,0,m_width,m_height,GL_RGB,GL_UNSIGNED_BYTE,m_renderImg);  //Õâ¸öÌ«ºÄÊ±ÁË£¬ÔõÃ´°ì£¿Ò»´Î½ü0.1s
    static cv::Mat bufferImg(m_height,m_width,CV_8UC1,cv::Scalar(0,0,0));
    //bufferÀïÃæµÄÍ¼Ïñ±ä³ÉopencvÀïÃæµÄÍ¼Ïñ
    for(int i=0; i<m_width; i++)
    {
        for(int j=0; j<m_height; j++)
        {

            bufferImg.at<uchar>(m_height-j-1,i) = m_renderImg[j*m_width*3+3*i];
            //bufferImg.at<uchar>(m_height-j-1,i) = m_renderImg[j*m_width+i];
        }
    }

    /*for(int i=0; i<400; i++)
    {
        for(int j=0; j<200; j++)
        {
            bufferImg.at<uchar>(340-j-1,200+i) = m_renderImg[j*400+i];
        }
    }*/

    return bufferImg;
}

cv::Mat Render::getRenderedImg(const cv::Point p1, const cv::Point p2)
{
    static cv::Mat bufferImg(m_height,m_width,CV_8UC1,cv::Scalar(0,0,0));
    int width = p2.x - p1.x;
    int height = p2.y - p1.y ;

    uchar* renderImg = new uchar[width*height*4];

    //int64 time0 = cv::getTickCount();
    glReadPixels(p1.x,m_height-p2.y,width,height,GL_RGBA,GL_UNSIGNED_BYTE,renderImg);  //Õâ¸öÌ«ºÄÊ±ÁË£¬ÔõÃ´°ì£¿Ò»´Î½ü0.1s
    //glReadPixels(1,1,202,202,GL_RGB,GL_UNSIGNED_BYTE,renderImg);
    //int64 time1 = cv::getTickCount();
    //printf("read image buffer:%f\n",(time1-time0)/cv::getTickFrequency());
    for(int i=0; i<m_width; i++)
    {
        for(int j=0; j<m_height; j++)
        {
            bufferImg.at<uchar>(j,i) = 0;
        }
    }

    for(int i=0; i<width-1; i++)
    {
        for(int j=0; j<height-1; j++)
        {
            bufferImg.at<uchar>(p2.y-j-1,p1.x+i) = renderImg[j*(width)*4+4*i];

        }
    }


    delete renderImg;
    return bufferImg;
}

void Render::buildProjectionMatrix(const cv::Mat& calibration, GLfloat* projectionMatrix)
{
    int screen_width = m_width;
    int screen_height = m_height;
    float nearPlane = 0.01f;  // Near clipping distance
    float farPlane  = 1000.0f;  // Far clipping distance


    // Camera parameters
    float f_x = calibration.at<float>(0,0); // Focal length in x axis
    float f_y = calibration.at<float>(1,1); // Focal length in y axis (usually the same?)
    float c_x = calibration.at<float>(0,2); // Camera primary point x
    float c_y = calibration.at<float>(1,2); // Camera primary point y


    projectionMatrix[0] = 2.0f * f_x / screen_width;
    projectionMatrix[1] = 0.0f;
    projectionMatrix[2] = 0.0f;
    projectionMatrix[3] = 0.0f;


    projectionMatrix[4] = 0.0f;
    projectionMatrix[5] = 2.0f * f_y / screen_height;
    projectionMatrix[6] = 0.0f;
    projectionMatrix[7] = 0.0f;


    projectionMatrix[8] = 2.0f * c_x / screen_width - 1.0f;
    projectionMatrix[9] = 2.0f * c_y / screen_height - 1.0f;
    projectionMatrix[10] = -( farPlane + nearPlane) / ( farPlane - nearPlane );
    projectionMatrix[11] = -1.0f;


    projectionMatrix[12] = 0.0f;
    projectionMatrix[13] = 0.0f;
    projectionMatrix[14] = -2.0f * farPlane * nearPlane / ( farPlane - nearPlane );
    projectionMatrix[15] = 0.0f;
}