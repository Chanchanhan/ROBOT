//
// Created by qqh on 18-1-11.
//
#ifndef ROBOT_MODEL_H
#define ROBOT_MODEL_H

#include <iostream>
#include <opencv2/core.hpp>
#include "./../ThirdParty/glm/glm.h"
#include "Frame.h"

class Pose;
class Model {
public:
    Model();
    Model(std::string model_file);
    ~Model();

    void loadObj(const std::string& filename);
    void setIntrinsic();
    //cv part
    void getContourPointsAndIts3DPoints( Pose &pose,std::vector<cv::Point3d> &verticesContour_Xs,std::vector<cv::Point2d> &verticesContour_xs,std::vector<cv::Point> &resContour);
    std::vector<cv::Point> GetContourAt(Pose &pose);
    void SampleVertex(FramePtr frame,std::vector<cv::Point3d> &verticesContour_Xs,std::vector<cv::Point2d> &verticesContour_xs);

    void displayCV( Pose &pose,const cv::Scalar &color, cv::Mat& frame);
    cv::Mat intrinsic;

private:
    void getVisualableVertices( Pose& pose, cv::Mat& vis_vertices);
    void project3D_2D( Pose &pose, const cv::Mat& visible_Xs,  cv::Mat &visible_xs);
    bool pointInFrame(const cv::Point &p);
    cv::Point X_to_x(const cv::Point3f &X,const cv::Mat &extrisic);
    int VerticesCount() {return model_->numvertices;}
private:
    inline void Cross(float* u, float* v, float* n) {
      n[0] = u[1] * v[2] - u[2] * v[1];
      n[1] = u[2] * v[0] - u[0] * v[2];
      n[2] = u[0] * v[1] - u[1] * v[0];
  }

    inline void Normalize(float* v) {
	float l = (float)sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
	v[0] /= l, v[1] /= l, v[2] /= l;
    }
    GLMmodel* model_;
    cv::Mat vertices_hom_;
};
#endif //ROBOT_MODEL_H
