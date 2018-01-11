//
// Created by qqh on 18-1-11.
//
#ifndef ROBOT_MODEL_H
#define ROBOT_MODEL_H

#include <iostream>
#include <opencv2/core.hpp>
#include "./../ThirdParty/glm/glm.h"

class Pose;
class Model {
public:
    Model();
    Model(std::string model_file);
    ~Model();

    void Load(const std::string& filename);
    int VerticesCount() {return model_->numvertices;}
    void GetVisualableVertices(Pose& pose, cv::Mat& vis_vertices);

    GLMmodel* model_;
    cv::Mat vertices_hom_;
};
#endif //ROBOT_MODEL_H
