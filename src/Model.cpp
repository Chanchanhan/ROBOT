//
// Created by qqh on 18-1-11.
//

#include "Model.h"
#include <glog/logging.h>
Model::Model() {
    model_ = NULL;
}

Model::Model(std::string model_file) {
    model_ = NULL;
    Load(model_file);
}

Model::~Model() {
    if(model_)
        glmDelete(model_);
}

void Model::Load(const std::string& filename) {
    if (model_)
        glmDelete(model_);

    model_ = glmReadOBJ(const_cast<char*>(filename.c_str()));
    CHECK(model_) << "failed to load model";

    vertices_hom_ = cv::Mat::zeros(4, VerticesCount()+1, CV_32FC1);
    for (int i = 0; i <= VerticesCount(); ++i) {
        vertices_hom_.at<float>(0, i) = model_->vertices[3 * (i)+0];
        vertices_hom_.at<float>(1, i) = model_->vertices[3 * (i)+1];
        vertices_hom_.at<float>(2, i) = model_->vertices[3 * (i)+2];
        vertices_hom_.at<float>(3, i) = 1;
    }
}