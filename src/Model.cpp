//
// Created by qqh on 18-1-11.
//
#include <glog/logging.h>
#include "Pose.h"
#include "Model.h"
#include "GlobalConfig.h"
#include "Render.h"
Model::Model() {
    model_ = NULL;

}

Model::Model(std::string model_file) {
    model_ = NULL;
    LoadObj(model_file);
}

Model::~Model() {
    if(model_)
        glmDelete(model_);
}
void Model::SetIntrinsic() {
    const Config &gConfig = Config::ConfigInstance();
    intrinsic=cv::Mat(3,4,CV_32FC1);
    intrinsic.at<float>(0,0)=gConfig.FX;    intrinsic.at<float>(0,1)=0;             intrinsic.at<float>(0,2)=gConfig.CX; intrinsic.at<float>(0,3)=0;
    intrinsic.at<float>(1,0)=0;             intrinsic.at<float>(1,1)=gConfig.FY;    intrinsic.at<float>(1,2)=gConfig.CY; intrinsic.at<float>(1,3)=0;
    intrinsic.at<float>(2,0)=0;             intrinsic.at<float>(2,1)=0;             intrinsic.at<float>(2,2)=1; intrinsic.at<float>(2,3)=0;
    intrinsics.resize(gConfig.IMG_PYR_NUMBER);
    intrinsics[0]=intrinsic.clone();
    for(int i=1;i<gConfig.IMG_PYR_NUMBER;i++){
        intrinsics[i]=cv::Mat(3,4,CV_32FC1);
        intrinsics[i].at<float>(0,0)=gConfig.FX*std::pow(0.5,i);    intrinsics[i].at<float>(0,1)=0;             intrinsics[i].at<float>(0,2)=gConfig.CX*std::pow(0.5,i); intrinsics[i].at<float>(0,3)=0;
        intrinsics[i].at<float>(1,0)=0;             intrinsics[i].at<float>(1,1)=gConfig.FY*std::pow(0.5,i);    intrinsics[i].at<float>(1,2)=gConfig.CY*std::pow(0.5,i); intrinsics[i].at<float>(1,3)=0;
        intrinsics[i].at<float>(2,0)=0;             intrinsics[i].at<float>(2,1)=0;             intrinsics[i].at<float>(2,2)=1; intrinsics[i].at<float>(2,3)=0;

    }
    char* renderName="render";
    render.init(intrinsic,
                Config::ConfigInstance().VIDEO_WIDTH,
                Config::ConfigInstance().VIDEO_HEIGHT,
                0,&renderName);

}

void Model::LoadObj(const std::string &filename) {

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
    SetIntrinsic();

}




