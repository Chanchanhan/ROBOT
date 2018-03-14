//
// Created by qqh on 18-3-14.
//

#ifndef ROBOT_MYSOLVER_H
#define ROBOT_MYSOLVER_H



#include <opencv2/highgui.hpp>
#include <sophus/se3.hpp>
#include <opencv2/core/eigen.hpp>
#include "Frame.h"
#include "GlobalConfig.h"

class Model;

class MySolver {
    struct option{

    };
public:
    MySolver(){}
    MySolver(cv::Mat &intrinsic){
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                K_(i,j) = intrinsic.at<float>(i,j);
            }
        }
    }
    void Solve(FramePtr cur_frame, const int &iLevel = 0);

private:
    bool Evaluate(const FramePtr cur_frame,const cv::Point3d &X_,
                  double &energy,
                  Sophus::Vector6d &jac) const ;

    void ComputeEnergy(const FramePtr cur_frame,const std::vector<cv::Point3d> &Xs, double &energySum);

    void ComputeEnergy(const FramePtr cur_frame,const cv::Point3d &X_, double &energy);
private:

    Sophus::Matrix3d K_;
};


#endif //ROBOT_MYSOLVER_H
