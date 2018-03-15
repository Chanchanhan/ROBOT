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
    struct Option{
        int max_iterations;
        double lamda;
        double lamdaSmaller;
        double energyOK;
        double energyLittle;
        double He_b;
        int max_wrong_point;
        double energyTooSmallSize;
        double stepTooBigSize;

    };
public:
    MySolver(){}
    MySolver(cv::Mat &intrinsic);
    void Solve(FramePtr cur_frame, const int &iLevel = 0);

private:
    bool Evaluate(const FramePtr cur_frame,const cv::Point3d &X_,
                  double &energy,
                  Sophus::Vector6d &jac,bool &judge) const ;

    void ComputeEnergy(const FramePtr cur_frame,const std::vector<cv::Point3d> &Xs, double &energySum,int &wrongPointCnt);

    bool ComputeEnergy(const FramePtr cur_frame,const cv::Point3d &X_, double &energy);
private:
    Option option;

    Sophus::Matrix3d K_;
};


#endif //ROBOT_MYSOLVER_H
