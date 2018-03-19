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
    Model *model;

private:
    bool Evaluate(const FramePtr cur_frame,const cv::Point3d &X_,
                  double &energy,
                  Sophus::Vector6d &jac,bool &judge) const ;

    void ComputeEnergy(const FramePtr cur_frame,const std::vector<cv::Point3d> &Xs, double &energySum,int &wrongPointCnt,const bool _debug=0);
    void ComputeEnergyAndDraw(const FramePtr cur_frame,const std::vector<cv::Point3d> &Xs,
                              double &energySum,int &wrongPointCnt,const bool _debug=0,const std::string owner="",const int iLevel=0);

    bool ComputeEnergy(const FramePtr cur_frame,const cv::Point3d &X_, double &energy,const bool _debug=0);
private:
    Option option;
    std::vector<unsigned char> pointStateTmp;
    int k_th_tmp;
    Sophus::Matrix3d K_;
};


#endif //ROBOT_MYSOLVER_H
