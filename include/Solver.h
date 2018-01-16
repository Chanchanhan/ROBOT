//
// Created by qqh on 18-1-16.
//

#ifndef ROBOT_SOLVER_H
#define ROBOT_SOLVER_H

#include <opencv2/highgui.hpp>
#include <sophus/se3.hpp>
#include <ceres/ceres.h>
#include "Frame.h"
class Model;
class Pose;


class CeresSolver{
    struct NumericDiffCostFunctor
    {

        NumericDiffCostFunctor(std::vector<cv::Point3d>& X,cv::Mat& fwd,cv::Mat& bg,cv::Mat& dt_map,Sophus::Matrix3d& K):X_(X),fwd_(fwd),bg_(bg),dt_map_(dt_map),K_(K) {}
        bool operator()(const double* pose,double* residual) const
        {
            double E = 0;
            auto m_pose= Sophus::SE3d(Sophus::SO3d::exp(Sophus::Vector3d(pose[0], pose[1],pose[2])),Sophus::Vector3d(pose[3],pose[4],pose[5]));
            for(auto Xi:X_)
            {
                Sophus::Vector3d Xis;
                Xis[0] = Xi.x;
                Xis[1] = Xi.y;
                Xis[2] = Xi.z;
                Xis = m_pose*Xis;
                auto x3 = K_*Xis;
                cv::Point x(x3(0)/x3(2),x3(1)/x3(2));
                auto Thetax = (double)(dt_map_.at<float>(x));
                auto He = (1.0)/((1) + ceres::exp(-Thetax));
                E+=ceres::log(He*fwd_.at<double>(x)+(1-He)*bg_.at<double>(x));
            }
            residual[0] = E;//ceres::exp((table[int(x[0]*10000)]))+ceres::exp(-x[1]);
            return true;
        }

        const std::vector<cv::Point3d> X_;
        const cv::Mat dt_map_;
        const cv::Mat fwd_;
        const cv::Mat bg_;
        const Sophus::Matrix3d K_;
    };
public:
    CeresSolver();
    ~CeresSolver();
    void SolveByNumericDiffCostFunction(Model& model, Pose& pose, FramePtr cur_frame,FramePtr last_frame , float& curEFValue );


private:
    ceres::Solver::Options options;
};
#endif //ROBOT_SOLVER_H
