//
// Created by qqh on 18-1-16.
//

#ifndef ROBOT_SOLVER_H
#define ROBOT_SOLVER_H

#include <opencv2/highgui.hpp>
#include <sophus/se3.hpp>
#include <ceres/ceres.h>
#include <opencv2/core/eigen.hpp>
#include "Frame.h"
#include "GlobalConfig.h"

class Model;

class CeresSolver {
    class CostFunctionByJac : public ceres::SizedCostFunction<1, 6> {
    public:
        CostFunctionByJac(cv::Point3d& X, cv::Mat &fwd, cv::Mat &bg, cv::Mat &dt_map, Sophus::Matrix3d &K) :
                X_(X), fwd_(fwd), bg_(bg), dt_map_(dt_map), K_(K) {

        }

        virtual ~CostFunctionByJac() {}

        virtual bool Evaluate(double const * const *parameters,
                              double *residuals,
                              double **jacobians) const {

            if (!jacobians) return true;
            double *jacobian = jacobians[0];
            if (!jacobian) return true;
            const double *pose = parameters[0];
            Sophus::Vector6d logpose;
            logpose<<pose[0], pose[1], pose[2],pose[3], pose[4], pose[5];
            auto m_pose = Sophus::SE3d::exp(logpose);

            double E = 0;
            Sophus::Vector3d Xis;
            Xis[0] = X_.x;
            Xis[1] = X_.y;
            Xis[2] = X_.z;

            Sophus::Vector3d X_Camera_coord = m_pose * Xis;
            Sophus::Vector3d x3 = K_ * X_Camera_coord;
            cv::Point x_plane(x3(0) / x3(2), x3(1) / x3(2));
            auto Theta_x = (double) (dt_map_.at<float>(x_plane));

//            auto sigmoid = [](double x){1.0/(1.0+std::exp(-Theta_x))};
//            auto He = (1.0) / ((1) + ceres::exp(-Theta_x));
            const int b=Config::configInstance().SV_HE_b;
            double He = M_1_PI*(-atan(b*Theta_x)+M_PI_2);
            double phi = -M_1_PI*b/(1+b*b*Theta_x*Theta_x);
//        if(Theta_x>8)
//            He = 0;
//        else if(Theta_x>-8)
//            He = 0.5;
//        else
//            He = 1;
            double left =  phi* (fwd_.at<double>(x_plane) - bg_.at<double>(x_plane)) /
                           (He * fwd_.at<double>(x_plane) + (1 - He) * bg_.at<double>(x_plane));


            Eigen::MatrixXd j_X_Lie(2, 6);
            Eigen::MatrixXd j_Phi_x(1, 2);

            Config &gConfig = Config::configInstance();
            double _x_in_Camera = X_Camera_coord[0];
            double _y_in_Camera = X_Camera_coord[1];
            double _z_in_Camera = X_Camera_coord[2];

//        assert(fabs((He * fwd_.at<double>(x_plane) + (1 - He) * bg_.at<double>(x_plane)))>0.001);


            j_X_Lie(0, 0) = -gConfig.FX / _z_in_Camera;
            j_X_Lie(0, 1) = 0;
            j_X_Lie(0, 2) = gConfig.FX * _x_in_Camera / (_z_in_Camera * _z_in_Camera);
            j_X_Lie(0, 3) = gConfig.FX * _x_in_Camera * _y_in_Camera / (_z_in_Camera * _z_in_Camera);
            j_X_Lie(0, 4) = -gConfig.FX * (1 + _x_in_Camera * _x_in_Camera / (_z_in_Camera * _z_in_Camera));
            j_X_Lie(0, 5) = gConfig.FX * _y_in_Camera / _z_in_Camera;

            j_X_Lie(1, 0) = 0;
            j_X_Lie(1, 1) = -gConfig.FY / _z_in_Camera;
            j_X_Lie(1, 2) = gConfig.FY * _y_in_Camera / (_z_in_Camera * _z_in_Camera);
            j_X_Lie(1, 3) = gConfig.FY * (1 + _y_in_Camera * _y_in_Camera / (_z_in_Camera * _z_in_Camera));
            j_X_Lie(1, 4) = -gConfig.FY * _x_in_Camera * _y_in_Camera / (_z_in_Camera * _z_in_Camera);
            j_X_Lie(1, 5) = -gConfig.FY * _x_in_Camera / _z_in_Camera;

            j_Phi_x(0, 0) = 0.5f * (dt_map_.at<float>(cv::Point(x_plane.x + 1, x_plane.y)) -
                                    dt_map_.at<float>(cv::Point(x_plane.x - 1, x_plane.y)));
            j_Phi_x(0, 1) = 0.5f * (dt_map_.at<float>(cv::Point(x_plane.x, x_plane.y + 1)) -
                                    dt_map_.at<float>(cv::Point(x_plane.x, x_plane.y - 1)));
            Eigen::MatrixXd jac = left * j_Phi_x * j_X_Lie;
            LOG(INFO)<<"left = "<<left;
            LOG(INFO)<<"j_Phi_x = "<<j_Phi_x;
            LOG(INFO)<<"j_X_Lie = "<<j_X_Lie;
            for (int i = 0; i < 6; i++) {
                jacobian[i] = jac(0, i);
            }
            residuals[0] = (-ceres::log(He * fwd_.at<double>(x_plane) + (1 - He) * bg_.at<double>(x_plane))) ;
            //  LOG(INFO)<<"jac = "<<jac;
            // LOG(INFO)<<"residuals[0] = " <<residuals[0];
            return true;
        }

        const cv::Point3d X_;
        const cv::Mat dt_map_;
        const cv::Mat fwd_;
        const cv::Mat bg_;
        const Sophus::Matrix3d K_;
    };
public:
    CeresSolver();

    ~CeresSolver();

//    void SolveByNumericDiffCostFunction(Model &model, FramePtr cur_frame, FramePtr last_frame);

    void SolveByCostFunctionWithJac(Model &model, FramePtr cur_frame);


private:
    ceres::Solver::Options options;
};

#endif //ROBOT_SOLVER_H
