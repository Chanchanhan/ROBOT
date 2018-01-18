//
// Created by qqh on 18-1-16.
//

#ifndef ROBOT_SOLVER_H
#define ROBOT_SOLVER_H

#include <opencv2/highgui.hpp>
#include <sophus/se3.hpp>
#include <ceres/ceres.h>
#include "Frame.h"
#include "GlobalConfig.h"

class Model;

class Pose;


class CeresSolver {
    struct NumericDiffCostFunctor {

        NumericDiffCostFunctor(std::vector<cv::Point3d> &X, cv::Mat &fwd, cv::Mat &bg, cv::Mat &dt_map,
                               Sophus::Matrix3d &K) :
                X_(X), fwd_(fwd), bg_(bg), dt_map_(dt_map), K_(K) {}

        bool operator()(const double *pose, double *residual) const {
            double E = 0;
            auto m_pose = Sophus::SE3d(Sophus::SO3d::exp(Sophus::Vector3d(pose[0], pose[1], pose[2])),
                                       Sophus::Vector3d(pose[3], pose[4], pose[5]));
            for (auto Xi:X_) {

                Sophus::Vector3d Xis;
                Xis[0] = Xi.x;
                Xis[1] = Xi.y;
                Xis[2] = Xi.z;
                Xis = m_pose * Xis;
                auto x3 = K_ * Xis;
                cv::Point x_plane(x3(0) / x3(2), x3(1) / x3(2));
                auto Thetax = (double) (dt_map_.at<float>(x_plane));
                auto He = (1.0) / ((1) + ceres::exp(-Thetax));


                E += ceres::log(He * fwd_.at<double>(x_plane) + (1 - He) * bg_.at<double>(x_plane));
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

    class CostFunctionByJac : public ceres::SizedCostFunction<1, 6> {

    public:
        CostFunctionByJac(cv::Point3d X, cv::Mat &fwd, cv::Mat &bg, cv::Mat &dt_map, Sophus::Matrix3d &K) :
                X_(X), fwd_(fwd), bg_(bg), dt_map_(dt_map), K_(K) {}

        virtual ~CostFunctionByJac() {}

        virtual bool Evaluate(double const *const *parameters,
                              double *residuals,
                              double **jacobians) const {
            if (!jacobians) return true;
            double *jacobian = jacobians[0];
            if (!jacobian) return true;
            const double *pose = parameters[0];
            auto m_pose = Sophus::SE3d(Sophus::SO3d::exp(Sophus::Vector3d(pose[0], pose[1], pose[2])),
                                       Sophus::Vector3d(pose[3], pose[4], pose[5]));
            double E = 0;
            Sophus::Vector3d Xis;
            Xis[0] = X_.x;
            Xis[1] = X_.y;
            Xis[2] = X_.z;

            Sophus::Vector3d X_Camera_coord = m_pose * Xis;
            auto x3 = K_ * X_Camera_coord;
            cv::Point x_plane(x3(0) / x3(2), x3(1) / x3(2));
            auto Thetax = (double) (dt_map_.at<float>(x_plane));

            auto He = (1.0) / ((1) + ceres::exp(-Thetax));

            double left = (abs(Thetax) <= 1.0f) * (fwd_.at<double>(x_plane) - bg_.at<double>(x_plane)) /
                          (He * fwd_.at<double>(x_plane) + (1 - He) * bg_.at<double>(x_plane));

            Eigen::MatrixXd j_X_Lie(2, 6);
            Eigen::MatrixXd j_Phi_x(1, 2);

            Config &gConfig = Config::configInstance();
            float _x_in_Camera = Xis[0];
            float _y_in_Camera = Xis[1];
            float _z_in_Camera = Xis[2];

            j_X_Lie(0, 0) = gConfig.FX / _z_in_Camera;
            j_X_Lie(0, 1) = 0;
            j_X_Lie(0, 2) = -gConfig.FX * _x_in_Camera / (_z_in_Camera * _z_in_Camera);
            j_X_Lie(0, 3) = -gConfig.FX * _x_in_Camera * _y_in_Camera / (_z_in_Camera * _z_in_Camera);
            j_X_Lie(0, 4) = gConfig.FX * (1 + _x_in_Camera * _x_in_Camera / (_z_in_Camera * _z_in_Camera));
            j_X_Lie(0, 5) = gConfig.FX * _y_in_Camera / _z_in_Camera;

            j_X_Lie(1, 0) = 0;
            j_X_Lie(1, 1) = gConfig.FY / _z_in_Camera;
            j_X_Lie(1, 2) = -gConfig.FY * _y_in_Camera / (_z_in_Camera * _z_in_Camera);
            j_X_Lie(1, 3) = -gConfig.FY * (1 + _y_in_Camera * _y_in_Camera * (1 / (_z_in_Camera * _z_in_Camera)));
            j_X_Lie(1, 4) = -gConfig.FY * _x_in_Camera * _y_in_Camera / (_z_in_Camera * _z_in_Camera);
            j_X_Lie(1, 5) = gConfig.FY * _x_in_Camera / _z_in_Camera;

            j_Phi_x(0, 0) = 0.5f * (dt_map_.at<float>(cv::Point(x_plane.x + 1, x_plane.y)) -
                                    dt_map_.at<float>(cv::Point(x_plane.x - 1, x_plane.y)));
            j_Phi_x(0, 1) = 0.5f * (dt_map_.at<float>(cv::Point(x_plane.x, x_plane.y + 1)) -
                                    dt_map_.at<float>(cv::Point(x_plane.x, x_plane.y - 1)));
            Eigen::MatrixXd jac = left * j_Phi_x * j_X_Lie;

            for (int i = 0; i < 6; i++) {
                jacobian[i] = jac(0, i);
            }
            residuals[0] = -ceres::log(He * fwd_.at<double>(x_plane) + (1 - He) * bg_.at<double>(x_plane));
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

    void SolveByNumericDiffCostFunction(Model &model, FramePtr cur_frame, FramePtr last_frame);

    void SolveByCostFunctionWithJac(Model &model, FramePtr cur_frame, FramePtr last_frame);


private:
    ceres::Solver::Options options;
};

#endif //ROBOT_SOLVER_H
