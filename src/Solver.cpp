//
// Created by qqh on 18-1-16.
//

#include "Solver.h"
#include "Model.h"
#include <opencv2/core/eigen.hpp>
#include <glog/logging.h>
using namespace Sophus;

class CostFunctionByJac : public ceres::SizedCostFunction<1, 6> {

public:
    CostFunctionByJac(cv::Point3d& X, cv::Mat &fwd, cv::Mat &bg, cv::Mat &dt_map, Sophus::Matrix3d &K) :
            X_(X), fwd_(fwd), bg_(bg), dt_map_(dt_map), K_(K) {}

    virtual ~CostFunctionByJac() {}

    virtual bool Evaluate(double const * const *parameters,
                          double *residuals,
                          double **jacobians) const {

        if (!jacobians) return true;
        double *jacobian = jacobians[0];
        if (!jacobian) return true;
        const double *pose = parameters[0];
        Vector6d logpose;
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
        auto Thetax = (double) (dt_map_.at<float>(x_plane));

//            auto sigmoid = [](double x){1.0/(1.0+std::exp(-Thetax))};
//            auto He = (1.0) / ((1) + ceres::exp(-Thetax));
        const int b=8;
        double He = M_1_PI*(-atan(b*Thetax)+M_PI_2);
//        if(Thetax>8)
//            He = 0;
//        else if(Thetax>-8)
//            He = 0.5;
//        else
//            He = 1;

        double left = (abs(Thetax) <= 8.0f) * (fwd_.at<double>(x_plane) - bg_.at<double>(x_plane)) /
                      (He * fwd_.at<double>(x_plane) + (1 - He) * bg_.at<double>(x_plane));


        Eigen::MatrixXd j_X_Lie(2, 6);
        Eigen::MatrixXd j_Phi_x(1, 2);

        Config &gConfig = Config::configInstance();
        double _x_in_Camera = X_Camera_coord[0];
        double _y_in_Camera = X_Camera_coord[1];
        double _z_in_Camera = X_Camera_coord[2];
        if(!fabs((He * fwd_.at<double>(x_plane) + (1 - He) * bg_.at<double>(x_plane))>0.1)) {
            _x_in_Camera = 1;
            std::cout<<x_plane<<std::endl;
        }
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
        j_X_Lie(1, 3) = gConfig.FY * (1 + _y_in_Camera * _y_in_Camera * (1 / (_z_in_Camera * _z_in_Camera)));
        j_X_Lie(1, 4) = -gConfig.FY * _x_in_Camera * _y_in_Camera / (_z_in_Camera * _z_in_Camera);
        j_X_Lie(1, 5) = -gConfig.FY * _x_in_Camera / _z_in_Camera;

        j_Phi_x(0, 0) = 0.5f * (dt_map_.at<float>(cv::Point(x_plane.x + 1, x_plane.y)) -
                                dt_map_.at<float>(cv::Point(x_plane.x - 1, x_plane.y)));
        j_Phi_x(0, 1) = 0.5f * (dt_map_.at<float>(cv::Point(x_plane.x, x_plane.y + 1)) -
                                dt_map_.at<float>(cv::Point(x_plane.x, x_plane.y - 1)));
        Eigen::MatrixXd jac = left * j_Phi_x * j_X_Lie;

        for (int i = 0; i < 6; i++) {
            jacobian[i] = jac(0, i);
        }
        residuals[0] = (-ceres::log(He * fwd_.at<double>(x_plane) + (1 - He) * bg_.at<double>(x_plane))) ;
        return true;
    }

    const cv::Point3d X_;
    const cv::Mat dt_map_;
    const cv::Mat fwd_;
    const cv::Mat bg_;
    const Sophus::Matrix3d K_;
};


CeresSolver::CeresSolver() {
    //options.line_search_direction_type = ceres::LBFGS;
    options.minimizer_type = ceres::TRUST_REGION;
   // options.linear_solver_type = ceres::CGNR;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 25;

}
CeresSolver::~CeresSolver() {}
void CeresSolver::SolveByNumericDiffCostFunction(Model& model, FramePtr cur_frame,FramePtr last_frame){
    auto so3_ = last_frame->m_pose.so3().log();
    Sophus::Vector3d& t3_ = last_frame->m_pose.translation();

    double pose_initial[6] = {so3_(0),so3_(1),so3_(2),t3_(0),t3_(1),t3_(2)};

    ceres::Problem min_enery;
    Sophus::Matrix3d KK;

//  LOG(INFO)<<"KK = "<<KK<<std::endl;
    ceres::CostFunction* cost_function =
            new ceres::NumericDiffCostFunction<NumericDiffCostFunctor, ceres::RIDDERS,1, 6>(new NumericDiffCostFunctor(
                    cur_frame->VerticesNear2ContourX3D[0],
                    cur_frame->fw_posterior,
                    cur_frame->bg_posterior,
                    cur_frame->dt,
                    KK
            ));
    min_enery.AddResidualBlock(cost_function, NULL, pose_initial);
    cur_frame->m_pose = Data2Pose(pose_initial);

}
void CeresSolver::SolveByCostFunctionWithJac(Model &model, FramePtr cur_frame){

    Vector6d pose_initial = cur_frame->m_pose.log();

    double* pose_var = pose_initial.data();
    assert(pose_var);

    Sophus::Matrix3d KK;
//    cv::cv2eigen(model.intrinsic,KK);

    ceres::Problem min_enery;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            KK(i,j) = model.intrinsic.at<float>(i,j);
        }
    }

    double last = 1000000;
//    std::cout<<pose_var<<std::endl;
    for (int ii = 0; ii < 1; ++ii) {

        double r_sum = 0;
    //    double* jacobians =  new double[6];
        double jacobians_sum[6];
        Sophus::Vector6d jacobians;
        Sophus::Matrix6d jtjsum;
        Sophus::Vector6d j;
        for (int k = 0; k < 6; ++k) {
            for (int i = 0; i < 6; ++i) {
                jtjsum(k,i) = 0;
            }
            j(k) = 0;
        }

        for(int i=0;i<cur_frame->VerticesNear2ContourX3D.size();i++){
            auto Xi = cur_frame->VerticesNear2ContourX3D[i];
            ceres::CostFunction* cost_function =
                    new CostFunctionByJac( Xi,
                                        cur_frame->fw_posterior,
                                        cur_frame->bg_posterior,
                                        cur_frame->dt,
                                        KK) ;
            min_enery.AddResidualBlock(cost_function, NULL, pose_var);
            double r;
    //        std::cout<<jacobians<<std::endl;

            double* sj = jacobians.data();
            const double* const* pose1 = (const double* const*)(&pose_var);
            cost_function->Evaluate(pose1,&r,&sj);
            jtjsum +=jacobians*jacobians.transpose();
            j+=jacobians;
            r_sum += r;
        }
        std::cout<<r_sum/cur_frame->VerticesNear2ContourX3D.size()<<std::endl;
//        std::cout<<cur_frame->VerticesNear2ContourX3D.size()<<std::endl;
//        if(r_sum>last)
//            break;
        last = r_sum;
        LOG(INFO)<<"r_sum"<<r_sum;
        ceres::Solver::Summary summary;


        ceres::Solve(options, &min_enery, &summary);
        std::cout<<cur_frame->m_pose.log()<<std::endl;
        Sophus::Vector6d v6d =  jtjsum.inverse()*j;
//        std::cout<<v6d<<std::endl;
        cur_frame->m_pose = Sophus::SE3d::exp(v6d) * cur_frame->m_pose;

        pose_var = new double[6]{v6d(0),v6d(1),v6d(2),v6d(3),v6d(4),v6d(5)};
    }
    std::cout<<"++++++++++++++"<<std::endl;
}
