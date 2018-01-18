//
// Created by qqh on 18-1-16.
//

#include "Solver.h"
#include "Model.h"
#include <opencv2/core/eigen.hpp>
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
    auto so3_ = last_frame->m_pose.m_pose.so3().log();
    Sophus::Vector3d& t3_ = last_frame->m_pose.m_pose.translation();

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
    cur_frame->m_pose = Pose(pose_initial);

}
void CeresSolver::SolveByCostFunctionWithJac(Model &model, FramePtr cur_frame, FramePtr last_frame){
    auto so3_ = last_frame->m_pose.m_pose.so3().log();
    Sophus::Vector3d& t3_ = last_frame->m_pose.m_pose.translation();

    double pose_initial[6] = {so3_(0),so3_(1),so3_(2),t3_(0),t3_(1),t3_(2)};
    double* pose_var = pose_initial;
    assert(pose_var);

    Sophus::Matrix3d KK;
//    cv::cv2eigen(model.intrinsic,KK);

    ceres::Problem min_enery;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            KK(i,j) = model.intrinsic.at<float>(i,j);
        }
    }
//    std::cout<<pose_var<<std::endl;
    for (int ii = 0; ii < 25; ++ii) {

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
        std::cout<<r_sum<<std::endl;


    //    ceres::Solver::Summary summary;


    //    ceres::Solve(options, &min_enery, &summary);
//        std::cout<<cur_frame->m_pose.m_pose.log()<<std::endl;

        cur_frame->m_pose.m_pose = Sophus::SE3d::exp(5*jtjsum.inverse()*j) * cur_frame->m_pose.m_pose;
        auto so3_t = cur_frame->m_pose.m_pose.so3().log();
        Sophus::Vector3d& t3_t = cur_frame->m_pose.m_pose.translation();

        double pose_initial[6] = {so3_t(0),so3_t(1),so3_t(2),t3_t(0),t3_t(1),t3_t(2)};
        pose_var = pose_initial;
    }
    std::cout<<"++++++++++++++"<<std::endl;
}
