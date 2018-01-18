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
                    cur_frame->VerticesNear2ContourX3D,
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

    ceres::Problem min_enery;
    Sophus::Matrix3d KK;
    cv::cv2eigen(model.intrinsic,KK);

    for(auto Xi:cur_frame->VerticesNear2ContourX3D){
        ceres::CostFunction* cost_function = new CostFunctionByJac( Xi,
                                                                    cur_frame->fw_posterior,
                                                                    cur_frame->bg_posterior,
                                                                    cur_frame->dt,
                                                                    KK) ;
        min_enery.AddResidualBlock(cost_function, NULL, pose_initial);

    }
    ceres::Solver::Summary summary;

    ceres::Solve(options, &min_enery, &summary);

    cur_frame->m_pose = Pose(pose_initial);


}
