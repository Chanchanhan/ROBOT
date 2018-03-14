//
// Created by qqh on 18-1-16.
//

#include "Solver.h"
#include "Model.h"
#include <glog/logging.h>
using namespace Sophus;



CeresSolver::CeresSolver() {
    //options.line_search_direction_type = ceres::LBFGS;
    // options.linear_solver_type = ceres::CGNR;
    options.minimizer_type = ceres::TRUST_REGION;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    options.minimizer_progress_to_stdout = false;
    options.max_num_iterations = 25;
    options.gradient_tolerance = 1e-15;
    options.function_tolerance = 1e-15;


}
CeresSolver::~CeresSolver() {}
void CeresSolver::SolveByCostFunctionWithJac(Model &model, FramePtr cur_frame){
    Vector6d pose_initial = cur_frame->m_pose.log();
    double* pose_var = pose_initial.data();
    assert(pose_var);
//    LOG(INFO)<<"pose_var input:";
//    for(int i=0;i<6;i++){
//        LOG(INFO)<<pose_var[i];
//    }
    Sophus::Matrix3d KK;

    ceres::Problem minEnergyProblem;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            KK(i,j) = model.intrinsic.at<float>(i,j);
        }
    }

    for (int i = 0; i < cur_frame->VerticesNear2ContourX3D.size(); i++) {
        auto Xi = cur_frame->VerticesNear2ContourX3D[i];
        minEnergyProblem.AddResidualBlock( new CostFunctionByJac(Xi,
                                                          cur_frame->fw_posterior,
                                                          cur_frame->bg_posterior,
                                                          cur_frame->dt,
                                                          KK)
                , NULL, pose_var);

    }
    ceres::Solver::Summary summary;


    ceres::Solve(options, &minEnergyProblem, &summary);



    LOG(INFO)<<summary.BriefReport();
    LOG(INFO)<<"pose_var outPut:";
//    for(int i=0;i<6;i++){
//        LOG(INFO)<<pose_var[i];
//    }

}
