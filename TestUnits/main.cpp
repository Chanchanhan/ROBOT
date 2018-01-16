#include <iostream>
#include "Tracker.h"
#include <glog/logging.h>
#include <ceres/ceres.h>

using namespace std;
using namespace cv;


int main(int argc, char* argv[])
{
    google::InitGoogleLogging(argv[0]);
    if (argc < 2) {
        LOG(ERROR) << "Not using  ROBOTConfig.yaml \n";
        return -1;
    }
    OcvYamlConfig ocvYamlConfig(argv[1]);
    auto dataPaser = std::make_unique<DataPaser>(ocvYamlConfig);
    Tracker tk;
    tk.init(ocvYamlConfig);
    bool initialized = false;
    while(1)
    {
        FramePtr cur_frame(new Frame);
        if(!dataPaser->parseAFrame(cur_frame))
            break;
        if(!initialized)
            cur_frame->m_pose = cur_frame->gt_Pose;
        tk.ProcessFrame(cur_frame);
    }
	return 0;
}

//struct CostFunctor
//{
//    CostFunctor()
//    {
//        for (int i = 0; i < 5000; ++i) {
//            table[i] = -i/10.0;
//        }
//    }
//    bool operator()(const double* x,double* residual) const
//    {
//        residual[0] = ceres::exp((table[int(x[0]*10000)]))+ceres::exp(-x[1]);
//        return true;
//    }
//
//private:
//    double table[5000];
//};
//
//int main(int argc,char** argv)
//{
//    google::InitGoogleLogging(argv[0]);
//
//    // 指定求解的未知数的初值，这里设置为5.0
//    double initial_x = 5e-5;
//    double x1[2] = { initial_x ,initial_x };
//
//    // 建立Problem
//    ceres::Problem problem;
//
//    // 建立CostFunction（残差方程）
//    ceres::CostFunction* cost_function =
//            new ceres::NumericDiffCostFunction<CostFunctor, ceres::RIDDERS, 1, 2>(new CostFunctor);
//    problem.AddResidualBlock(cost_function, NULL, x1);
//
//    // 求解方程!
//    ceres::Solver::Options options;
//    options.minimizer_type = ceres::LINE_SEARCH;
//    options.linear_solver_type = ceres::DENSE_SCHUR;
//    options.minimizer_progress_to_stdout = true;
//    ceres::Solver::Summary summary;
//    ceres::Solve(options, &problem, &summary);
//
//    std::cout << summary.BriefReport() << "\n";
//    std::cout << "x1 : " << initial_x
//              << " -> " << x1[0] << "\n";
//    std::cout << "x2 : " << initial_x
//              << " -> " << x1[1] << "\n";
//    return 0;
//}