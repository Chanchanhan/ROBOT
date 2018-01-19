//
// Created by flamming on 2018/1/13.
//
#include <ceres/ceres.h>
#include <opencv2/core/eigen.hpp>
#include "Tracker.h"
#include "GlobalConfig.h"
#include "Region.h"

using namespace cv;
using namespace std;
using namespace ceres;


void Tracker::init(const OcvYamlConfig& ocvYamlConfig) {
    model_.loadObj(Config::configInstance().objFile);
}

void Tracker::ProcessFirstFrame(FramePtr cur_frame)
{
    cur_frame_ = cur_frame;
    cur_frame_->m_pose = cur_frame->gt_Pose;
    model_.getContourPointsAndIts3DPoints(
            cur_pose_,cur_frame_->VerticesNear2ContourX3D,
            cur_frame_->VerticesNear2ContourX2D,
            cur_frame_->contourX2D);
    cur_frame_->Segment();
    cur_frame_->DTMap();

    std::vector<Region> sample_regions;
    for (auto v:cur_frame_->contourX2D) {
        sample_regions.emplace_back(v,10);
    }
    cur_frame_->ComputePosterior(sample_regions);
}


void Tracker::ProcessFrame(FramePtr cur_frame) {
    last_frame_ = cur_frame_;
    cur_frame_ = cur_frame;
    //TODO:...
    if(last_frame_.use_count()!=0) {
        cur_pose_ = last_frame_->m_pose;
        cur_frame_->m_pose = last_frame_->m_pose;
    }
    //
//    cur_frame_->m_pose = cur_frame_->gt_Pose;

    model_.getContourPointsAndIts3DPoints(
            cur_pose_,cur_frame_->VerticesNear2ContourX3D,
            cur_frame_->VerticesNear2ContourX2D,
            cur_frame_->contourX2D);
    cur_frame_->Segment();
    std::vector<Region> sample_regions;
    for (auto v:cur_frame_->contourX2D) {
        sample_regions.emplace_back(v,10);
    }
    last_frame_->ComputePosterior(sample_regions);
    cur_frame_->ComputePosterior(sample_regions);
    cur_frame_->fw_posterior = last_frame_->fw_posterior*0.9+cur_frame_->fw_posterior*0.1;
    cur_frame_->bg_posterior = last_frame_->bg_posterior*0.8+cur_frame_->bg_posterior*0.2;
    Mat post_map = cur_frame_->fw_posterior > cur_frame_->bg_posterior;
//    post_map = post_map*255;


    cur_frame_->DTMap();

    //that's the result we want
//    ceresSolver.SolveByNumericDiffCostFunction(model_,cur_frame,last_frame_);
//    ceresSolver.SolveByCostFunctionWithJac(model_, cur_frame_, last_frame_);
    cur_pose_ = Pose(cur_frame_->m_pose);
    Mat out = cur_frame_->img.clone();
    model_.displayCV(cur_frame_->m_pose,{0,255,0},out);
    imshow("initial",out);
    imshow("result",post_map);
    waitKey(0);
}


/*
 *
 *
// ceres_test.cpp: 定义控制台应用程序的入口点。
//

#include <ceres/ceres.h>
#include <sophus/se3.hpp>

using namespace ceres;
using namespace Sophus;

struct CostFunctor {
	CostFunctor(double x,double y,double fwd,double bg_)
		: x_(x),y_(y),fwd_(fwd) {}

	template <typename T>
	bool operator()(const T* const pose, T* residual) const {
		residual[0] = T(y_) - (ceres::exp(m[0] * T(x_) + c[0]));
		return true;
	}

private:
	// 观测值
	const double x_;
	const double y_;
	const double fwd_;
	const double bg_;
};

int main(int argc, char** argv)
{
	google::InitGoogleLogging(argv[0]);

	double m = 1.0;
	double c = 1.0;

	double data[] = { 1,2,2,4,3,8,4,16,5,32 };

	Problem problem;
	for (int i = 0; i < 5; ++i)
	{
		CostFunction* cost_function =
			new AutoDiffCostFunction<CostFunctor, 1, 1, 1>(
				new CostFunctor(data[2 * i], data[2 * i + 1]));
		problem.AddResidualBlock(cost_function, NULL, &m, &c);
	}

	// 求解方程!
	Solver::Options options;
	options.line_search_direction_type = LBFGS;
	options.minimizer_type = LINE_SEARCH;
	options.linear_solver_type = ceres::CGNR;
	options.minimizer_progress_to_stdout = true;
	Solver::Summary summary;
	Solve(options, &problem, &summary);

	std::cout << summary.BriefReport() << "\n";
	std::cout << "m : " << m
		<< " c " << c << "\n";
	return 0;
}

 */
