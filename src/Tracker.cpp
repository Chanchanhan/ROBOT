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

struct CostFunctor
{
    CostFunctor(vector<Point3d>& X,Mat& fwd,Mat& bg,Mat& dt_map,Sophus::Matrix3d& K):X_(X),fwd_(fwd),bg_(bg),dt_map_(dt_map),K_(K) {}
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

    const vector<Point3d> X_;
    const Mat dt_map_;
    const Mat fwd_;
    const Mat bg_;
    const Sophus::Matrix3d K_;
};

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
    std::vector<Region> sample_regions;
    for (auto v:cur_frame_->contourX2D) {
        sample_regions.emplace_back(v,5);
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
    model_.getContourPointsAndIts3DPoints(
            cur_pose_,cur_frame_->VerticesNear2ContourX3D,
            cur_frame_->VerticesNear2ContourX2D,
            cur_frame_->contourX2D);
    cur_frame_->Segment();
    std::vector<Region> sample_regions;
    for (auto v:cur_frame_->contourX2D) {
        sample_regions.emplace_back(v,5);
    }
    cur_frame_->ComputePosterior(sample_regions);
    last_frame_->ComputePosterior(sample_regions);
    cur_frame_->fw_posterior = last_frame_->fw_posterior*0.9+cur_frame_->fw_posterior*0.1;
    cur_frame_->bg_posterior = last_frame_->bg_posterior*0.8+cur_frame_->bg_posterior*0.2;
    Mat post_map = cur_frame_->fw_posterior > cur_frame_->bg_posterior;
    post_map = post_map*255;


    cur_frame_->DTMap();
    Sophus::Matrix3d KK;

    cv2eigen(model_.intrinsic,KK);

    auto so3_ = last_frame_->m_pose.m_pose.so3().log();
    Sophus::Vector3d& t3_ = last_frame_->m_pose.m_pose.translation();

    double pose_initial[6] = {so3_(0),so3_(1),so3_(2),t3_(0),t3_(1),t3_(2)};
    for (int i = 0; i < 6; ++i) {
        cout<<pose_initial[i]<<endl;
    }
    ceres::Problem min_enery;

    CostFunction* cost_function =
            new NumericDiffCostFunction<CostFunctor, RIDDERS,1, 6>(new CostFunctor(
                    cur_frame_->VerticesNear2ContourX3D,
                    cur_frame_->fw_posterior,
                    cur_frame_->bg_posterior,
                    cur_frame_->dt,
                    KK
            ));
    min_enery.AddResidualBlock(cost_function, NULL, pose_initial);
//
    // 求解方程!
    Solver::Options options;
    options.line_search_direction_type = LBFGS;
    options.minimizer_type = LINE_SEARCH;
    options.linear_solver_type = ceres::CGNR;
    options.minimizer_progress_to_stdout = true;
    Solver::Summary summary;
    Solve(options, &min_enery, &summary);

    std::cout << summary.BriefReport() << "\n";
    for (int i = 0; i < 6; ++i) {
        cout<<pose_initial[i]<<endl;
    }

    //that's the result we want
    cur_frame_->m_pose = Pose(pose_initial);
    cur_pose_ = Pose(pose_initial);
    imshow("initial",cur_frame_->img);
    imshow("result",post_map);
    waitKey(1);
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
