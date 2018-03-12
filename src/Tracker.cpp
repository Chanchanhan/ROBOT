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
//    model_.getContourPointsAndIts3DPoints(
//            cur_pose_,cur_frame_->VerticesNear2ContourX3D,
//            cur_frame_->VerticesNear2ContourX2D,
//            cur_frame_->contourX2D);
//    cur_frame_->Segment();
//    cur_frame_->DTMap();
//    std::vector<Region> sample_regions;
//    for (auto v:cur_frame_->contourX2D) {
//        sample_regions.emplace_back(v,10);
//    }
//  cur_frame_->fw_posteriorPyramid.resize(Config::configInstance().IMG_PYR_NUMBER);
    cur_frame_->GetPyraid(Config::configInstance().IMG_PYR_NUMBER);

    // cur_frame_->ComputePosterior(sample_regions);
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

    for(int iLevel=Config::configInstance().IMG_PYR_NUMBER-1;iLevel>=0;iLevel--){
        model_.getContourPointsAndIts3DPoints(
                cur_pose_,cur_frame_->VerticesNear2ContourX3D,
                cur_frame_->VerticesNear2ContourX2D,
                cur_frame_->contourX2D,iLevel);
        cv::Mat segment,boundMap;
        std::vector<cv::Point> contourX2D;
        cur_frame_->Segment(cur_frame_->imgPyramid[iLevel],cur_frame_->contourX2D, cur_frame_->segmentation,cur_frame_->bound_map);
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
        cv::imshow("dt",cur_frame_->dt);
        cv::waitKey(0);
        //that's the result we want
        //ceresSolver.SolveByNumericDiffCostFunction(model_,cur_frame,last_frame_);
        ceresSolver.SolveByCostFunctionWithJac(model_, cur_frame_);
        cur_pose_ = cur_frame_->m_pose;
        Mat out = cur_frame_->img.clone();
        model_.displayCV(cur_frame_->m_pose,{0,255,0},out);
        imshow("initial",out);
        imshow("result",post_map);
        waitKey(0);
    }

//    model_.getContourPointsAndIts3DPoints(
//            cur_pose_,cur_frame_->VerticesNear2ContourX3D,
//            cur_frame_->VerticesNear2ContourX2D,
//            cur_frame_->contourX2D);
//
//    cur_frame_->Segment();
//    std::vector<Region> sample_regions;
//    for (auto v:cur_frame_->contourX2D) {
//        sample_regions.emplace_back(v,10);
//    }
//    last_frame_->ComputePosterior(sample_regions);
//    cur_frame_->ComputePosterior(sample_regions);
//    cur_frame_->fw_posterior = last_frame_->fw_posterior*0.9+cur_frame_->fw_posterior*0.1;
//    cur_frame_->bg_posterior = last_frame_->bg_posterior*0.8+cur_frame_->bg_posterior*0.2;
//    Mat post_map = cur_frame_->fw_posterior > cur_frame_->bg_posterior;
////    post_map = post_map*255;
//
//
//    cur_frame_->DTMap();
//
//    //that's the result we want
////    ceresSolver.SolveByNumericDiffCostFunction(model_,cur_frame,last_frame_);
//    ceresSolver.SolveByCostFunctionWithJac(model_, cur_frame_);
//    cur_pose_ = cur_frame_->m_pose;
//    Mat out = cur_frame_->img.clone();
//    model_.displayCV(cur_frame_->m_pose,{0,255,0},out);
//    imshow("initial",out);
//    imshow("result",post_map);
//    waitKey(0);
}
