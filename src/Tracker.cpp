//
// Created by flamming on 2018/1/13.
//
#include <ceres/ceres.h>
#include <opencv2/core/eigen.hpp>
#include "Tracker.h"
#include "GlobalConfig.h"
#include "Region.h"
#include "MySolver.h"
using namespace cv;
using namespace std;
using namespace ceres;

void Tracker::init(const OcvYamlConfig& ocvYamlConfig) {
    model_.LoadObj(Config::configInstance().objFile);
}

void Tracker::ProcessFirstFrame(FramePtr cur_frame)
{
    cur_frame_ = cur_frame;
    cur_frame_->m_pose = cur_frame->gt_Pose;
    cur_frame_->GetPyraid(Config::configInstance().IMG_PYR_NUMBER);
}

void Tracker::ProcessFrame(FramePtr cur_frame) {
    last_frame_ = cur_frame_;
    cur_frame_ = cur_frame;
    cur_frame_->GetPyraid(Config::configInstance().IMG_PYR_NUMBER);


    if(last_frame_.use_count()!=0) {
        cur_pose_ = last_frame_->m_pose;
        cur_frame_->m_pose = last_frame_->m_pose;
    }
    for(int iLevel=Config::configInstance().IMG_PYR_NUMBER-1;iLevel>=0;iLevel--){
#ifdef   PROJECT_WITH_GT
        model_.GetContourPointsAndIts3DPoints(
                cur_frame->gt_Pose, cur_frame_->VerticesNear2ContourX3D,
                cur_frame_->VerticesNear2ContourX2D,
                cur_frame_->gt_contourX2D, iLevel);

#endif

        model_.GetContourPointsAndIts3DPoints(
                cur_pose_, cur_frame_->VerticesNear2ContourX3D,
                cur_frame_->VerticesNear2ContourX2D,
                cur_frame_->contourX2D, iLevel);

        if(cur_frame_->contourX2D.size()==0||cur_frame_->VerticesNear2ContourX3D.size()==0)
            continue;
        cv::Mat segment,boundMap;
        std::vector<cv::Point> contourX2D;
#ifdef   PROJECT_WITH_GT
        cur_frame_->Segment(cur_frame_->imgPyramid[iLevel],cur_frame_->gt_contourX2D, cur_frame_->segmentation,cur_frame_->bound_map);
        last_frame_->Segment(last_frame_->imgPyramid[iLevel],last_frame_->gt_contourX2D, last_frame_->segmentation,last_frame_->bound_map);

#else
        cur_frame_->Segment(cur_frame_->imgPyramid[iLevel],cur_frame_->contourX2D, cur_frame_->segmentation,cur_frame_->bound_map);
        last_frame_->Segment(last_frame_->imgPyramid[iLevel],last_frame_->contourX2D, last_frame_->segmentation,last_frame_->bound_map);

#endif



        std::vector<Region> sample_regions;
        for (auto v:cur_frame_->contourX2D) {
            sample_regions.emplace_back(v,8);
        }


        last_frame_->ComputePosterior(last_frame_->imgPyramid[iLevel],sample_regions);
        cur_frame_->ComputePosterior(cur_frame_->imgPyramid[iLevel],sample_regions);

        cur_frame_->fw_posterior = last_frame_->fw_posterior*0.9+cur_frame_->fw_posterior*0.1;
        cur_frame_->bg_posterior = last_frame_->bg_posterior*0.8+cur_frame_->bg_posterior*0.2;

        Mat post_map = cur_frame_->fw_posterior > cur_frame_->bg_posterior;


        cur_frame_->DTMap();

        Mat input = cur_frame_->img.clone();
        model_.DisplayCV(cur_frame_->m_pose, {0, 255, 0}, input);
        imshow("input",input);
        //to solve
        MySolver mySolver(model_.intrinsics[iLevel]);
        mySolver.Solve(cur_frame_, model_,iLevel);
        //draw points
        model_.DrawPoints(cur_frame_->m_pose,cur_frame_->VerticesNear2ContourX3D,*cur_frame_,iLevel);


        //draw out
        Mat out = cur_frame_->img.clone();
        model_.DisplayCV(cur_frame_->m_pose, {0, 255, 0}, out);
        imshow("outPut",out);
        imshow("result",post_map);
        waitKey(0);
    }

}
