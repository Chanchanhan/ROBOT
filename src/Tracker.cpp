//
// Created by flamming on 2018/1/13.
//

#include "Tracker.h"
#include "GlobalConfig.h"
#include "Region.h"

using namespace cv;
void Tracker::init(const OcvYamlConfig& ocvYamlConfig) {
    model_.loadObj(Config::configInstance().objFile);
}



void Tracker::ProcessFrame(FramePtr cur_frame) {
    last_frame_ = cur_frame;
    cur_frame_ = cur_frame;

    //TODO:...
    cur_pose_ = cur_frame->gt_Pose;
//
    model_.getContourPointsAndIts3DPoints(
            cur_pose_,cur_frame_->VerticesNear2ContourX3D,
            cur_frame_->VerticesNear2ContourX2D,
            cur_frame_->contourX2D);
    cur_frame_->Segment();
    cur_frame_->ComputePrior();
    vector<Region> sample_regions;
    for (auto v:cur_frame_->contourX2D) {
        sample_regions.emplace_back(v,5);
        sample_regions.back().UpdatingHistorgram(cur_frame_);
    }

    imshow("result",cur_frame_->segmentation);
    waitKey(1);




}

