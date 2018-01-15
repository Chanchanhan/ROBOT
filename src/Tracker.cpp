//
// Created by flamming on 2018/1/13.
//

#include "Tracker.h"
#include "GlobalConfig.h"
#include "Region.h"

using namespace cv;
using namespace std;

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
    std::vector<Region> sample_regions;
    for (auto v:cur_frame_->contourX2D) {
        sample_regions.emplace_back(v,5);
        sample_regions.back().UpdateHistorgram(cur_frame_);
    }
    cur_frame_->ComputePosterior(sample_regions);
    Mat post_map = cur_frame_->fw_posterior > cur_frame_->bg_posterior;
    post_map = post_map*255;

    imshow("initial",cur_frame_->img);
    imshow("result",post_map);
    waitKey(1);




}

