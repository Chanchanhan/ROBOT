//
// Created by flamming on 2018/1/13.
//

#include "Tracker.h"
#include "GlobalConfig.h"

using namespace cv;
void Tracker::init(const OcvYamlConfig& ocvYamlConfig) {
    parser_ = std::make_unique<DataPaser>(ocvYamlConfig);
    model_.loadObj(Config::configInstance().objFile);
}



void Tracker::ProcessFrame(FramePtr cur_frame) {
    cur_pose = cur_frame->gt_Pose;

    cur_frame->contourX2D = model_.GetContourAt(cur_pose);
    cur_frame->Segment();
    cur_frame->ComputePrior();

    imshow("result",cur_frame->segmentation);
    waitKey(1);
//
//    model_.getContourPointsAndIts3DPoints(
//            cur_pose,cur_frame->VerticesNear2ContourX3D,
//            cur_frame->VerticesNear2ContourX2D,
//            cur_frame->contourX2D);



}

void Tracker::run() {

    while(1)
    {
        FramePtr cur_frame(new Frame);
        if(!parser_->parseAFrame(cur_frame))
            break;
        ProcessFrame(cur_frame);
    }

}
