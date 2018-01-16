//
// Created by flamming on 2018/1/13.
//

#ifndef ROBOT_SYSTEM_H
#define ROBOT_SYSTEM_H

#include <memory>
#include "DataPaser.h"
#include "Model.h"
#include "Solver.h"

class Tracker {
public:
    void init(const OcvYamlConfig& ocvYamlConfig);
    void ProcessFrame(FramePtr f);
    void ProcessFirstFrame(FramePtr cur_frame);

private:
    std::unique_ptr<DataPaser> parser_;
    Model model_;
    CeresSolver ceresSolver;
    Pose cur_pose_;
    FramePtr last_frame_;
    FramePtr cur_frame_;
};


#endif //ROBOT_SYSTEM_H
