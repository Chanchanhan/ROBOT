//
// Created by flamming on 2018/1/13.
//

#ifndef ROBOT_SYSTEM_H
#define ROBOT_SYSTEM_H

#include <memory>
#include "DataPaser.h"
#include "Model.h"

class Tracker {
public:
    void init(const OcvYamlConfig& ocvYamlConfig);
    void run();
    void ProcessFrame(FramePtr f);

private:
    std::unique_ptr<DataPaser> parser_;
    Model model_;
    Pose cur_pose;
};


#endif //ROBOT_SYSTEM_H
