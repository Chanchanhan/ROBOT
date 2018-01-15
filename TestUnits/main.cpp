#include <iostream>
#include "Tracker.h"
#include <glog/logging.h>

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

