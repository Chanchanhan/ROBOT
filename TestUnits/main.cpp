#include <iostream>
#include "Tracker.h"
#include <glog/logging.h>

using namespace std;
using namespace cv;
int main(int argc, char* argv[])
{
//    vector<cv::Point2i> region;
//    BresenhamCircle({150,150},55,region);
//    Mat draw = imread("/Users/flamming/Downloads/MarkerAR550/Assets/Resources/marker3.jpg");
//    Mat out = draw.clone();
//    for(auto& p : region)
//    {
//        out.at<Vec3b>(p.x,p.y)[0] = 0;
//        out.at<Vec3b>(p.x,p.y)[1] = 255;
//        out.at<Vec3b>(p.x,p.y)[2] = 0;
//    }
//    Frame f;
//    f.img = draw;
//    f.segmentation = Mat::zeros(draw.rows,draw.cols,CV_8U);
//    vector<vector<cv::Point2i>> regions;
//    regions.push_back(region);
//    drawContours(f.segmentation,regions,-1,cv::Scalar(255,255,255),CV_FILLED);
//    Histogram fore,bg;
//    UpdatingHistorgram(f,region,fore,bg);
//    imshow("result",draw);
//    imshow("result",f.segmentation);
//
//    VizHistImg(bg);
//
//
//
//    cout << "ROBOT:region based glm tracking" << endl;

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

