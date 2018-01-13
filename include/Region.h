//
// Created by flamming on 2018/1/14.
//

#ifndef ROBOT_REGION_H
#define ROBOT_REGION_H

#include <opencv2/opencv.hpp>
#include "Frame.h"

struct Histogram
{
    int R[255]{};
    int G[255]{};
    int B[255]{};
};

void BresenhamCircle(cv::Point2i veterx,int radius,std::vector<cv::Point2i>& sampleset);

class Region {
public:
    Region(){};
    Region(int r,cv::Point center);
    cv::Point center_;
    int radius_;
    Histogram bg;
    Histogram fwd;
    void UpdatingHistorgram(const Frame& curFrame);
    void VizHistImg(const Histogram& img);
    std::vector<cv::Point> circle_bound_;
};


#endif //ROBOT_REGION_H
