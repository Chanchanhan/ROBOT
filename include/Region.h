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
    Histogram operator+(const Histogram& other)
    {
        Histogram r;
        for (int i = 0; i < 255; ++i) {
            r.B[i] = this->B[i]+other.B[i];
            r.G[i] = this->G[i]+other.G[i];
            r.R[i] = this->R[i]+other.R[i];
        }
        return r;
    }
};

void BresenhamCircle(cv::Point2i veterx,int radius,std::vector<cv::Point2i>& sampleset);

class Region {
public:
    Region(){};
    Region(cv::Point center,int r);
    cv::Point center_;
    int radius_;
    Histogram bg;
    Histogram fwd;
    double n_fw{};
    double n_bg{};
    double aera;


    void UpdateHistorgram(FramePtr curFrame);
    void UpdateHistorgram(Frame* curFrame);
    void VizHistImg(const Histogram& img);
    std::vector<cv::Point> circle_bound_;
};


#endif //ROBOT_REGION_H
