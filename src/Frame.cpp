//
// Created by flamming on 2018/1/12.
//
#include <Frame.h>
#include "GlobalConfig.h"
#include "DT.h"
using namespace cv;
using namespace std;
void Frame::Segment()
{
    vector<vector<Point> > contours;
    contours.push_back(this->contourX2D);
    this->segmentation = Mat::zeros(this->img.rows,this->img.cols,CV_8U);
    cv::drawContours(this->segmentation, contours,-1, CV_RGB(255, 255, 255), CV_FILLED);
}

void Frame::ComputePrior() {
    int iVal255 = countNonZero(segmentation);
    fw_prior = iVal255*1.0/(segmentation.cols*segmentation.rows);
    bg_prior = 1-fw_prior;
}

void Frame::DTMap() {
    vector<float> weights;
    weights.push_back(Config::configInstance().IMG_DT_WEIGHT);
    weights.push_back(Config::configInstance().IMG_DT_WEIGHT);
    distanceTransform(segmentation,dt,dtLocation,weights);
}
cv::Point Frame::nearstContourP(const cv::Point &point) {
    int x= point.y;
    int y= point.x;

    int *_locations=(int *)dtLocation.data;
    Config &gConfig = Config::configInstance();

    while(_locations[y + gConfig.VIDEO_WIDTH * x]!=x||_locations[gConfig.VIDEO_HEIGHT *gConfig.VIDEO_WIDTH+y + gConfig.VIDEO_WIDTH * x]!=y){
        x=_locations[y + gConfig.VIDEO_WIDTH * x];
        y=_locations[gConfig.VIDEO_HEIGHT *gConfig.VIDEO_WIDTH+y + gConfig.VIDEO_WIDTH* x];
    }
    return Point(y,x);
}