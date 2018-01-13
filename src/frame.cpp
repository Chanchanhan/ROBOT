//
// Created by flamming on 2018/1/12.
//
#include <frame.h>

using namespace cv;

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

