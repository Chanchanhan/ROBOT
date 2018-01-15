//
// Created by flamming on 2018/1/12.
//
#include <Frame.h>

#include "GlobalConfig.h"
#include "DT.h"

#include "Region.h"


using namespace cv;
using namespace std;
void Frame::Segment()
{
    vector<vector<Point> > contours;
    contours.push_back(this->contourX2D);
    this->segmentation = Mat::zeros(this->img.rows,this->img.cols,CV_8U);
    cv::drawContours(this->segmentation, contours,-1, CV_RGB(255, 255, 255), CV_FILLED);
}

void Frame::ComputePrior()
{
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
void Frame::ComputePosterior(const std::vector<Region>& rg)
{
    int n = rg.size();
    fw_posterior = Mat::zeros(img.size(),CV_64F) ;
    bg_posterior = Mat::zeros(img.size(),CV_64F) ;
    Mat occur = Mat::zeros(img.size(),CV_16S);
    for(auto s:rg)
    {
        s.UpdateHistorgram(this);
        int i = 1;
        for (; i < s.circle_bound_.size()-1; i++) {
            int left = s.circle_bound_[i].x;
            while(s.circle_bound_[i+1].y == s.circle_bound_[i].y)
                i++;
            auto y = s.circle_bound_[i].y;
            for (int x = left; x < s.circle_bound_[i].x; ++x) {
                auto& p = img.at<Vec3b>(y,x);
                occur.at<short>(y,x)++;
                auto ar = s.aera;
                fw_posterior.at<double>(y,x) += s.prior_fw * (s.fwd.B[p[0]]/ar * s.fwd.G[p[1]]/ar *s.fwd.R[p[2]]/ar);
                bg_posterior.at<double>(y,x) += s.prior_bg * (s.bg.B[p[0]]/ar * s.bg.G[p[1]]/ar *s.bg.R[p[2]]/ar);
            }
        }
    }
    auto p1 = occur.ptr<short>(0);
    auto f = fw_posterior.ptr<double>(0);
    auto b = bg_posterior.ptr<double>(0);

    for (int j = 0; j < occur.size().area(); ++j) {
        *f/=*p1;
        *b/=*p1;
        p1++;
        f++;
        b++;
    }
}


