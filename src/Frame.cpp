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
    this->bound_map = Mat::zeros(this->img.rows,this->img.cols,CV_32F);
    cv::drawContours(this->segmentation, contours,-1, CV_RGB(255, 255, 255), CV_FILLED);
    cv::drawContours(this->bound_map, contours,-1, CV_RGB(255, 255, 255));
//    imshow("bound",this->bound_map);
}


void Frame::DTMap() {
    vector<float> weights;
    weights.push_back(Config::configInstance().IMG_DT_WEIGHT);
    weights.push_back(Config::configInstance().IMG_DT_WEIGHT);
    distanceTransform(this->bound_map,dt,dtLocation,weights);/// it's wired
    cout << dt<<endl;
    imshow("dt",dt*255);
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
                auto fw_likelihood = (s.fwd.B[p[0]]/ar * s.fwd.G[p[1]]/ar *s.fwd.R[p[2]]/ar);
                auto bg_likelihood = (s.bg.B[p[0]]/ar * s.bg.G[p[1]]/ar *s.bg.R[p[2]]/ar);
                auto btm = s.prior_fw * fw_likelihood + s.prior_bg*bg_likelihood;
                fw_posterior.at<double>(y,x) += fw_likelihood /btm;
                bg_posterior.at<double>(y,x) += bg_likelihood / btm ;
            }
        }
    }
    auto p1 = occur.ptr<short>(0);
    auto f = fw_posterior.ptr<double>(0);
    auto b = bg_posterior.ptr<double>(0);
    auto seg = segmentation.ptr<unsigned char>(0);
    auto oc = occur.ptr<short>(0);

    for (int j = 0; j < occur.size().area(); ++j) {
        if(*oc==0&&(*seg))
        {
            *f = 1;
            *b = 0;
        }
        else if(*oc==0&&!(*seg)) {
            *f = 0;
            *b = 1;
        }
        else
        {
            *f /= *p1;
            *b /= *p1;
        }
        p1++;
        f++;
        b++;
        seg++;
        oc++;
    }
}


